#pragma once

#include "Types.hpp"
#include "GridMap.hpp"
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>

/// A* planner with obstacle inflation + elastic band + Catmull-Rom smoothing.
/// Set allowUnknown=true to plan through uncharted territory (for GO_HOME).
class PathPlanner {
public:
    std::vector<Point> plan(const GridMap& map, Point start, Point goal,
                            bool allowUnknown = false, int stride = 1, bool do_smooth = true) const
    {
        auto raw = astarRaw(map, start, goal, allowUnknown, stride);
        // ★步长>1: 最后一段"粗格末端 → 精确目标点"单独用 1 格搜索补上 ✓
        if (stride > 1 && raw.size() >= 2) {
            Point last = raw.back();
            if (std::hypot(last.x - goal.x, last.y - goal.y) > 1e-3) {
                auto tail = astarRaw(map, last, goal, allowUnknown, 1);
                if (tail.size() >= 2) raw.insert(raw.end(), tail.begin() + 1, tail.end());
            }
        }
        if (raw.size() < 2) return {};
        auto pruned = prunePath(map, raw, allowUnknown);          // LOS 剪枝
        // ★★ do_smooth=false: 只到剪枝就返回 —— 用于"选探索点"的候选评估 ✓
        //    EGO 梯度优化是这段的重头(几百 ms~秒级 ✗), 而选点只需要【路径长度+离障】
        //    来算分数 ✓ → 跳过后选点快 10 倍以上 ✓ (胜出后由跟随器 fromPath 建样条 ✓)
        if (!do_smooth) {
            for (auto& p : pruned) {
                p.x = std::clamp(p.x, MAP_X_MIN + DRONE_RADIUS, MAP_X_MAX - DRONE_RADIUS);
                p.y = std::clamp(p.y, MAP_Y_MIN + DRONE_RADIUS, MAP_Y_MAX - DRONE_RADIUS);
            }
            return pruned;
        }
        auto optimized = egoOptimize(map, pruned, allowUnknown);  // EGO 梯度优化
        auto smooth = interpolatePath(map, optimized, allowUnknown); // Catmull-Rom 加密
        for (auto& p : smooth) {
            p.x = std::clamp(p.x, MAP_X_MIN + DRONE_RADIUS, MAP_X_MAX - DRONE_RADIUS);
            p.y = std::clamp(p.y, MAP_Y_MIN + DRONE_RADIUS, MAP_Y_MAX - DRONE_RADIUS);
        }
        return smooth;
    }

    /// 保守规划(窄走廊/A→B 专用): A* → LOS 剪枝 → 直线加密(GRID_RES/2),
    /// **不做** EGO 优化和 Catmull-Rom 平滑。
    /// 原因: 后两步会把路径从"A* 保证可走"的栅格自由区往外拉, 于是
    /// interpolatePath 里大量采样点被判定不安全而跳过, 留下几百毫米的"跳点"
    /// 直线段 —— 这些段本身没被校验过, 在 0.75~0.8m 宽的窄口里会贴到真实墙面上
    /// (实测真实间隙只剩 1cm, 而机身半径 0.25m)。剪枝后的每一段直线都经过
    /// lineClear() 逐点 cellWalkable 校验, 加密后再由跟踪器直着飞。
    std::vector<Point> planConservative(const GridMap& map, Point start, Point goal,
                                        bool allowUnknown = false,
                                        double push_target = -1.0) const
    {
        auto raw = astarRaw(map, start, goal, allowUnknown);
        if (raw.size() < 2) return {};
        auto pruned = prunePath(map, raw, allowUnknown);
        // 可选: 把路径推离障碍到 push_target(m). 栅格 A* 的最短路一定贴着
        // "可走约束"的边界走 → 真实间隙永远只有 边缘距离-栅格量化; 推开之后
        // (在窄口里等于被两侧墙一起推到通道中间)真实间隙能到 push_target 附近。
        if (push_target > 0.0) {
            auto pushed = pushAwayFromObstacles(map, pruned, push_target, allowUnknown);
            if (pushed.size() >= 2) pruned.swap(pushed);
        }
        std::vector<Point> dense;
        dense.reserve(pruned.size() * 8);
        dense.push_back(pruned.front());
        const double STEP = GRID_RES * 0.5;   // 5cm
        for (size_t i = 0; i + 1 < pruned.size(); ++i) {
            double seg = pruned[i].dist(pruned[i + 1]);
            int n = std::max(1, (int)std::lround(seg / STEP));
            for (int k = 1; k <= n; ++k)
                dense.emplace_back(pruned[i].x + (pruned[i + 1].x - pruned[i].x) * k / n,
                                   pruned[i].y + (pruned[i + 1].y - pruned[i].y) * k / n);
        }
        return dense;
    }

    /// 通用"离障推进"(势场推开, 不识别窄口/不算中点): 对路径上每个点求"最近障碍
    /// 中心"的方向, 若距离 < target 就把该点沿反方向推到 target —— 只接受推完仍
    /// 满足 cellWalkable 的点。迭代 3 次后按 5cm 重新加密; 若加密中有任何点不可走,
    /// 直接放弃优化返回原路径(宁可不优化也不给出未校验的路径)。
    /// 效果: 在两侧都有墙的窄通道里, 点被两侧交替推开 → 自然落到通道中间。
    std::vector<Point> pushAwayFromObstacles(const GridMap& map, const std::vector<Point>& in,
                                             double target, bool allowUnknown) const
    {
        if (in.size() < 3) return in;
        const int R = (int)std::ceil(target / GRID_RES) + 2;
        std::vector<Point> ctrl = in;
        for (int it = 0; it < 3; ++it) {
            std::vector<Point> nxt = ctrl;
            for (size_t i = 1; i + 1 < ctrl.size(); ++i) {
                auto nd = nearestObstacle(map, ctrl[i].x, ctrl[i].y, R);
                double d = nd.first;
                if (d >= target || d <= 1e-6) continue;
                Point cand(ctrl[i].x + nd.second.x * (target - d),
                           ctrl[i].y + nd.second.y * (target - d));
                if (cellWalkable(map, worldToGrid(cand.x, cand.y), allowUnknown))
                    nxt[i] = cand;
            }
            ctrl.swap(nxt);
        }
        // 重新加密 + 逐点校验
        std::vector<Point> dense;
        dense.reserve(ctrl.size() * 8);
        dense.push_back(ctrl.front());
        const double STEP = GRID_RES * 0.5;
        for (size_t i = 0; i + 1 < ctrl.size(); ++i) {
            double seg = ctrl[i].dist(ctrl[i + 1]);
            int n = std::max(1, (int)std::lround(seg / STEP));
            for (int k = 1; k <= n; ++k) {
                Point p(ctrl[i].x + (ctrl[i + 1].x - ctrl[i].x) * k / n,
                        ctrl[i].y + (ctrl[i + 1].y - ctrl[i].y) * k / n);
                if (!cellWalkable(map, worldToGrid(p.x, p.y), allowUnknown)) return in;
                dense.push_back(p);
            }
        }
        return dense;
    }

private:
    /// 最近障碍栅格中心: 返回 {距离(m), 从该点指向远离障碍方向的单位向量}。
    static std::pair<double, Point> nearestObstacle(const GridMap& map,
                                                    double wx, double wy, int R) {
        GridPos gp = worldToGrid(wx, wy);
        double best = std::numeric_limits<double>::max();
        Point dir(0, 0);
        for (int dy = -R; dy <= R; ++dy)
            for (int dx = -R; dx <= R; ++dx) {
                if (dx == 0 && dy == 0) continue;
                if (map.get(gp.x + dx, gp.y + dy) != CS_OCCUPIED) continue;
                double d = std::sqrt(double(dx*dx + dy*dy)) * GRID_RES;
                if (d < best) { best = d; dir = Point(-dx*GRID_RES/d, -dy*GRID_RES/d); }
            }
        return {best, dir};
    }

    /// 纯 A*(含起终点 nearestFree 修正) → 返回栅格中心连成的原始路径。

    /// ★步长 A* 用: 检查 a→b 这条直线上的【每一格】都可走(逐格步进, 不跳格)
    static bool segmentWalkable(const GridMap& map, GridPos a, GridPos b, bool allowUnknown, bool relaxed) {
        int dx = b.x - a.x, dy = b.y - a.y;
        int n = std::max(std::abs(dx), std::abs(dy));
        if (n <= 0) return true;
        for (int i = 1; i <= n; ++i) {
            GridPos c{ a.x + (int)std::lround((double)dx * i / n),
                       a.y + (int)std::lround((double)dy * i / n) };
            bool ok = relaxed ? isTraversableCell(map, c, allowUnknown)
                              : cellWalkable(map, c, allowUnknown);
            if (!ok) return false;
        }
        return true;
    }

    std::vector<Point> astarRaw(const GridMap& map, Point start, Point goal,
                                bool allowUnknown, int stride = 1) const{
        // --- 1) A* with inflated obstacles ---
        GridPos gs = worldToGrid(start.x, start.y);
        GridPos gg = worldToGrid(goal.x, goal.y);

        gs = nearestFree(map, gs, allowUnknown);
        gg = nearestFree(map, gg, allowUnknown);
        if (!gs.valid() || !gg.valid()) return {};

        using Key = int;
        auto encode = [](GridPos p) -> Key { return p.y * GRID_N + p.x; };

        struct Node { Key key; GridPos pos; double g, f; Key parent; bool closed; };
        std::unordered_map<Key, Node> nodes;
        nodes.reserve(GRID_N * GRID_N);  // never rehash — keeps Node* stable

        auto cmp = [](Node* a, Node* b) { return a->f > b->f; };
        std::priority_queue<Node*, std::vector<Node*>, decltype(cmp)> open(cmp);

        Key start_key = encode(gs);
        nodes[start_key] = {start_key, gs, 0, heuristic(gs, gg), start_key, false};
        open.push(&nodes[start_key]);

        bool found = false;
        Key goal_key = encode(gg);

        while (!open.empty()) {
            Node* cur = open.top(); open.pop();
            if (cur->closed) continue;
            cur->closed = true;
            if (cur->key == goal_key) { found = true; break; }
            // ★步长>1: 目标点不一定落在粗格上 → 距目标 <= stride 格也收尾(最后一段单独补) ✓
            if (stride > 1) {
                int ax = std::abs(cur->pos.x - gg.x), ay = std::abs(cur->pos.y - gg.y);
                // ★★ 收尾必须做两件事(否则回溯从 goal_key 走不回去 → 空路径, 选点全失败 ✗):
                //   ① 当前节点→目标点的【直线逐格可走】(不许穿墙 ✗)
                //   ② 把目标点挂到当前节点下面(parent=cur->key), 并写入 nodes ✓
                if (ax <= stride && ay <= stride &&
                    segmentWalkable(map, cur->pos, gg, allowUnknown, false)) {
                    double gfin = cur->g + std::hypot((double)(gg.x - cur->pos.x), (double)(gg.y - cur->pos.y));
                    auto itg = nodes.find(goal_key);
                    if (itg == nodes.end())
                        nodes[goal_key] = {goal_key, gg, gfin, gfin, cur->key, false};
                    else { itg->second.g = gfin; itg->second.f = gfin;
                           itg->second.parent = cur->key; itg->second.closed = false; }
                    found = true; break;
                }
            }

            for (const auto& nb : neighbors8(cur->pos, stride)) {
                Key nk = encode(nb);

                // ★步长>1 时: 必须逐格检查【整段直线】上的每一格 ✗
                //   (否则会"跨过"一格宽的障碍/墙 → 出事)
                if (stride > 1 && !segmentWalkable(map, cur->pos, nb, allowUnknown,
                                                  cur->key == start_key)) continue;

                // Start cell: only require not-occupied; in relaxed mode allow FREE or UNKNOWN
                bool walkable = (cur->key == start_key)
                    ? isTraversableCell(map, nb, allowUnknown)
                    : cellWalkable(map, nb, allowUnknown);
                if (!walkable) continue;

                double step_cost = ((nb.x != cur->pos.x && nb.y != cur->pos.y)
                                   ? 1.414 : 1.0) * stride;
                double new_g = cur->g + step_cost;

                auto it = nodes.find(nk);
                if (it == nodes.end()) {
                    nodes[nk] = {nk, nb, new_g, new_g + heuristic(nb, gg), cur->key, false};
                    open.push(&nodes[nk]);
                } else if (!it->second.closed && new_g < it->second.g) {
                    it->second.g = new_g;
                    it->second.f = new_g + heuristic(nb, gg);
                    it->second.parent = cur->key;
                    open.push(&it->second);
                }
            }
        }
        if (!found) return {};

        // Reconstruct raw A* path
        std::vector<Point> raw;
        Key k = goal_key;
        while (k != start_key) {
            auto it = nodes.find(k);
            if (it == nodes.end()) break;
            raw.push_back(gridToWorld(it->second.pos.x, it->second.pos.y));
            k = it->second.parent;
        }
        raw.push_back(gridToWorld(gs.x, gs.y));
        std::reverse(raw.begin(), raw.end());
        return raw;
    }

    /// Smooth an existing geometric path using LOS pruning + EGO optimisation
    /// + Catmull-Rom interpolation.  Does NOT run A* — assumes the input path
    /// is already collision-free.
    std::vector<Point> smoothPath(const GridMap& map, const std::vector<Point>& raw,
                                  bool allowUnknown = false) const
    {
        if (raw.size() < 2) return raw;
        auto pruned    = prunePath(map, raw, allowUnknown);
        auto optimized = egoOptimize(map, pruned, allowUnknown);
        auto smooth    = interpolatePath(map, optimized, allowUnknown);
        for (auto& p : smooth) {
            p.x = std::clamp(p.x, MAP_X_MIN + DRONE_RADIUS, MAP_X_MAX - DRONE_RADIUS);
            p.y = std::clamp(p.y, MAP_Y_MIN + DRONE_RADIUS, MAP_Y_MAX - DRONE_RADIUS);
        }
        return smooth;
    }

private:
    // -----------------------------------------------------------
    //  Obstacle inflation check
    // -----------------------------------------------------------
    /// True if cell is traversable: the whole airframe fits with clearance.
    /// Safety radius about the cell centre that must contain no obstacle:
    ///   exploration region : INFLATE_RADIUS*GRID_RES + DRONE_RADIUS  (0.3+0.25 = 0.55 m)
    ///   A→B / non-explore  : AB_EDGE_MARGIN (launch, 默认 0.25 m) —— 走廊太窄,
    ///                         不加膨胀光晕, 只留一段边缘距离; A→B 也靠它过窄口。
    /// (exploration region already includes the body, so no separate halo.)
    /// 距探索区边界这么近的格子算"过渡带" → 通行判定用走廊边距(AB_EDGE_MARGIN)
    static constexpr double EXPLORE_BOUNDARY_BAND = 0.60;
    static bool inExploreBoundaryBand(const Point& wp) {
        if (!inExploreRegion(wp.x, wp.y)) return true;         // 本来就在走廊/非探索区
        return (wp.x - EXPLORE_X_MIN < EXPLORE_BOUNDARY_BAND) ||
               (EXPLORE_X_MAX - wp.x < EXPLORE_BOUNDARY_BAND) ||
               (wp.y - EXPLORE_Y_MIN < EXPLORE_BOUNDARY_BAND) ||
               (EXPLORE_Y_MAX - wp.y < EXPLORE_BOUNDARY_BAND);
    }

    static bool cellWalkable(const GridMap& map, GridPos p, bool allowUnknown = false) {
        if (!p.valid()) return false;
        Point wp = gridToWorld(p.x, p.y);
        if (wp.x < MAP_X_MIN + DRONE_RADIUS || wp.x > MAP_X_MAX - DRONE_RADIUS ||
            wp.y < MAP_Y_MIN + DRONE_RADIUS || wp.y > MAP_Y_MAX - DRONE_RADIUS)
            return false;
        CellState s = map.get(p);
        if (s == CS_OCCUPIED) return false;
        if (s == CS_UNKNOWN && !allowUnknown) return false;
        // CS_FREE and CS_FOV_FREE are both traversable

        // ★探索区"边界过渡带"内用走廊边距(小): 探索区通往降落通道的入口常常只有
        //   1.0~1.5m 宽, 若按探索区膨胀(0.55m)判定会被整条堵死 → 返航 leg 规划不出来,
        //   重试 20 次后原地降落(实测现象)。边界 0.6m 带内按走廊处理即可。
        const double clear_r = (inExploreRegion(wp.x, wp.y) && !inExploreBoundaryBand(wp))
                             ? INFLATE_RADIUS * GRID_RES + DRONE_RADIUS
                             : AB_EDGE_MARGIN;
        const double r2 = clear_r * clear_r;
        const int br = (int)std::ceil(clear_r / GRID_RES) + 1;   // +1 for the cell itself
        for (int dy = -br; dy <= br; ++dy) {
            for (int dx = -br; dx <= br; ++dx) {
                double dc2 = (dx*GRID_RES)*(dx*GRID_RES) + (dy*GRID_RES)*(dy*GRID_RES);
                if (dc2 > r2) continue;              // outside the safety circle
                if (map.get(p.x + dx, p.y + dy) == CS_OCCUPIED)
                    return false;
            }
        }
        return true;
    }

    /// Lax check for start-node exemption: FREE always ok, UNKNOWN ok in relaxed
    /// mode, OCCUPIED never ok.
    static bool isTraversableCell(const GridMap& map, GridPos p, bool allowUnknown) {
        CellState s = map.get(p);
        return s == CS_FREE || s == CS_FOV_FREE || (allowUnknown && s == CS_UNKNOWN);
    }

    // -----------------------------------------------------------
    //  Heuristic & neighbours
    // -----------------------------------------------------------
    static double heuristic(GridPos a, GridPos b) {
        double dx = a.x - b.x, dy = a.y - b.y;
        return std::sqrt(dx*dx + dy*dy);
    }

    static std::vector<GridPos> neighbors8(GridPos p, int stride = 1) {
        std::vector<GridPos> nbs;
        for (int dy = -stride; dy <= stride; dy += stride)
            for (int dx = -stride; dx <= stride; dx += stride) {
                if (dx == 0 && dy == 0) continue;
                GridPos nb(p.x + dx, p.y + dy);
                if (nb.valid()) nbs.push_back(nb);
            }
        return nbs;
    }

    static GridPos nearestFree(const GridMap& map, GridPos p, bool allowUnknown = false) {
        if (cellWalkable(map, p, allowUnknown)) return p;
        // Search for nearest cell that passes full inflation check
        for (int r = 1; r <= 50; ++r)
            for (int dy = -r; dy <= r; ++dy)
                for (int dx = -r; dx <= r; ++dx) {
                    GridPos nb(p.x + dx, p.y + dy);
                    if (nb.valid() && cellWalkable(map, nb, allowUnknown)) return nb;
                }
        return GridPos(-1, -1);
    }

    // -----------------------------------------------------------
    //  Line-of-sight pruning
    // -----------------------------------------------------------
    std::vector<Point> prunePath(const GridMap& map,
                                 const std::vector<Point>& raw,
                                 bool allowUnknown = false) const
    {
        if (raw.size() <= 2) return raw;
        std::vector<Point> pruned;
        pruned.push_back(raw.front());

        size_t i = 0;
        while (i < raw.size() - 1) {
            size_t farthest = i + 1;
            for (size_t j = raw.size() - 1; j > i; --j) {
                if (lineClear(map, raw[i], raw[j], allowUnknown)) {
                    farthest = j;
                    break;
                }
            }
            pruned.push_back(raw[farthest]);
            i = farthest;
        }
        return pruned;
    }

    bool lineClear(const GridMap& map, const Point& a, const Point& b,
                   bool allowUnknown = false) const
    {
        double dx = b.x - a.x, dy = b.y - a.y;
        double dist = std::sqrt(dx*dx + dy*dy);
        int steps = static_cast<int>(dist / (GRID_RES * 0.5));
        if (steps < 4) steps = 4;
        for (int i = 0; i <= steps; ++i) {
            double t = static_cast<double>(i) / steps;
            double wx = a.x + t * dx, wy = a.y + t * dy;
            if (!cellWalkable(map, worldToGrid(wx, wy), allowUnknown)) return false;
        }
        return true;
    }

    // ============================================================
    //  EGO-style gradient-descent path optimisation
    //    Inspired by EGO-Planner: cubic distance penalty + jerk
    //    smoothness, solved via iterative gradient descent.
    // ============================================================
    std::vector<Point> egoOptimize(const GridMap& map,
                                   const std::vector<Point>& path,
                                   bool allowUnknown) const
    {
        if (path.size() <= 2) return path;

        // ---- helpers ----
        // drone 0.4m□ half-diag≈0.283m; INFLATE_RADIUS must cover body + margin
        // ★改成按米(原先 (INFLATE_RADIUS+4)*GRID_RES 在 10cm 时=0.70m; 分辨率变 2.5cm 后
        //   若仍按格算会缩到 0.40m → 平滑后的路径会贴墙)。保持物理距离不变。
        const double SAFE_DIST = std::max(0.70, INFLATE_RADIUS * GRID_RES + DRONE_RADIUS + 0.15);
        constexpr double SMOOTH_W    = 0.8;
        constexpr double DIST_W      = 3.0;
        constexpr double STEP_INIT   = 0.04;
        constexpr int    MAX_ITER    = 200;
        const int    SR          = INFLATE_RADIUS * 2 + 4;

        auto nearestObs = [&](double wx, double wy) -> std::pair<double, Point> {
            GridPos gp = worldToGrid(wx, wy);
            double best_d = std::numeric_limits<double>::max();
            Point dir(0, 0);
            for (int dy = -SR; dy <= SR; ++dy) {
                for (int dx = -SR; dx <= SR; ++dx) {
                    if (dx == 0 && dy == 0) continue;
                    if (map.get(gp.x + dx, gp.y + dy) == CS_OCCUPIED) {
                        double d = std::sqrt(dx*dx + dy*dy) * GRID_RES;
                        if (d < best_d) {
                            best_d = d;
                            dir = Point(-dx * GRID_RES / d, -dy * GRID_RES / d);
                        }
                    }
                }
            }
            return {best_d, dir};
        };

        auto computeCostAndGrad = [&](const std::vector<Point>& ctrl,
                                       std::vector<Point>& grad,
                                       double& total_cost) {
            const size_t N = ctrl.size();
            total_cost = 0.0;
            // --- distance cost (cubic penalty like EGO) ---
            // The inflated clearance only applies INSIDE the exploration
            // sub-region. In the A→B corridor (outside EXPLORE_*) the optimizer
            // keeps the same generic margin as cellWalkable (AB_EDGE_MARGIN +
            // 1 cell) so it can pass randomly-placed gates instead of being
            // pushed off by a too-large margin that the narrow slot cannot satisfy.
            const double BODY_DIST = AB_EDGE_MARGIN + GRID_RES;  // 0.25+0.1 = 0.35 m
            for (size_t i = 1; i + 1 < N; ++i) {
                double safe = inExploreRegion(ctrl[i].x, ctrl[i].y) ? SAFE_DIST : BODY_DIST;
                auto [d, dir] = nearestObs(ctrl[i].x, ctrl[i].y);
                if (d < safe && d > 1e-6) {
                    double err = safe - d;
                    total_cost += DIST_W * err * err * err;
                    double g = DIST_W * (-3.0 * err * err);
                    grad[i].x += g * dir.x;
                    grad[i].y += g * dir.y;
                }
            }
            // --- jerk smoothness cost ---
            for (size_t i = 2; i + 1 < N; ++i) {
                double jx = ctrl[i+1].x - 3.0*ctrl[i].x + 3.0*ctrl[i-1].x - ctrl[i-2].x;
                double jy = ctrl[i+1].y - 3.0*ctrl[i].y + 3.0*ctrl[i-1].y - ctrl[i-2].y;
                total_cost += SMOOTH_W * (jx*jx + jy*jy);
                double gx = SMOOTH_W * 2.0 * jx;
                double gy = SMOOTH_W * 2.0 * jy;
                grad[i-2].x += -gx;   grad[i-2].y += -gy;
                grad[i-1].x +=  3.0*gx; grad[i-1].y +=  3.0*gy;
                grad[i  ].x += -3.0*gx; grad[i  ].y += -3.0*gy;
                grad[i+1].x +=   gx;    grad[i+1].y +=   gy;
            }
        };

        // ---- initialise ----
        std::vector<Point> ctrl = path;
        double cost_k = 0.0;
        {
            std::vector<Point> g(ctrl.size());
            computeCostAndGrad(ctrl, g, cost_k);
        }
        double step = STEP_INIT;

        // ---- gradient descent with Armijo line search ----
        for (int iter = 0; iter < MAX_ITER; ++iter) {
            std::vector<Point> grad(ctrl.size());
            double cost;
            computeCostAndGrad(ctrl, grad, cost);

            // Armijo condition: find step that sufficiently reduces cost
            std::vector<Point> next(ctrl.size());
            double step_try = step;
            bool accepted = false;
            for (int ls = 0; ls < 12; ++ls) {
                for (size_t i = 1; i + 1 < ctrl.size(); ++i) {
                    next[i].x = ctrl[i].x - step_try * grad[i].x;
                    next[i].y = ctrl[i].y - step_try * grad[i].y;
                    // clamp to map bounds with drone body clearance
                    next[i].x = std::clamp(next[i].x, MAP_X_MIN + DRONE_RADIUS, MAP_X_MAX - DRONE_RADIUS);
                    next[i].y = std::clamp(next[i].y, MAP_Y_MIN + DRONE_RADIUS, MAP_Y_MAX - DRONE_RADIUS);
                }
                next[0] = ctrl[0];
                next.back() = ctrl.back();

                std::vector<Point> g_next(next.size());
                double cost_next;
                computeCostAndGrad(next, g_next, cost_next);

                double grad_norm2 = 0.0;
                for (size_t i = 0; i < grad.size(); ++i)
                    grad_norm2 += grad[i].x*grad[i].x + grad[i].y*grad[i].y;

                if (cost_next <= cost - 1e-4 * step_try * grad_norm2) {
                    accepted = true;
                    ctrl = next;
                    cost_k = cost_next;
                    step = std::min(step_try * 1.5, 0.2);
                    break;
                }
                step_try *= 0.5;
            }
            if (!accepted || cost_k < 0.001) break;
        }

        return ctrl;
    }

    // -----------------------------------------------------------
    //  Catmull-Rom spline interpolation
    // -----------------------------------------------------------
    std::vector<Point> interpolatePath(const GridMap& map,
                                       const std::vector<Point>& ctrl,
                                       bool allowUnknown = false) const
    {
        if (ctrl.size() < 2) return ctrl;

        std::vector<Point> dense;
        dense.reserve(1024);
        dense.push_back(ctrl.front());

        for (size_t i = 0; i + 1 < ctrl.size(); ++i) {
            Point p0 = (i > 0)               ? ctrl[i-1] : ctrl[i];
            Point p1 = ctrl[i];
            Point p2 = ctrl[i+1];
            Point p3 = (i + 2 < ctrl.size()) ? ctrl[i+2] : ctrl[i+1];

            double seg_len = std::sqrt((p2.x-p1.x)*(p2.x-p1.x) + (p2.y-p1.y)*(p2.y-p1.y));
            int steps = std::max(2, static_cast<int>(seg_len / SPLINE_RES));

            for (int s = 1; s <= steps; ++s) {
                double t  = static_cast<double>(s) / steps;
                double t2 = t * t;
                double t3 = t2 * t;

                double x = 0.5 * ( 2.0 * p1.x
                    + (-p0.x + p2.x) * t
                    + (2.0*p0.x - 5.0*p1.x + 4.0*p2.x - p3.x) * t2
                    + (-p0.x + 3.0*p1.x - 3.0*p2.x + p3.x) * t3);
                double y = 0.5 * ( 2.0 * p1.y
                    + (-p0.y + p2.y) * t
                    + (2.0*p0.y - 5.0*p1.y + 4.0*p2.y - p3.y) * t2
                    + (-p0.y + 3.0*p1.y - 3.0*p2.y + p3.y) * t3);

                if (cellWalkable(map, worldToGrid(x, y), allowUnknown)) {
                    dense.emplace_back(x, y);
                } else {
                    // Linear fallback — also check safety
                    double lx = p1.x + t * (p2.x - p1.x);
                    double ly = p1.y + t * (p2.y - p1.y);
                    if (cellWalkable(map, worldToGrid(lx, ly), allowUnknown))
                        dense.emplace_back(lx, ly);
                    // if both unsafe → skip this point (gap is < SPLINE_RES = 4cm)
                }
            }
        }

        if (dense.size() < 2 || dense.back().dist(ctrl.back()) > 0.01)
            dense.push_back(ctrl.back());

        return dense;
    }
};
