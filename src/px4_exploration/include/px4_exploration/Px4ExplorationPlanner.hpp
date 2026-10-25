#pragma once

#include "Types.hpp"
#include "GridMap.hpp"
#include "FrontierDetector.hpp"
#include "PathPlanner.hpp"
#include "BsplineTrajectory2D.hpp"
#include "BsplineOptimizer2D.hpp"
#include <vector>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <future>
#include <thread>
#include <chrono>
#include <chrono>
#include <utility>

/// Continuous-exploration FSM for PX4 drone.
/// Simple greedy: find nearest CS_FREE cell, fly there, extend when near end.
/// Re-plans while moving — no stop-and-go settle cycle needed when using
/// a stable global point cloud (FastLIO /Laser_map).
class Px4ExplorationPlanner {
public:
    /// ★近距直线安全检查(给节点用): 从 (x0,y0)→(x1,y1) 的直线段, 沿途离障碍是否都
    ///   ≥ DRONE_RADIUS + margin。安全 → 可以直接位置环直线飞; 不安全 → 必须走路径规划。
    bool straightLineSafe(double x0, double y0, double x1, double y1,
                          double margin = 0.05) const {
        const double need = DRONE_RADIUS + margin;
        const double L = std::hypot(x1 - x0, y1 - y0);
        const int n = std::max(2, (int)std::ceil(L / (GRID_RES * 0.5)));
        std::vector<Point> seg;
        seg.reserve(n + 1);
        for (int k = 0; k <= n; ++k) {
            const double t = (double)k / n;
            seg.push_back(Point(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t));
        }
        return pathMinClearance(seg) >= need;      // 与障碍的最小距离
    }

    /// 异步规划的一次结果: A* 路径 + EGO 平滑后的 B 样条一次算完,
    /// 控制线程只做一次移动赋值(µs 级) → 重规划不会卡住控制循环。
    struct BgPlan {
        std::vector<Point> path;
        UniformBspline2D   bspline;
        Point goal{0, 0};
        bool ok = false;
    };
    enum State { INIT, DETECT, PLAN, MOVE, SCAN_TURN, GO_HOME, FINISHED };

    Px4ExplorationPlanner() { reset(); }

    // --- modifiable parameters ---
    double home_pct_          = 98.0;
    double waypoint_tol_      = 0.10;
    double heading_penalty_   = 2.5;   // prefer forward direction when picking next viewpoint
    double extend_ratio_      = 0.3;   // extend when remaining < ratio × total_len (last 30% of path)
    double go_home_speed_ = 2.0;  // cruise speed when flying HOME (m/s)
    double corner_speed_  = 0.5;  // 探索完→探索区角点巡航(launch corner_speed)
    double replan_interval_   = 0.5;  // EGO-style replan period (s) while MOVE

    // ===== Return / landing configuration =====
    // flight_mode_ 0 → fly straight to pad A and land there.
    // flight_mode_ 1 → fly to pad A (corridor entry, at go_home_speed_), then
    //                   continue to pad B at channel_speed_ and land there.
    // flight_mode_ 2 → 与 1 同样进走廊, 但飞到"第一个窄口中点(过口出清)"后就
    //                   直接降落(不再继续后续口子/padB)。
    // flight_mode_ 3 → 不进走廊也不走探索区角点: 探索完/投放完直接 A* 回**起飞点**
    //                   (节点把 pad_a 设成起飞点), 到点原地降落。通道可能过不去,
    //                   这个模式最保险。
    int    flight_mode_    = 0;
    double pad_a_x_        = 8.5;   // landing corridor entry (left / north pad)
    double pad_a_y_        = 4.5;
    double pad_b_x_        = 8.5;   // true landing pad (right / south end, mode 1)
    double pad_b_y_        = -4.5;
    double channel_speed_  = 0.5;   // cruise speed of the corridor leg A→B (mode 1)
    // A→B 窄通道检测只在走廊横向窗口内进行(ego_world: 西=隔墙x7.5, 东=x9.1)。
    // 窗口外(探索大厅 x<7.5 的随机挡板、墙外)一律不参与 —— 否则会把大厅里
    // 挡板间的空隙误判成“窄口”, 中点跳进大厅/墙上。可被 launch 覆盖。
    double corridor_x0_ = 7.0;    // ★通道横向带下限(现场: 通道斜着从 x=7.6 到 9.45 ✓ 留余量)
    double corridor_x1_ = 10.0;   // ★通道横向带上限(现场 map_x_max=8.5 ✓ 留余量到 10 兼容)

    /// Configure the return-to-landing behaviour.
    void setReturnConfig(double ax, double ay, double bx, double by,
                         int mode, double channel_speed) {
        pad_a_x_ = ax; pad_a_y_ = ay;
        pad_b_x_ = bx; pad_b_y_ = by;
        flight_mode_   = (mode < 0) ? 0 : (mode > 3 ? 1 : mode);   // 支持 0/1/2/3
        channel_speed_ = (channel_speed > 0.1) ? channel_speed : go_home_speed_;
    }
    /// Convenience: pad A only (flight_mode 0). Backwards-compatible name kept.
    void setHomeGoal(double gx, double gy) {
        pad_a_x_ = gx; pad_a_y_ = gy;
        flight_mode_ = 0;
    }

    // --- public read-only state ---
    const GridMap&               map()        const { return map_; }
    const std::set<GridPos>&     frontiers()  const { return fd_.cells(); }
    const std::vector<Point>&    path()       const { return path_; }
    const std::vector<Point>&    viewpoints() const { return viewpoints_; }
    Point                        currentGoal()const { return goal_; }
    State                        state()      const { return state_; }
    double                       exploredPct()const { return map_.exploredPercent(); }
    bool                         finished()   const { return state_ == FINISHED; }
    int                          failCount()  const { return fail_count_; }
    Point                        padA()       const { return Point(pad_a_x_, pad_a_y_); }
    Point                        padB()       const { return Point(pad_b_x_, pad_b_y_); }
    int                          flightMode() const { return flight_mode_; }
    /// 返航是否已过"探索区角点"(之后进入降落走廊, 用 corridor_height 飞行)。
    bool                         cornerDone() const { return home_corner_done_; }
    /// ★指定探索完后的"左上角点"(odom系); 传 -999 = 自动(探索区边界内缩0.5m)
    void setHomeCorner(double x, double y) { home_corner_x_ = x; home_corner_y_ = y; }

    /// Sample the active B-spline (the trajectory the drone actually follows)
    /// into a polyline for GUI drawing. Returns empty when no spline is valid.
    std::vector<Point> sampledTrajectory(int n = 60) const {
        std::vector<Point> pts;
        double total_t = bspline_.duration();
        if (bspline_.numCtrl() < 4 || total_t < 0.01) return pts;
        pts.reserve(n);
        for (int i = 0; i <= n; ++i) {
            double t = total_t * i / n;
            pts.push_back(bspline_.pos(t));
        }
        return pts;
    }

    /// Full reset for a new run.
    void reset() {
        map_.clear();
        fd_ = FrontierDetector();
        path_.clear();
        viewpoints_.clear();
        goal_ = Point(0, 0);
        sense_timer_ = 0.0;
        stuck_timer_ = 0.0;
        fail_count_   = 0;
        consecutive_fail_ = 0;
        scan_turn_accum_ = 0.0;
        scan_settle_ = 0;
        scan_dir_ = 1.0;
        scan_no_target_ = 0;
        home_planned_ = false;
        home_yaw_align_ = false;
        yaw_align_phase_ = false;
        return_stage_ = 0;
        move_elapsed_ = 0.0;
        replan_timer_ = 0.0;
        recovery_used_ = false;
        last_yaw_ = 0.0;
        last_yawdot_ = 0.0;
        bspline_ = UniformBspline2D();
        state_ = INIT;
    }

    /// Mark downward camera FOV rectangle on ground as explored.
    // Px4ExplorationPlanner.hpp

void markDownwardFov(double drone_x, double drone_y, double drone_yaw,
                     double fx, double fy, double cx, double cy,
                     int cam_w, int cam_h, double height_m,
                     double tilt_angle_deg = 30.0)  // 与水平面夹角
{
    const double H = height_m > 0.3 ? height_m : 0.3;
    const double TILT_RAD = tilt_angle_deg * M_PI / 180.0;
    
    // ============================================================
    // 关键修改: 因为摄像头旋转90°，宽高互换
    // ============================================================
    // 原始: fov_width = cam_w * H / fx, fov_height = cam_h * H / fy
    // 旋转90°后: 宽变高，高变宽
    double fov_width  = (cam_h * H / fy);   // 原来高度变成宽度
    double fov_height = (cam_w * H / fx);   // 原来宽度变成高度
    
    // 由于倾斜，实际FOV是梯形，远端更宽
    // 无人机前向方向
    
    // 角度范围
    // 垂直方向: 从 -fov_height/2 到 +fov_height/2 (相对于中心线)
    // 水平方向: 从 -fov_width/2 到 +fov_width/2
    
    const double STEP = GRID_RES * 1.2;
    
    // 旋转矩阵（无人机偏航）
    double ch = std::cos(drone_yaw), sh = std::sin(drone_yaw);
    
    // 处理梯形FOV
    for (double vy = -fov_height/2.0; vy <= fov_height/2.0; vy += STEP) {
        // vy: 沿无人机前向方向（从近到远）
        // 倾斜导致透视效应: 远处的网格更大
        double dist_scale = 1.0 + vy / (H * 2.0);  // 远处放大
        
        // 有效高度 = 垂直投影高度
        // 摄像头向前倾斜，视野覆盖更远
        double effective_height = H + vy * std::tan(TILT_RAD);
        if (effective_height < 0.1) effective_height = 0.1;
        
        // 该行的水平FOV宽度（梯形，远处更宽）
        double row_width = fov_width * (effective_height / H);
        double half_row_width = row_width / 2.0;
        
        for (double vx = -half_row_width; vx <= half_row_width; vx += STEP * dist_scale) {
            // vx: 横向偏移（左/右）
            // vy: 纵向偏移（前/后），加上倾斜偏移
            
            // 倾斜导致视野中心前移
            double forward_offset = effective_height * std::tan(TILT_RAD);
            
            // 世界坐标 (无人机坐标系)
            // x: 前向 (vy + forward_offset)
            // y: 横向 (vx)
            double wx = drone_x + (vy + forward_offset) * ch - vx * sh;
            double wy = drone_y + (vy + forward_offset) * sh + vx * ch;
            
            // 检查是否在地图范围内
            if (wx < MAP_X_MIN || wx > MAP_X_MAX || wy < MAP_Y_MIN || wy > MAP_Y_MAX)
                continue;
            // 严格限制: 超出探索子区的 FOV 一律不算
            if (!inExploreRegion(wx, wy))
                continue;
            
            GridPos gp = worldToGrid(wx, wy);
            if (!gp.valid()) continue;
            
            // 不覆盖障碍物
            if (map_.get(gp) != CS_OCCUPIED) {
                map_.set(gp, CS_FOV_FREE);
            }
        }
    }
}

    /// Mark forward-facing camera FOV as the exact ground footprint of a
    /// landscape camera whose optical axis pitches DOWN fov_tilt_deg from
    /// horizontal. The footprint is an ANNULAR SECTOR on the ground:
    ///   - azimuth ±hfov/2 about drone_yaw, hfov = 2*atan(cam_w/(2*fx))
    ///     (or fov_h_deg when >0);
    ///   - radius R = H*cot(α), α = depression angle covered by the sensor:
    ///     α ∈ [tilt − vfov/2, tilt + vfov/2], vfov = 2*atan(cam_h/(2*fy));
    ///     R ∈ [H/tan(α_hi), H/tan(α_lo)], far edge clipped to fov_max_range.
    /// Cell (R,β) is seen by the camera ⟺ R·(cosβ,sinβ) is inside that sector.
    void markForwardFov(double drone_x, double drone_y, double drone_yaw,
                        double fx, double fy, double cx, double cy,
                        int cam_w, int cam_h, double height_m,
                        double fov_max_range = 4.0, double fov_h_deg = -1.0,
                        double fov_tilt_deg = 55.0)
    {
        (void)cx; (void)cy;
        const double H = height_m > 0.3 ? height_m : 0.3;      // camera height above ground
        const double hfov = (fov_h_deg > 0.0)
            ? fov_h_deg * M_PI / 180.0
            : 2.0 * std::atan(cam_w / (2.0 * fx));             // horizontal FOV (rad)
        const double vfov = 2.0 * std::atan(cam_h / (2.0 * fy)); // vertical FOV (rad)
        const double T = fov_tilt_deg * M_PI / 180.0;          // optical axis depression
        const double half = hfov / 2.0;

        // Depression angles covered by the sensor (both rays look forward-down).
        double a_hi = T + vfov / 2.0;  if (a_hi > M_PI / 2.0 - 1e-3) a_hi = M_PI / 2.0 - 1e-3;
        double a_lo = T - vfov / 2.0;  if (a_lo < 1e-3)               a_lo = 1e-3;
        if (a_lo >= a_hi) return;

        // Ground radii R = H*cot(α): near edge (steepest ray) → far edge (min α).
        double R_near = H / std::tan(a_hi);
        double R_far  = std::min(H / std::tan(a_lo), fov_max_range);
        if (R_far <= R_near) return;

        const double ch = std::cos(drone_yaw), sh = std::sin(drone_yaw);
        // Ray-cast coverage: for each azimuth β sweep a ray from R_near to
        // R_far.  Mark cells as explored ONLY up to the first obstacle that
        // blocks the ray — anything behind an obstacle stays unexplored, which
        // is the real FOV coverage (no seeing through walls/obstacles).
        // Angular step sized so consecutive rays are ≤ ~0.5 cell apart at the
        // far edge → no gaps between rays near the FOV edge.
        const int n_rays = std::max(24, (int)(hfov * R_far / (GRID_RES * 0.5)) + 1);
        for (int i = 0; i < n_rays; ++i) {
            const double beta = -half + hfov * i / (n_rays - 1);
            const double cbt = ch * std::cos(beta) - sh * std::sin(beta);
            const double sbt = sh * std::cos(beta) + ch * std::sin(beta);
            for (double R = R_near; R <= R_far; R += GRID_RES * 0.5) {
                const double wx = drone_x + cbt * R;
                const double wy = drone_y + sbt * R;
                if (wx < MAP_X_MIN || wx > MAP_X_MAX || wy < MAP_Y_MIN || wy > MAP_Y_MAX)
                    break;                      // left the map → stop this ray
                // 严格限制: 超出探索子区的 FOV 一律不算(墙/走廊点云可能未加载)
                if (!inExploreRegion(wx, wy))
                    break;
                GridPos gp = worldToGrid(wx, wy);
                if (!gp.valid()) break;
                if (map_.get(gp) == CS_OCCUPIED)
                    break;                      // blocked → don't see past it
                map_.set(gp, CS_FOV_FREE);
            }
        }
    }

    /// Process a SENSE cycle: detect frontiers from the current grid.
    void senseCycle() {
        fd_.update(map_, map_.getChanges());
        fd_.detectAll(map_);
        map_.clearChanges();
    }

    /// Run the exploration FSM. Returns position setpoint for PX4 position controller.
    struct Cmd { double pos_x=0, pos_y=0, pos_z=0, vel_x=0, vel_y=0, vel_z=0, yaw=0;
                 double acc_x=0, acc_y=0;   // ★加速度前馈(PX4 AFX/AFY) = 惯性补偿 ✓
                 bool valid=false; };
    Cmd update(double dt, double drone_x, double drone_y, double drone_z,
               double drone_yaw, double drone_vx, double drone_vy,
               double drone_vz, double drone_vyaw, double target_z)
    {
        Cmd cmd = {0.0, 0.0, 0.0, 0.0};
        if (!yaw_last_init_) { yaw_last_ = drone_.heading; yaw_last_init_ = true; }
        pollBgPlan();          // ★先取后台规划结果(有就应用, 没有立即返回)
        sense_timer_ += dt;

        // Store actual velocity for D damping in posTrack
        drone_.vx = drone_vx; drone_.vy = drone_vy;

        // Z height — set in position setpoint
        double set_z = target_z;

        // Trigger go-home: 探索覆盖率达标即返回(不再等当前 MOVE 轨迹飞完)。
        // GO_HOME 的 Phase1 会用当前位置保持 + vel=0 原地刹停, 再转去返回。
        if (state_ != INIT && state_ != FINISHED && state_ != GO_HOME
            && state_ != SCAN_TURN
            && map_.exploredPercent() >= home_pct_)
        {
            printf("[FSM] %.1f%% >= %.1f%% → GO_HOME\n", map_.exploredPercent(), home_pct_);
            beginGoHome();
        }

        switch (state_) {
        case INIT: {
            drone_.x = drone_x;
            drone_.y = drone_y;
            drone_.heading = drone_yaw;
            drone_.speed = 0.0;
            drone_.yaw_rate = 0.0;
            last_yaw_ = drone_yaw;
            last_yawdot_ = 0.0;
            senseCycle();
            fail_count_ = 0;
            consecutive_fail_ = 0;
            recovery_used_ = false;
            state_ = DETECT;
            break;
        }
        case DETECT: {
            Point dp(drone_.x, drone_.y);
            // Prefer targets in the forward-180° sector of the current heading,
            // never a near-180° reversal behind the drone
            Point target = findNearestUnvisited(dp, drone_yaw);
            if (target.x == 0 && target.y == 0) {
                // 没找到目标: 若已达标或探索区确无可去自由点 → 回家;
                // 否则(未达标)原地自转重扫再找 —— 不允许找身后 ~180° 的目标
                // 会让角上"找不到", 转几圈把候选扫进前方后 DETECT 再来。
                if (map_.exploredPercent() >= home_pct_ || !anyFreeTargetLeft()) {
                    printf("[DIAG] no target & done → GO_HOME @%.1f%%\n",
                           map_.exploredPercent());
                    beginGoHome(); break;
                }
                if (++scan_no_target_ < 16) {      // 转多圈仍无才兜底回家
                    scan_dir_        = -scan_dir_;  // 交替方向
                    scan_turn_accum_ = 0.0;
                    scan_settle_     = 0;
                    state_ = SCAN_TURN;             // 自转90°+sense 后回 DETECT
                    printf("[DIAG] no target @%.1f%% → rotate & rescan (try %d/16)\n",
                           map_.exploredPercent(), scan_no_target_);
                    break;
                }
                printf("[DIAG] no target after %d rotates → GO_HOME @%.1f%%\n",
                       scan_no_target_, map_.exploredPercent());
                beginGoHome(); break;
            }
            scan_no_target_ = 0;
            viewpoints_.clear();
            viewpoints_.push_back(target);
            state_ = PLAN;
            break;
        }
        case PLAN: {
            drone_.x = drone_x; drone_.y = drone_y; drone_.heading = drone_yaw;
            Point dp(drone_.x, drone_.y);
            double cos_h = std::cos(drone_.heading), sin_h = std::sin(drone_.heading);

            // Quick local density helper: count CS_FREE cells in ~1.2m radius (sampled every 3 cells)
            auto localDensity = [&](int gx, int gy) -> int {
                int cnt = 0;
                const int R = 12;  // 1.2m in grid cells (0.1m each)
                for (int dy = -R; dy <= R; dy += 3)
                    for (int dx = -R; dx <= R; dx += 3)
                        if (map_.get(gx + dx, gy + dy) == CS_FREE) ++cnt;
                return cnt;
            };

            // Gather ALL CS_FREE cells with density, split forward-180° / rear
            struct Cand { Point p; int density; };
            std::vector<Cand> fwd, rear;
            for (int gy = 0; gy < GRID_N; ++gy)
                for (int gx = 0; gx < GRID_N; ++gx) {
                    if (map_.get(gx, gy) != CS_FREE) continue;
                    Point wp = gridToWorld(gx, gy);
                    if (!inExplore(wp)) continue;
                    if (dp.distSq(wp) < 0.25) continue;
                    if (nearObstacle(gx, gy)) continue;  // skip cells hugging obstacles

                    double to_x = wp.x - dp.x, to_y = wp.y - dp.y;
                    double dist = std::sqrt(to_x*to_x + to_y*to_y);
                    double dot = (to_x/dist)*cos_h + (to_y/dist)*sin_h;

                    int dens = localDensity(gx, gy);
                    if (dot >= 0.0) fwd.push_back({wp, dens});
                    else            rear.push_back({wp, dens});
                }

            // Strongly prefer forward cells; only use rear if forward is empty
            auto& pool = fwd.empty() ? rear : fwd;
            if (pool.empty()) {
                // ★诊断(不改行为): 为什么没候选? 现场看这一行就够 —— CS_FREE 格数量、
                //   前/后向拆分、探索率、视觉暂缓返航(hold_home)是否把返航挡住了。
                if (++diag_nocand_ % 50 == 1)
                    printf("[DIAG] 无候选: 探索=%.1f%% hold_home=%d → %s\n",
                           exploredPct(), (int)hold_home_,
                           hold_home_ ? "暂缓返航(等视觉做完) → 保持 PLAN 空转" : "beginGoHome()");
                beginGoHome(); break;
            }

            // Sort by scoring function:
            //   score = distance × heading_penalty / density
            // Lower score = better (closer, more forward, bigger unexplored region)
            // density+1 avoids div-by-zero and gives isolated cells a penalty
            double dens_weight = 1.0;  // how strongly to prefer dense regions
            // ★★ 探索策略: x >= far_first_x_ 的候选【一律优先】(起飞点附近 x 小的放最后探索)。
            //    两层排序: 先比"是否在优先区", 同层内再比原来的 score。
            const double ffx = far_first_x_;
            std::sort(pool.begin(), pool.end(),
                [&](const Cand& a, const Cand& b) {
                    if (a.p.x >= ffx && b.p.x < ffx) return true;    // a 在优先区
                    if (a.p.x <  ffx && b.p.x >= ffx) return false;  // b 在优先区
                    double da = dp.distSq(a.p), db = dp.distSq(b.p);
                    double aa = std::atan2(a.p.y-dp.y, a.p.x-dp.x);
                    double ab = std::atan2(b.p.y-dp.y, b.p.x-dp.x);
                    double dh_a = std::abs(aa - drone_.heading);
                    double dh_b = std::abs(ab - drone_.heading);
                    if (dh_a > PI) dh_a = 2*PI - dh_a;
                    if (dh_b > PI) dh_b = 2*PI - dh_b;
                    // ★★ 太近的目标(默认<2.0m)施加 3 倍惩罚:
                    //    原来 score ∝ sqrt(距离) → 永远优先选最近的点 ✗
                    //    → 刚起飞时地图小, 第一段只有 ~1m → B样条太短只能慢加速 → (现场 FOV 最远 2.6m, 故阈值取 1.5m ✓)
                    //      "慢悠悠 + 冲过终点再回来" ✗ (后面地图大了就正常 ✓)
                    const double MIN_T = 1.5;              // 最小目标距离(m) — 现场 FOV 最远只 ~2.6m, 2.0 太严格可能无点 ✓
                    double ds_a = std::sqrt(da); if (std::sqrt(da) < MIN_T) ds_a *= 3.0;
                    double ds_b = std::sqrt(db); if (std::sqrt(db) < MIN_T) ds_b *= 3.0;
                    double score_a = ds_a * (1.0 + heading_penalty_*(1.0-std::cos(dh_a)))
                                   / (1.0 + dens_weight * a.density);
                    double score_b = ds_b * (1.0 + heading_penalty_*(1.0-std::cos(dh_b)))
                                   / (1.0 + dens_weight * b.density);
                    return score_a < score_b;
                });

            // ★诊断日志(不改行为): 每 ~1s 打印选点情况 —— 现场判断"为什么在原地磨蹭":
            //   池大小/前后向拆分/前 3 候选的距离·密度·分数。分数公式与下面排序完全一致。
            if (++diag_pick_ % 50 == 1) {
                auto score_of = [&](const Cand& c) {
                    double tx = c.p.x - dp.x, ty = c.p.y - dp.y;
                    double dist = std::sqrt(tx*tx + ty*ty);
                    if (dist < 1e-6) return 0.0;
                    double dh = std::abs(std::atan2(ty, tx) - drone_.heading);
                    if (dh > PI) dh = 2*PI - dh;
                    double ds = dist; if (dist < 1.5) ds *= 3.0;   // 与上面排序一致(近点惩罚) ✓
                    return ds * (1.0 + heading_penalty_ * (1.0 - std::cos(dh)))
                                 / (1.0 + c.density);
                };
                std::string top;
                for (size_t i = 0; i < pool.size() && i < 3; ++i) {
                    char tmp[160];
                    snprintf(tmp, sizeof(tmp), "%s#%zu(%.2f,%.2f) d=%.2f dens=%d s=%.3f",
                             i ? " | " : "", i + 1, pool[i].p.x, pool[i].p.y,
                             std::hypot(pool[i].p.x - dp.x, pool[i].p.y - dp.y),
                             pool[i].density, score_of(pool[i]));
                    top += tmp;
                }
                printf("[DIAG] 选点: 池=%zu(前=%zu 后=%zu%s) 机=(%.2f,%.2f) 航向=%.0f° 探索=%.1f%% | %s\n",
                       pool.size(), fwd.size(), rear.size(), fwd.empty() ? " ※前向空,用后向" : "",
                       dp.x, dp.y, drone_.heading * 180.0 / PI, exploredPct(), top.c_str());
            }

            // ★候选点规划放后台: 就绪先用结果, 没就绪提交任务并原地悬停等
            bool any_path = false;
            if (bg_gh_done_) {
                any_path = bg_gh_ok_;
                if (any_path) {
                    goal_ = bg_gh_goal_;
                    fail_count_ = 0; consecutive_fail_ = 0;
                    path_ = std::move(bg_gh_path_);
                    if (++diag_pick_ok_ % 10 == 1) {
                        double L = 0.0;
                        for (size_t i = 1; i < path_.size(); ++i) L += path_[i].dist(path_[i-1]);
                        const double straight = std::hypot(goal_.x - drone_.x, goal_.y - drone_.y);
                        const double ratio = (straight > 0.2) ? L / straight : 0.0;
                        // ★"绕远"警告: 目标就在附近却规划出很长的路径 → 飞机要绕一大圈,
                        //   现场表现就是"在原地附近磨蹭/到处转"。多半是起/终点格子被判成
                        //   不可通行(或地图出现假障碍环)导致 A* 绕行。
                        const bool detour = (straight > 0.3 && L > 3.0 && ratio > 3.0);
                        printf("[DIAG] 选点成功: 目标=(%.2f,%.2f) 机=(%.2f,%.2f) 直线=%.2fm 路径=%.2fm(%zu点)%s\n",
                               goal_.x, goal_.y, drone_.x, drone_.y, straight, L, path_.size(),
                               detour ? " ★绕远!" : "");
                    }
                }
                bg_gh_done_ = false;
            } else if (bg_kind_ == 6 && bg_fut_.valid()) {
                // 在算: 原地悬停(与 DETECT/PLAN 原本的行为一致), 不阻塞控制循环
                cmd.pos_x = drone_x; cmd.pos_y = drone_y;
                cmd.vel_x = 0; cmd.vel_y = 0; cmd.yaw = yaw_last_; cmd.valid = true;
                return cmd;
            } else {
                std::vector<Point> pl; pl.reserve(pool.size());
                for (const auto& c : pool) pl.push_back(c.p);
                startTargetPlanJob(std::move(pl), dp);
                cmd.pos_x = drone_x; cmd.pos_y = drone_y;
                cmd.vel_x = 0; cmd.vel_y = 0; cmd.yaw = yaw_last_; cmd.valid = true;
                return cmd;
            }

            if (!any_path) {
                ++fail_count_; ++consecutive_fail_;
                if (diag_pick_fail_++ % 50 == 1)
                    printf("[DIAG] 选点失败(第%d次, 池=%zu 首选=(%.2f,%.2f) d=%.2fm) → %s\n",
                           fail_count_, pool.size(), pool[0].p.x, pool[0].p.y,
                           std::hypot(pool[0].p.x - dp.x, pool[0].p.y - dp.y),
                           fail_count_ >= 20 ? "beginGoHome/自转" : "重新 DETECT");
                if (fail_count_ >= 20) {
                    beginGoHome();
                    // ★beginGoHome() 会被"视觉标靶没做完 → 暂缓返航"(hold_home_)挡住而
                    //   什么都不做 —— 那样状态机会永远停在 PLAN 里空转(每拍跑一次失败的
                    //   A*, 飞机原地悬停)。这里补上: 没真的转成 GO_HOME 就去自转一圈
                    //   重新感知(和"转多圈无目标"的兜底一致), 顺便把失败计数清零。
                    if (state_ != GO_HOME) {
                        fail_count_ = 0;
                        scan_dir_        = -scan_dir_;   // 交替方向
                        scan_turn_accum_ = 0.0;
                        scan_settle_     = 0;
                        state_ = SCAN_TURN;
                    }
                } else { state_ = DETECT; }
            } else {
                state_ = MOVE;
                move_elapsed_ = 0.0;
                // Yaw-align if drone heading is >90° off from the new path direction.
                // Prevents flying sideways/backwards into obstacles from standstill.
                double path_yaw = std::atan2(path_[1].y - path_[0].y, path_[1].x - path_[0].x);
                double yaw_err = path_yaw - drone_yaw;
                while (yaw_err >  PI) yaw_err -= 2.0 * PI;
                while (yaw_err < -PI) yaw_err += 2.0 * PI;
                yaw_align_phase_ = (std::abs(yaw_err) > 1.57);  // > 90° misalignment
                if (!yaw_align_phase_) rebuildBspline(DRONE_SPEED);
            }
            break;
        }
        case SCAN_TURN: {
            constexpr double SCAN_ANGLE = 90.0 * DEG2RAD;
            drone_.speed = 0.0;
            if (scan_settle_ > 0) {
                cmd.pos_x = drone_x; cmd.pos_y = drone_y; cmd.valid = true;
                scan_settle_--;
                if (scan_settle_ == 0) {
                    last_yaw_ = drone_yaw;
                    last_yawdot_ = 0.0;
                    senseCycle();
                    state_ = DETECT;
                }
            } else if (scan_turn_accum_ < SCAN_ANGLE) {
                // ★自转扫描也以"上一次指令yaw"为基准累加: 用测量yaw做基准的话,
                //   SLAM 的 yaw 漂移会被累加进去 → 越转越多/来回摆。
                cmd.yaw = (yaw_last_init_ ? yaw_last_ : drone_yaw)
                        + scan_dir_ * MAX_YAW_RATE * 0.6 * dt;
                yaw_last_ = cmd.yaw;
                cmd.pos_x = drone_x; cmd.pos_y = drone_y; cmd.valid = true;
                scan_turn_accum_ += MAX_YAW_RATE * 0.6 * dt;
            } else {
                cmd.pos_x = drone_x; cmd.pos_y = drone_y; cmd.valid = true;
                scan_turn_accum_ = 0.0;
                scan_settle_ = 20;
            }
            break;
        }
        // ============================================================
        //  MOVE — fly path, extend with nearest CS_FREE when near end.
        // ============================================================
        case MOVE: {
            if (path_.size() < 2) { state_ = DETECT; break; }

            drone_.x = drone_x; drone_.y = drone_y; drone_.heading = drone_yaw;

            // --- yaw alignment phase: rotate to face path before moving ---
            if (yaw_align_phase_) {
                Point yaw_pt = findLookAheadFrom(path_, 0, 1.4);
                cmd.yaw = std::atan2(yaw_pt.y - drone_.y, yaw_pt.x - drone_.x);
                yaw_last_ = cmd.yaw;
                cmd.pos_x = drone_x;
                cmd.pos_y = drone_y;
                cmd.vel_x = 0; cmd.vel_y = 0;
                cmd.valid = true;

                double heading_error = cmd.yaw - drone_yaw;
                while (heading_error >  PI) heading_error -= 2.0 * PI;
                while (heading_error < -PI) heading_error += 2.0 * PI;
                if (std::abs(heading_error) < 0.26) {   // ~15° — aligned enough
                    yaw_align_phase_ = false;
                    stuck_timer_ = 1e-6;
                    move_start_x_ = drone_x;
                    move_start_y_ = drone_y;
                    move_elapsed_ = 0.0;
                    rebuildBspline(DRONE_SPEED);       // fresh B-spline after alignment
                }
                break;
            }

            if (stuck_timer_ <= 0.0) {
                move_start_x_ = drone_x;
                move_start_y_ = drone_y;
                stuck_timer_ = 1e-6;
            }

            // 先算"是否已到路径末端"(下面悬停分支还用它)
            double dist_end_pre = std::sqrt((drone_.x-path_.back().x)*(drone_.x-path_.back().x)
                                          + (drone_.y-path_.back().y)*(drone_.y-path_.back().y));
            // ★轨迹时钟只在真正推进航段时前进: 到末端"等延长"的悬停期间不能走 ——
            //   否则悬停 2s 后 move_elapsed_ 多跑 2s, 换上新样条后设定点会一下跳到
            //   机位前方约 1m(cruise×2s) → PX4 猛追 → "原地悬停后突然非常快地飞"。
            bool ext_holding = false;
            if (dist_end_pre < waypoint_tol_ && bg_fut_.valid() && bg_kind_ == 3) ext_holding = true;
            if (!ext_holding) move_elapsed_ += dt;   // advance trajectory clock (time-based posTrack)
            cmd = posTrack(DRONE_SPEED);

            // Safety: the obstacle map refreshes every frame — if a newly
            // marked obstacle now blocks the remaining path, re-plan from
            // DETECT instead of flying into it.
            obstacle_check_timer_ += dt;
            if (obstacle_check_timer_ > 0.4) {
                obstacle_check_timer_ = 0.0;
                if (newObstacleOnPath()) {
                    printf("[SAFE] obstacle on remaining path → re-plan\n");
                    state_ = DETECT;
                    break;
                }
            }

            // EGO-style periodic LOCAL re-plan: every replan_interval_ seconds,
            // A* again from the CURRENT drone position to a ~2m look-ahead on
            // the existing path and re-attach the untouched tail.  The head is
            // therefore continuously re-optimised / obstacle-aware in real time
            // while the sweeping intent of the extended path is preserved —
            // the flown trajectory is no longer frozen at MOVE start.
            replan_timer_ += dt;
            if (replan_timer_ >= replan_interval_) {
                replan_timer_ = 0.0;
                // ★重规划扔给后台线程(A* 不再卡住控制循环): 本拍继续沿当前路径飞,
                //   后台算好后由 pollBgPlan() 在下一拍无缝换上新路径。
                startMoveReplan();
            }

            // Remaining path length + total path length
            double remaining = 0.0, total_path_len = 0.0;
            {
                size_t ci = 0; double bd2 = 1e9;
                for (size_t i = 0; i < path_.size(); ++i) {
                    double d2 = (path_[i].x-drone_.x)*(path_[i].x-drone_.x)
                              + (path_[i].y-drone_.y)*(path_[i].y-drone_.y);
                    if (d2 < bd2) { bd2 = d2; ci = i; }
                }
                for (size_t i = ci; i + 1 < path_.size(); ++i)
                    remaining += path_[i].dist(path_[i+1]);
                for (size_t i = 1; i < path_.size(); ++i)
                    total_path_len += path_[i].dist(path_[i-1]);
            }

            // Near end → extend to nearest CS_FREE (with loop-back guards).
            // Once the explored% threshold is reached we STOP extending and
            // just fly the remaining path to its end before returning home.
            double extend_dist = total_path_len * extend_ratio_;
            if (remaining < extend_dist && map_.exploredPercent() < home_pct_) {
                // ★"延长轨迹"交给后台(原来在这里同步试最多10个候选 A* → 最坏卡200~300ms)。
                //   两个关键约束(实测踩过):
                //   ① 节流到 0.5s 一次 —— 以前每拍都提交, 失败时会把唯一的后台槽占满,
                //      视觉目标的任务提交不进去 → "视觉有数据一直不飞视觉";
                //   ② 视觉正在等规划时(m_bg_vis_want_)一律让路。
                ext_retry_t_ += dt;
                if (!bg_vis_want_ && ext_retry_t_ >= 0.5) {
                    ext_retry_t_ = 0.0;
                    startExtendJob();
                }
            } else {
                ext_retry_t_ = 0.0;
            }

            // End detection (只按位置判定, 不看速度): 到路径终点 < waypoint_tol_(默认0.10m)
            double dist_end = std::sqrt((drone_.x-path_.back().x)*(drone_.x-path_.back().x)
                                      + (drone_.y-path_.back().y)*(drone_.y-path_.back().y));
            if (dist_end < waypoint_tol_) {
                // ★到路径末端了, 但后台"延长"还没算出来 → 原地位置环悬停等(最多2s),
                //   而不是急着转 DETECT(这正是"飞完当前轨迹到末端悬停等下一段"的行为)。
                if (bg_fut_.valid() && bg_kind_ == 3) {
                    ext_wait_ += dt;
                    if (ext_wait_ < 2.0) {
                        cmd.pos_x = drone_.x; cmd.pos_y = drone_.y; cmd.pos_z = 0.0;
                        cmd.vel_x = 0.0; cmd.vel_y = 0.0; cmd.vel_z = 0.0;
                        cmd.yaw = yaw_last_; cmd.valid = true;   // ★重复上次指令yaw
                        break;
                    }
                    printf("[EXT] 等后台延长超过 %.0fs → 转 DETECT\n", ext_wait_);
                }
                ext_wait_ = 0.0;
                if (map_.exploredPercent() >= home_pct_) {
                    printf("[FSM] path end reached (%.1f%%) → GO_HOME\n",
                           map_.exploredPercent());
                    beginGoHome(); break;
                }
                state_ = DETECT; break;
            }
            ext_wait_ = 0.0;

            // Stuck detection
            double moved = std::sqrt(
                (drone_.x - move_start_x_)*(drone_.x - move_start_x_) +
                (drone_.y - move_start_y_)*(drone_.y - move_start_y_));
            if (moved > 0.15) { move_start_x_ = drone_.x; move_start_y_ = drone_.y; stuck_timer_ = 0.0; }
            stuck_timer_ += dt;
            if (stuck_timer_ > 5.0) {
                stuck_timer_ = 0.0;
                state_ = DETECT;
                break;
            }

            break;
        }
        case GO_HOME: {
            drone_.x = drone_x; drone_.y = drone_y; drone_.heading = drone_yaw;
            last_yaw_ = drone_yaw;

            // --- plan the current return leg when none is active ---
            if (!home_planned_) {
                // return_stage_ 0 → pad A (landing corridor entry / the landing
                // pad in mode 0).  return_stage_ 1 → pad B (flight_mode 1 keeps
                // going down the corridor at channel_speed to the true pad).
                if (return_stage_ == 1 && !st1_aligned_) {
                    // 到达 padA: 先原地 yaw 对准 -y —— 前180°点云模式下, 机头
                    // 没朝走廊时点云根本没覆盖窄墙, 不能此时检测。对准完成后
                    // (Phase1 对准确认处) 才检测窄墙中点并规划。
                    home_yaw_align_ = true;
                    st1_await_align_ = true;
                    home_planned_    = true;     // 本 tick 不规划, 交给 Phase1 对准
                    path_.clear();
                } else {
                const bool direct_home = (flight_mode_ == 3);   // mode3: 直接回起飞点, 不走角点
                // ★"探索区左上角"这步: 可用 launch 的 home_corner_x/y 直接指定
                //   (默认 -999 = 自动, 沿用"探索区边界内缩0.5m")
                const bool corner_auto = !std::isfinite(home_corner_x_) ||
                                         !std::isfinite(home_corner_y_) ||
                                         (home_corner_x_ < -900.0) || (home_corner_y_ < -900.0);
                const double corner_x = corner_auto ? (EXPLORE_X_MAX - 0.5) : home_corner_x_;
                const double corner_y = corner_auto ? (EXPLORE_Y_MAX - 0.5) : home_corner_y_;
                double tx = (return_stage_ == 0)
                            ? ((!home_corner_done_ && !direct_home)
                               ? corner_x : pad_a_x_)
                            : pad_b_x_;
                double ty = (return_stage_ == 0)
                            ? ((!home_corner_done_ && !direct_home)
                               ? corner_y : pad_a_y_)
                            : pad_b_y_;
                if (return_stage_ == 1) {
                    // A→B: 和探索区同一套路径规划(PathPlanner: A* + LOS剪枝 +
                    // 离障推进 + 5cm加密; 不做 EGO/样条平滑 —— 平滑在窄走廊里会把
                    // 路径拉到贴墙)。不找口中点去拼折线, 由 A* 按地图几何自己穿
                    // 0/1/N 个窄口。两个模式只差"目标点":
                    //   mode 1 → padB 本身(到点降落)
                    //   mode 2 → 第一个窄口中点再沿 -y 0.5m(到点即降落)
                    // 非探索区(走廊)不加膨胀光晕, 只用 ab_edge_margin —— 见
                    // PathPlanner::cellWalkable 与 planCorridor()。
                    const double CLR_M2 = 0.5;      // mode2: 第一口之后再沿-y 0.5m
                    double tgx = pad_b_x_, tgy = pad_b_y_;
                    st1_goal_is_gate_ = false;
                    if (flight_mode_ == 2) {
                        auto gates = gateCentersBetween(drone_.y, pad_b_y_);
                        for (auto& g : gates) {
                            if (g.y < drone_.y - 0.2) {   // 前方第一个口
                                tgx = g.x;
                                tgy = g.y - CLR_M2;
                                st1_goal_is_gate_ = true;
                                break;
                            }
                        }
                    }
                    st1_leg_goal_ = Point(tgx, tgy);
                    // A→B 用"保守规划"(A* + LOS剪枝 + 直线加密, 不做EGO/样条平滑):
                    // 平滑会把路径从 A* 保证可走的位置拉到真实墙面附近(实测只剩
                    // 1cm, 会撞), 保守规划的真实间隙 ≈ 边缘距离 - 5cm(栅格量化)。
                    if (!bg_gh_done_) {           // ★后台还没算好 → 提交并本拍原地悬停
                        startGoHomePlanJob(5, Point(drone_.x, drone_.y), Point(tgx, tgy));
                        cmd.vel_x = 0; cmd.vel_y = 0; cmd.yaw = yaw_last_; cmd.valid = true;
                        break;
                    }
                    path_ = std::move(bg_gh_path_); bg_gh_done_ = false;
                    printf("[GH] stage1 plan: cur=(%.2f,%.2f) → goal=(%.2f,%.2f)%s path=%zu\n",
                           drone_.x, drone_.y, tgx, tgy,
                           st1_goal_is_gate_ ? " (mode2 第一口-0.5m)" : " (padB)",
                           path_.size());
                } else {
                    if (!bg_gh_done_) {          // ★同上: 后台算, 本拍悬停等
                        startGoHomePlanJob(4, Point(drone_.x, drone_.y), Point(tx, ty));      // ★回原值(stride默认1) —— 建图抽样已解决卡顿, 不需要粗搜索 ✓
                        cmd.pos_x = drone_x; cmd.pos_y = drone_y;
                        cmd.vel_x = 0; cmd.vel_y = 0; cmd.yaw = yaw_last_; cmd.valid = true;
                        break;
                    }
                    path_ = std::move(bg_gh_path_); bg_gh_done_ = false;
                }
                if (path_.empty()) {
                    // 随机挡板布局可能让某次规划暂时找不到可行路径(地图在实时更新):
                    // 悬停原地, 每 tick 重试规划; 连续失败过多才放弃降落。
                    if (++gh_plan_fail_ < 20) {
                        printf("[GH] leg plan empty (stage %d) retry %d/20 — hover...\n",
                               (int)return_stage_, gh_plan_fail_);
                        cmd.pos_x = drone_x; cmd.pos_y = drone_y;
                        cmd.vel_x = 0; cmd.vel_y = 0;
                        cmd.valid = true;
                    } else {
                        printf("[GH] plan failed 20x → FINISHED (hover/land)\n");
                        state_ = FINISHED;
                    }
                    break;
                }
                gh_plan_fail_ = 0;
                goal_ = (return_stage_ == 1)
                        ? st1_leg_goal_             // A→B 本段目标=口中点 或 padB
                        : Point(tx, ty);
                home_planned_ = true;
                replan_timer_ = 0.0;   // fresh leg: restart the re-check timer
                if (return_stage_ == 0) {
                    // 去探索角: 用 corner_speed_; 角→pad A 用 go_home_speed_。
                    // (mode 0 / mode 1 都一样: 先到角点再到 pad A)
                    return_cruise_ = (!home_corner_done_)
                                     ? corner_speed_ : go_home_speed_;
                    leg_speed_     = return_cruise_;
                } else {
                    // A→B: 直接直线过口中点; 开阔段 channel_speed_(0.4), 窄口实时降速。
                    double minc = pathMinGateClearance(path_);   // 仅诊断用
                    return_cruise_ = channel_speed_;
                    leg_speed_     = channel_speed_;
                    slow_active_   = false;
                    st1_last_y_    = drone_.y;   // 推进式重扫的基线
                    printf("[GH] leg%d plan: gate-clear %.2f m → open-cruise %.2f (variable)\n",
                           (int)return_stage_, minc, return_cruise_);
                }
                if (return_stage_ == 1) {
                    // A→B: 机头固定朝 -y。仅"尚未对准"时需要原地对准;
                    // 已经对准过(st1_aligned_)=直接进入 Phase2 沿路径飞,
                    // 否则会 规划→清路径→再对准 死循环(轨迹一闪就没)。
                    home_yaw_align_ = !st1_aligned_;
                    if (!home_yaw_align_) move_elapsed_ = 0.0;
                } else {
                    // 去 pad A 段: 只有机头与返航方向误差 > 50° 才原地对准;
                    // 否则直接沿路径飞行(posTrack 切线自行平滑转向)。
                    // ★过了探索区角点后, 去 padA 这段【固定朝 -y】(走廊方向):
                    //   在角点(开阔处)就把机头转好, 避免进通道后在气流里转身(不稳/易撞)。
                    double tyaw;
                    if (home_corner_done_) {
                        tyaw = -M_PI / 2.0;
                    } else {
                        Point la = findLookAheadFrom(path_, 0, 1.4);
                        tyaw = std::atan2(la.y - drone_.y, la.x - drone_.x);
                    }
                    double he = tyaw - drone_yaw;
                    while (he >  PI) he -= 2.0 * PI;
                    while (he < -PI) he += 2.0 * PI;
                    home_yaw_align_ = (std::abs(he) > (home_corner_done_ ? 10.0 * DEG2RAD : 50.0 * DEG2RAD));
                    if (!home_yaw_align_) {
                        move_elapsed_ = 0.0;
                        rebuildBspline(return_cruise_);   // 直接进入Phase2
                    }
                }
                }   // end else (非“等对准”的常规规划)
            }

            // --- Phase 1: yaw align before flying home (prevents high-speed sideways collision) ---
            if (home_yaw_align_) {
                // Phase1 对准目标: A→B 段固定 -y(机头朝通道), 去 A 段按路径
                // 前方 ~1.4m look-ahead 点(不用第一小步, 防转一圈)。
                double target_yaw;
                if (return_stage_ == 1) {
                    target_yaw = -M_PI / 2.0;              // 固定朝 -y
                } else if (home_corner_done_) {
                    target_yaw = -M_PI / 2.0;          // ★过角点后固定朝 -y
                } else {
                    Point yaw_pt = findLookAheadFrom(path_, 0, 1.4);
                    target_yaw = std::atan2(yaw_pt.y - drone_.y, yaw_pt.x - drone_.x);
                }
                cmd.yaw   = target_yaw;
                cmd.pos_x = drone_x;
                cmd.pos_y = drone_y;
                cmd.vel_x = 0; cmd.vel_y = 0;
                cmd.valid = true;

                double heading_error = target_yaw - drone_yaw;
                while (heading_error >  PI) heading_error -= 2.0 * PI;
                while (heading_error < -PI) heading_error += 2.0 * PI;
                if (std::abs(heading_error) < 0.18) {   // ~10° — 对准更利落再飞
                    home_yaw_align_ = false;
                    move_elapsed_ = 0.0;
                    if (return_stage_ == 1) {
                        // A→B: 此时机头已朝 -y, 前180°点云才覆盖走廊 →
                        // 现在才检测窄墙中点并规划(A→口 轴对齐路径)。
                        // 置 st1_aligned_ 允许后续用点云检测, 下一 tick 规划。
                        st1_aligned_     = true;
                        st1_await_align_ = false;
                        home_planned_    = false;   // 触发下一 tick 检测+规划
                        path_.clear();
                        bspline_ = UniformBspline2D();
                        printf("[GH] aligned -y → detect gates from cloud & plan A→B\n");
                        cmd = Cmd{};    // hover this tick
                        break;
                    } else {
                        rebuildBspline(return_cruise_);      // now build B-spline and start
                    }
                    printf("[GH] aligned err=%.2f → Phase2 fly home (stage %d)\n",
                           heading_error, (int)return_stage_);
                }
                break;
            }

            // Periodic obstacle re-check while flying the return leg.
            // A→B: 口子在 padA 规划时已从整张全局地图一次找好并写进直线折线,
            // 飞行中不做“每 0.35m 往前探新口子”的增量判断 —— 只在真正遇到
            // 障碍挡路时才重规划, 避免轨迹在飞行中不断被改成“靠近才拐”的线。
            replan_timer_ += dt;
            double replan_period = (return_stage_ == 1)
                                   ? std::min(replan_interval_, 0.5)
                                   : replan_interval_;
            if (replan_timer_ >= replan_period && path_.size() >= 2) {
                replan_timer_ = 0.0;
                bool need = newObstacleOnPath()
                         || (return_stage_ == 1 && pathGrazeAhead(2.5));
                if (need) {
                    // A→B 重规划: 和初规划一样, 用 A* 重新规划到本段目标
                    // (mode1=padB / mode2=第一个口再-y0.5m)。不再拼折线。
                    std::vector<Point> np;
                    // ★关键: 重规划只是"优化/避新障", 当前路径仍然有效 —— 在算或算失败都
                    //   继续沿当前路径飞, 绝不原地悬停(以前在算时悬停, 算失败还会一直悬停
                    //   到放弃降落)。
                    const bool have_result = bg_gh_done_;
                    if (return_stage_ == 1) {
                        const double CLR_M2 = 0.5;
                        double tgx = pad_b_x_, tgy = pad_b_y_;
                        st1_goal_is_gate_ = false;
                        if (flight_mode_ == 2) {
                            auto gates = gateCentersBetween(drone_.y, pad_b_y_);
                            for (auto& g : gates) {
                                if (g.y < drone_.y - 0.2) {
                                    tgx = g.x;
                                    tgy = g.y - CLR_M2;
                                    st1_goal_is_gate_ = true;
                                    break;
                                }
                            }
                        }
                        st1_leg_goal_ = Point(tgx, tgy);
                        if (!bg_gh_done_) {      // ★后台重规划: 本拍原地悬停等(不沿被挡路径飞)
                            startGoHomePlanJob(5, Point(drone_.x, drone_.y), Point(tgx, tgy));
                            cmd.vel_x = 0; cmd.vel_y = 0; cmd.yaw = yaw_last_; cmd.valid = true;
                            break;
                        }
                        np = std::move(bg_gh_path_); bg_gh_done_ = false;
                    } else {
                        if (!have_result) {       // ★同上: 继续飞当前路径
                            startGoHomePlanJob(4, Point(drone_.x, drone_.y), path_.back());      // ★回原值 ✓
                        } else {
                            np = std::move(bg_gh_path_); bg_gh_done_ = false;
                        }
                    }
                    // 新路径起点必须在机附近(<0.6m), 否则视为无效(防止横向跳变撞墙)
                    bool adopt = have_result && (np.size() >= 2)
                              && (np.front().dist(Point(drone_.x, drone_.y)) < 0.6);
                    if (adopt) {
                        path_ = std::move(np);
                        if (return_stage_ == 1) goal_ = st1_leg_goal_;
                        st1_last_y_ = drone_.y;      // 每次重建后重置推进基线
                        move_elapsed_ = 0.0;
                        if (return_stage_ == 1) {
                            // A→B 重规划: 纯折线飞行, 不建 B 样条(样条=曲线)。
                            // 保持样条清空, GUI 只画直线折线 path_。
                            return_cruise_ = channel_speed_;
                            leg_speed_     = channel_speed_;
                            bspline_ = UniformBspline2D();
                            updateStage1Cruise(dt);
                        } else {
                            rebuildBspline(return_cruise_);
                        }
                        printf("[GH] blocked/grazing ahead → re-plan leg (stage %d, cruise %.2f)\n",
                               (int)return_stage_,
                               return_stage_ == 1 ? leg_speed_ : return_cruise_);
                    } else if (have_result) {
                        // ★重规划失败/不可用 → 继续飞当前路径(不再原地悬停等死)
                        printf("[GH] replan 不可用(stage %d) → 继续沿当前路径飞\n",
                               (int)return_stage_);
                    }
                }
            }

            // --- Phase 2: fly the current return leg ---
            if (!path_.empty()) {
                move_elapsed_ += dt;
                double cruise_use = return_cruise_;
                if (return_stage_ == 1) {
                    updateStage1Cruise(dt);   // A→B: 开阔 0.4 / 窄口自动降速
                    cruise_use = leg_speed_;
                } else {
                    // 去 A 段分两种: 到探索角(整定)不减速、和探索一样快;
                    // 只有目标确实是 pad A(已过角点)才在最后 1.5m 收油。
                    const bool target_is_A = home_corner_done_;
                    double rem0 = 1e18;
                    if (target_is_A) {
                        size_t ci0 = 0; double b0 = 1e18;
                        for (size_t i = 0; i < path_.size(); ++i) {
                            double d2 = (path_[i].x-drone_.x)*(path_[i].x-drone_.x)
                                      + (path_[i].y-drone_.y)*(path_[i].y-drone_.y);
                            if (d2 < b0) { b0 = d2; ci0 = i; }
                        }
                        rem0 = 0.0;
                        for (size_t i = ci0; i + 1 < path_.size(); ++i)
                            rem0 += path_[i].dist(path_[i+1]);
                    }
                    if (rem0 < 1.5) {                       // 仅 pad A 前 1.5m
                        double frac = std::clamp(rem0 / 1.5, 0.03, 1.0);
                        double v = go_home_speed_ * frac;
                        if (v > 0.35) v = 0.35;
                        if (v < 0.08) v = 0.08;
                        double lim = 1.2 * std::max(dt, 0.02);
                        if (leg_speed_ <= 1e-6) leg_speed_ = go_home_speed_;
                        leg_speed_ = (v > leg_speed_)
                                     ? std::min(v, leg_speed_ + lim)
                                     : std::max(v, leg_speed_ - lim);
                        retimeBspline(leg_speed_);
                        cruise_use = leg_speed_;
                    }
                }
                // A→B(stage1): 轨迹 = A* 规划出的走廊折线本身; 设定点按"沿折线弧长"
                // 推进 + 切向速度前馈 —— 实际速度 = leg_speed_(参数值), 不再被
                // PX4 位置环的 P 项限速(旧实现把速度前馈清零, 只发"机位前方0.18m"
                // 的位置点, 实际只有 P*0.18≈0.17m/s, channel_speed 完全不生效)。
                if (return_stage_ == 1)
                    cmd = polylineTrack(cruise_use, dt);
                else {
                    cmd = posTrack(cruise_use);
                }
                if (return_stage_ == 1 || (return_stage_ == 0 && home_corner_done_)) {
                    // ★过探索区角点之后(去 padA 段 + A→B 段): 机头【固定朝 -y】, 不转向。
                    //   原来 stage0 用 posTrack() → cmd.yaw = 路径切线 ✗ → 飞 A 的过程中
                    //   机头一直在跟着路径微调(有偏航速度) → 通道口附近尤其危险。
                    cmd.yaw   = -M_PI / 2.0;
                    yaw_last_ = cmd.yaw;     // 同步, 避免其他分支用回旧切向角
                    // vel_x/vel_y 是折线切向 * leg_speed(本体系无关, 与 position
                    // 同为 FRAME_LOCAL_NED), 保留作为速度前馈。
                }
                // 到本段路径终点的真实距离(m) —— 直接算距离, 不做"距离平方 vs 阈值平方"
                double dist_home = std::sqrt((drone_.x-path_.back().x)*(drone_.x-path_.back().x)
                                           + (drone_.y-path_.back().y)*(drone_.y-path_.back().y));
                double actual_sp = std::sqrt(drone_vx*drone_vx + drone_vy*drone_vy);
                const double ARRIVE_R = 0.10;   // 到点判定半径: 距路径终点 10cm 内算到达
                // 所有到达判定都只按位置(距路径终点 <10cm), 不看速度:
                //   角点 → padA → A→B(A* 规划到 padB, 或 mode2 到"第一口-0.5m")
                // 到 A→B 的目标点就 FINISHED(主循环收尾用 land 服务原地降落)。
                // DIAG: arrival / hold issue — log distance & speed near the pad.
                if (++gh_dbg_ % 20 == 0)
                    printf("[GH2] t=%.2f dist=%.3f pend=(%.2f,%.2f) drone=(%.2f,%.2f) cmd=(%.2f,%.2f) yaw=%.2f stg=%d "
                           "planned=%d yalign=%d bg(kind=%d run=%d done=%d) fail=%d path=%zu\n",
                           move_elapsed_, dist_home,
                           path_.back().x, path_.back().y, drone_.x, drone_.y,
                           cmd.pos_x, cmd.pos_y, cmd.yaw, (int)return_stage_,
                           (int)home_planned_, (int)home_yaw_align_,
                           bg_kind_, (int)bg_fut_.valid(), (int)bg_gh_done_,
                           gh_plan_fail_, path_.size());
                // ★★ "轨迹已走完但离本段目标还差一点" → 直接用位置环飞完这个点, 不再等规划:
                //   实飞出现过: 样条时间走完、飞机停在离角点 0.5m 处一直悬停(既不降高也不去
                //   padA)。原因就是"只在样条推进时才给设定点"; 样条到头后设定点虽然等于终点,
                //   但只要出现稳态误差/卡滞就没有别的机制把它拉过去。这里加一条硬保证:
                //   路程时间已耗尽 && 距本段目标 0.10~1.0m → 直接把 path_.back() 作为位置环设定点。
                //   适用范围(按现场要求): 仅"返航第一段(还没过左上角点)" 且 距角点 <0.70m。
                if (return_stage_ == 0 && !home_corner_done_ && !path_.empty()
                    && bspline_.duration() > 0.01
                    && move_elapsed_ >= bspline_.duration() - 1e-3
                    && dist_home > ARRIVE_R && dist_home < 0.70) {
                    cmd.pos_x = path_.back().x;
                    cmd.pos_y = path_.back().y;
                    cmd.pos_z = 0.0;
                    cmd.vel_x = 0.0; cmd.vel_y = 0.0; cmd.vel_z = 0.0;
                    cmd.yaw   = (home_corner_done_ ? -M_PI / 2.0 : yaw_last_);   // ★过角点后固定 -y
                    yaw_last_ = cmd.yaw;
                    cmd.valid = true;
                    if (++gh_direct_log_ % 25 == 1)
                        printf("[GH] 轨迹已走完, 距左上角点 %.2fm → 位置环直接飞该点(不依赖规划)\n",
                               dist_home);
                }

                // ★"到位+停稳即算到达"兜底(只用于返航各段终点):
                //   螺旋桨/位置环的稳态误差、地图里角点附近被贴边格子挤掉一点, 都可能让飞机
                //   停在离 path_.back() 11~20cm 的地方 —— 那时 10cm 判定永不成立, 于是
                //   (a)角点判定不过 → 不进入降高阶段 → 在上空(1.2m)原地悬停;
                //   (b)padA/padB 判定不过 → 一直悬停。这里补一条: 已在 0.35m 内且速度很慢
                //   持续 1.5s → 视为到达(正常情况都在 10cm 内, 不受影响)。
                {
                    const double v_now = std::sqrt(drone_vx*drone_vx + drone_vy*drone_vy);
                    // ★阈值放宽到 0.80m + 停稳 2s: 实飞出现过"停在离角点 0.55m 处一直悬停,
                    //   既不降高也不去 padA"(角点只是整定/刹车点, 差几十厘米不影响后续)。
                    // 角点(整定/刹车点)放宽到 0.80m; padA/padB 收紧到 0.25m(别让落点差太多)
                    const double near_tol = (return_stage_ == 0 && !home_corner_done_) ? 0.80 : 0.25;
                    if (dist_home < near_tol && v_now < 0.06) gh_near_t_ += dt;
                    else                                      gh_near_t_ = 0.0;
                    gh_near_tol_ = near_tol;
                }
                const bool arrived = (dist_home < ARRIVE_R) || (gh_near_t_ > 2.0);
                if (arrived && gh_near_t_ > 2.0 && dist_home >= ARRIVE_R) {
                    if (++gh_near_log_ % 5 == 1)
                        printf("[GH] 离目标点 %.2fm 处已停稳 2s → 视为到达(容差兜底, 阈值%.2fm)\n",
                               dist_home, gh_near_tol_);
                }
                if (arrived) {
                    gh_near_t_ = 0.0;
                    // --- arrived at current leg target ---
                    if (return_stage_ == 0 && !home_corner_done_ && flight_mode_ != 3) {
                        // 第0步(mode 0 / mode 1 都有): 已到探索区角点(整定/刹车点),
                        // 停稳后再飞 pad A。
                        printf("[GH] reached explore corner (%.2f,%.2f) → fly pad A (%.1f,%.1f)\n",
                               EXPLORE_X_MAX - 0.5, EXPLORE_Y_MAX - 0.5, pad_a_x_, pad_a_y_);
                        home_corner_done_ = true;
                        home_planned_ = false;
                        move_elapsed_ = 0.0;
                        cmd = Cmd{};      // hover this tick; next tick plans pad A leg
                        break;
                    }
                    if (return_stage_ == 0 && (flight_mode_ == 1 || flight_mode_ == 2)) {
                        // mode 1/2: pad A is only the corridor-entry waypoint.
                        // 进入走廊段(stage1): 对准 -y → 用 A* 规划到 A→B 目标。
                        printf("[GH] reached pad A (%.1f,%.1f) → corridor (mode %d)\n",
                               pad_a_x_, pad_a_y_, flight_mode_);
                        return_stage_ = 1;
                        home_planned_ = false;
                        move_elapsed_ = 0.0;
                        st1_aligned_     = false;   // 先对准 -y, 再规划 A→B
                        st1_await_align_ = false;
                        cmd = Cmd{};      // hover this tick; next tick aligns then plans
                        break;
                    }
                    if (return_stage_ == 1) {
                        // A→B 目标到点: mode2 = 第一个口出清点; mode1 = padB。
                        printf("[GH] reached A→B goal (%s) at (%.2f,%.2f) → LAND here\n",
                               st1_goal_is_gate_ ? "mode2 第一口-0.5m" : "padB",
                               goal_.x, goal_.y);
                        cmd = Cmd{};
                        state_ = FINISHED;
                        break;
                    }
                    // mode 0 / mode 3(回起飞点): 到点即降落
                    printf("[GH] reached goal (mode %d) at (%.2f,%.2f) → LAND here\n",
                           flight_mode_, goal_.x, goal_.y);
                    cmd = Cmd{};
                    state_ = FINISHED;
                }
            } else { state_ = FINISHED; }
            break;
        }
        case FINISHED: break;
        }
        return cmd;
    }

    void markObstacle(double wx, double wy) { map_.set(worldToGrid(wx, wy), CS_OCCUPIED); }
    void clearObstacles() { map_.clearObstacles(); }
    /// ★★ 只清【探索区外】的障碍格(探索区内累加, 不消失) —— 现场要求:
    ///   探索区: 见过就留着 ✓ 薄障碍/柱子不会因为某一帧没扫到而消失 ✗ → 不再"停一下又撞" ✓
    ///   探索区外(通道): 仍然每帧全清重建 ✓ → 裁判/人走进通道, 走开就消失 ✓✓
    void clearObstaclesOutsideExplore() {
        const double x0 = EXPLORE_X_MIN, x1 = EXPLORE_X_MAX;
        const double y0 = EXPLORE_Y_MIN, y1 = EXPLORE_Y_MAX;
        for (int gy = 0; gy < GRID_N; ++gy) {
            for (int gx = 0; gx < GRID_N; ++gx) {
                if (map_.get(gx, gy) != CS_OCCUPIED) continue;
                Point w = gridToWorld(gx, gy);
                const bool in_explore = (w.x >= x0 && w.x <= x1 && w.y >= y0 && w.y <= y1);
                if (!in_explore) map_.set(gx, gy, CS_UNKNOWN);   // 只清探索区外 ✓
            }
        }
    }

    /// 细缝填充: 点云残缺会让薄墙(挡板)中间断 1~2 格, 规划会从缺口穿墙。
    /// 只把“左右(行)/上下(列)都被障碍夹住、缺口≤2 格”的小洞补成障碍;
    /// 真实 0.75m 通道有 7~8 格不会被填, 孤立噪点也不会被填。
    void fillObstacleHoles() {
        // 横向(每行): 两个占用列之间 ≤2 空列 → 填
        for (int gy = 0; gy < GRID_N; ++gy) {
            int lastOcc = -10;
            for (int gx = 0; gx < GRID_N; ++gx) {
                if (map_.get(gx, gy) == CS_OCCUPIED) {
                    if (gx - lastOcc >= 2 && gx - lastOcc <= 3)   // 缺口 1~2 格
                        for (int k = lastOcc + 1; k < gx; ++k)
                            map_.set(k, gy, CS_OCCUPIED);
                    lastOcc = gx;
                }
            }
        }
        // 纵向(每列): 同理
        for (int gx = 0; gx < GRID_N; ++gx) {
            int lastOcc = -10;
            for (int gy = 0; gy < GRID_N; ++gy) {
                if (map_.get(gx, gy) == CS_OCCUPIED) {
                    if (gy - lastOcc >= 2 && gy - lastOcc <= 3)
                        for (int k = lastOcc + 1; k < gy; ++k)
                            map_.set(gx, k, CS_OCCUPIED);
                    lastOcc = gy;
                }
            }
        }
    }

    void clearObstaclesInSector(double dx, double dy, double dyaw) {
        map_.clearObstaclesInSector(dx, dy, dyaw);
    }

    /// After FAST_LIO Laser_map arrives, mark all in-bounds non-obstacle cells
    /// as CS_FREE so the planner knows which areas still need camera coverage.
    /// Only cells inside the exploration sub-region [EXPLORE_*] are marked:
    /// the landing corridor east of the internal wall (x>7.5) is deliberately
    /// left CS_UNKNOWN so it is never chosen as an exploration target.  The
    /// corridor is only flown through (allowUnknown=true) during GO_HOME.
    void markFreeSpace() {
        // Explore sub-region is inside the full map rectangle → clamp so the
        // worldToGrid conversion stays valid even at the boundaries.
        double ex0 = std::clamp(EXPLORE_X_MIN, MAP_X_MIN, MAP_X_MAX);
        double ex1 = std::clamp(EXPLORE_X_MAX, MAP_X_MIN, MAP_X_MAX);
        double ey0 = std::clamp(EXPLORE_Y_MIN, MAP_Y_MIN, MAP_Y_MAX);
        double ey1 = std::clamp(EXPLORE_Y_MAX, MAP_Y_MIN, MAP_Y_MAX);
        int gx_min = worldToGrid(ex0 + GRID_RES, 0).x;
        int gx_max = worldToGrid(ex1 - GRID_RES, 0).x;
        int gy_min = worldToGrid(0, ey0 + GRID_RES).y;
        int gy_max = worldToGrid(0, ey1 - GRID_RES).y;
        for (int y = gy_min; y <= gy_max; ++y)
            for (int x = gx_min; x <= gx_max; ++x) {
                CellState s = map_.get(x, y);
                if (s == CS_UNKNOWN) map_.set(x, y, CS_FREE);
            }
    }

    void triggerStart() { if (state_ == INIT) state_ = DETECT; }

    /// 暂缓"探索进度达标自动返航": 视觉标靶任务还没做完时置 true, 探索器就算
    /// 覆盖率达标也不自己 GO_HOME(继续扫/继续飞剩下的视觉目标); 视觉任务全部
    /// 处理完(或再无可飞目标)后由外部置 false 恢复正常。
    void setHoldHome(bool h) { hold_home_ = h; if (!h) hold_home_warned_ = false; }
    /// A→B 速度前馈比例(0=退回"只发位置点"的旧行为, 会很慢; 1=按参数速度飞)。
    /// 留这个口子是因为早期实飞出现过"速度前馈把飞机往西带"的现象(疑似坐标系),
    /// 现场若再遇到侧偏可临时调 0 或 0.5。
    void setStage1VelFF(double s) { st1_vel_ff_ = std::clamp(s, 0.0, 1.5); }
    /// 立即返航(忽略暂缓): 用于"标靶数已飞满 → 直接返回"。
    void forceGoHome() {
        hold_home_ = false;
        hold_home_warned_ = false;
        if (state_ != GO_HOME && state_ != FINISHED) {
            printf("[FSM] vision targets full → force GO_HOME (explored %.1f%%)\n",
                   map_.exploredPercent());
            beginGoHome();
        }
    }

    /// A→B 走廊段规划(= 探索同一套 A*, 只是不做 EGO/样条平滑 + 用走廊边缘距离)。
    /// 与"有几个窄墙"完全无关: 目标就是 padB, A* 自己按地图几何穿 0/1/N 个口。
    /// 边缘距离先按 AB_EDGE_MARGIN(launch ab_edge_margin), 若该余量下走廊无解
    /// (现场口比参数更窄/地图里板子更厚), 每次放宽 5cm 重试; 下限 = min(参数, 0.30m)
    /// —— 你显式把参数设得更小时以你为准(现场口窄就调小), 默认 0.35 时下限 0.30
    /// (再低就小于机身半径+栅格量化, 宁可让上层悬停重试也不给必然擦墙的路径)。
    std::vector<Point> planCorridor(const Point& from, const Point& to) {
        return planCorridorOn(map_, from, to, AB_EDGE_MARGIN);
    }
    /// ★通道路径方式 1: 先x再y 的轴对齐路径。
    ///   做法: 取通道内所有窄口中点(从当前位置往目标方向), 逐个: 先横移(x)到口子中线,
    ///   再沿 -y 穿过去, 并且"过口后再多走 overshoot(默认0.7m)"才进行下一次横移;
    ///   过完最后一个口之后(不再有窄墙) 直接斜线飞目标点。返回空表示不可用(调用方退回 A*)。
    static std::vector<Point> buildAxisCorridor(const std::vector<Point>& gates,
                                                const Point& from, const Point& to,
                                                double overshoot, double lead = 1.2) {
        std::vector<Point> p;
        p.push_back(from);
        double curx = from.x, cury = from.y;
        for (const auto& g : gates) {
            // ★★ 现场要求: 【提前 lead 米并线】—— 在口子【上方 lead 米】就斜着并到口中线 ✓
            //    原来: 先横移到 x_g (直角 ✗) 再往下 → 转弯全集中在口子跟前 ✗
            //          → B样条只能到跟前才磨圆 → 到窄处还在拐 → 惯性甩出去蹭墙 ✓
            //    现在: 转弯发生在【离口子还有 lead 米的开阔处】✓ 到口子时已经在正中 ✓✓
            if (std::abs(g.x - curx) > 1e-3) {
                double yMerge = g.y + lead;
                if (yMerge > cury) yMerge = cury;        // 别跑到当前位置身后
                if (cury - yMerge > 0.05)
                    p.push_back(Point(g.x, yMerge));     // ★斜线提前并到中线 ✓
                else
                    p.push_back(Point(g.x, cury));       // 已经在口子附近 → 只能横移
                curx = g.x;
                cury = yMerge;
            }
            double ny = g.y - overshoot;                 // 过口后再多走 overshoot
            if (to.y > ny) ny = to.y;                    // 别越过最终目标
            if (std::abs(ny - cury) > 1e-3) { p.push_back(Point(curx, ny)); cury = ny; }
        }
        // ★过完最后一个口(后面没有窄墙了) → 直接斜线飞 B(不再先x再y)
        if (std::hypot(to.x - curx, to.y - cury) > 1e-3) p.push_back(to);
        return p;
    }
    /// 轴对齐折线自检: 沿线取样, 要求【离墙 ≥ 机体半径 + 余量】(原来只查占用格、
    /// 不做膨胀 → 允许贴着墙走 → 窄墙口处侧边会蹭墙)。
    static bool axisCorridorClear(const GridMap& m, const std::vector<Point>& p) {
        if (p.size() < 2) return false;
        // 需要的净空: 机体半径0.25 + 安全余量0.05(走廊里贴墙会擦到侧边)
        const double need = DRONE_RADIUS + 0.05;
        const int rr = std::max(1, (int)std::ceil(need / GRID_RES));   // 半径≈6格
        for (size_t i = 0; i + 1 < p.size(); ++i) {
            double L = p[i].dist(p[i+1]);
            int n = std::max(2, (int)(L / (GRID_RES * 0.5)));
            for (int k = 0; k <= n; ++k) {
                double t = (double)k / n;
                double wx = p[i].x + (p[i+1].x - p[i].x) * t;
                double wy = p[i].y + (p[i+1].y - p[i].y) * t;
                GridPos gc = worldToGrid(wx, wy);
                if (!gc.valid()) return false;
                // 检查以该点为中心、半径 rr 的圆盘内有没有占用格(≈ 到墙距离 ≥ need)
                bool hit = false;
                for (int dy = -rr; dy <= rr && !hit; ++dy) {
                    for (int dx = -rr; dx <= rr; ++dx) {
                        if (dx * dx + dy * dy > rr * rr) continue;
                        GridPos g(gc.x + dx, gc.y + dy);
                        if (!g.valid() || m.get(g) == CS_OCCUPIED) { hit = true; break; }
                    }
                }
                if (hit) return false;
            }
        }
        return true;
    }

    /// ★带地图版本(后台线程用快照; 用局部 PathPlanner, 不碰成员 planner_)
    static std::vector<Point> planCorridorOn(const GridMap& m, const Point& from,
                                             const Point& to, double base,
                                             int mode = 0, double overshoot = 0.7,
                                             const std::vector<Point>& gates = {},
                                             double lead = 1.2) {
        if (mode == 1) {                       // ★先x再y 的轴对齐路径(过口再多走 overshoot)
            auto ax = buildAxisCorridor(gates, from, to, overshoot, lead);
            if (ax.size() >= 2 && axisCorridorClear(m, ax)) {
                printf("[GH] corridor plan: mode1(先x再y, 过口+%.2fm) path=%zu\n", overshoot, ax.size());
                return ax;
            }
            printf("[GH] corridor plan: mode1 轴对齐路径不可用 → 退回 A*\n");
        }
        AB_EDGE_MARGIN = base;               // thread_local: 只改本线程副本 ✓
        // ★★ 现场要求: 【保留自动降余量】✓ 参数默认 0.35 ✓ 算不出路径就逐级往下放宽 ✓
        //    原来 FLOOR = min(base, 0.30) ✗ → base=0.35 时最低只到 0.30 ✗ 
        //    而 0.75m 窄口需要 2×(margin+0.10) ≤ 0.75 ⇒ margin ≤ 0.275 ✗ 
        //    ⇒ 0.30 根本不够 ✓ → 永远 NO PATH ✗✓✓
        //    现在: FLOOR = min(base, 0.10) ✓ 且步长改成 0.05 逐级降 ✓
        //          base=0.35 → 依次试 0.35/0.30/0.25/0.20/0.15/0.10 ✓
        //          0.10 时只需净宽 2×0.20 = 0.40m ✓ 0.75m 口子轻松过 ✓✓
        const double FLOOR = std::min(base, 0.10);
        std::vector<Point> p;
        PathPlanner pl;
        for (double relax : {0.0, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35}) {
            const double mm = std::max(FLOOR, base - relax);
            AB_EDGE_MARGIN = mm;                     // planner 内部读这个(本线程)量
            // push_target = 边缘距离 + 10cm: 把路径从"贴着约束边界"推到通道中间
            // (两侧墙交替推开 → 居中), 真实间隙从 m-5cm 提到 ~m+5cm。
            p = pl.planConservative(m, from, to, true, mm + 0.10);
            if (!p.empty()) {
                if (relax > 0.0)
                    printf("[GH] corridor plan: margin %.2f→%.2f (relaxed %.2f) path=%zu\n",
                           base, mm, relax, p.size());
                else
                    printf("[GH] corridor plan: margin %.2f path=%zu\n", mm, p.size());
                break;
            }
            if (mm <= FLOOR) break;                  // 已到下限 → 不再放宽
        }
        AB_EDGE_MARGIN = base;
        if (p.empty())
            printf("[GH] corridor plan: NO PATH at margin %.2f..%.2f — 口可能比参数窄, "
                   "把 launch 的 ab_edge_margin 调小再试\n", base, FLOOR);
        return p;
    }

    /// 视觉任务结束后恢复探索: 强制回到 DETECT 重新找目标/规划,
    /// 丢弃任务期间可能已过期的旧路径与样条。
    void restartExplore() {
        path_.clear();
        bspline_ = UniformBspline2D();
        move_elapsed_ = 0.0;
        replan_timer_ = 0.0;
        obstacle_check_timer_ = 0.0;
        state_ = DETECT;
    }

    // ===== 外部目标(视觉识别点)接口: 走 A* 规划, 可绕开中间障碍 =====
    /// 更新无人机当前位姿(视觉任务期间不跑 update(), 需手动同步)。
    void setDronePose(double x, double y, double yaw) {
        drone_.x = x; drone_.y = y; drone_.heading = yaw;
    }
    /// 用 A* 规划一条从当前位置到 (tx,ty) 的路径并生成样条。
    /// 返回 false = 规划失败(目标不可达/被障碍包围)。
    // ================================================================
    //  异步规划(控制线程不再被 A* 阻塞)
    //  设计: 重规划在后台线程里、对"地图快照"计算, 算完把路径交给控制线程
    //        取用 → 后台只读快照, 控制线程独占地图写, 因此不需要加锁。
    //        算法完全不变(还是同一个 plan()/planConservative())。
    // ================================================================
    /// 是否已有后台规划任务在算
    bool bgBusy() const { return bg_fut_.valid(); }

    /// 节点每拍同步"当前是否有视觉任务": 置 false 时, 之后到达的视觉旧结果会被丢弃,
    /// 防止(visionFinishTarget/放弃目标之后)迟到的路径把探索路径覆盖掉。
    /// 设置通道路径规划方式(launch): 0=保守A*, 1=先x再y(过口+overshoot)
    void setAbPathMode(int mode, double overshoot, double x_speed, double lead = -1.0) {
        if (lead > 0.0) ab_axis_lead_ = lead;   // ★提前并线距离(launch ab_axis_lead)
        ab_path_mode_ = (mode == 1) ? 1 : 0;
        ab_axis_overshoot_ = (overshoot > 0.0) ? overshoot : 0.7;
        ab_axis_x_speed_   = (x_speed > 0.05) ? x_speed : 0.25;
        printf("[GH] corridor path mode = %d %s (mode1过口后多走 %.2fm, 横移限速 %.2fm/s; 走廊边距=ab_edge_margin)\n",
               ab_path_mode_, ab_path_mode_ == 1 ? "先x再y轴对齐" : "保守A*(斜线穿口中点)",
               ab_axis_overshoot_, ab_axis_x_speed_);
    }

    void setVisionActive(bool a) {
        if (vision_active_ && !a) {
            ++vision_gen_;                         // 任务结束 → 代号+1, 迟到的旧结果自然被丢
            bg_vis_want_ = false;                  // ★任务已结束 → 解除"视觉在等规划", 延长任务恢复
            bg_vis_queued_ = false;
        }
        vision_active_ = a;
    }

    /// 取后台结果: -1=失败(不可达) 0=在算/未提交 1=成功(路径已生效)。视觉用。
    int planToPointStatus() const { return bg_pt_result_; }
    bool planToPointPending() const { return bg_fut_.valid() && bg_kind_ == 2; }

    /// 异步"飞到某点"(视觉目标): 立即返回, 不阻塞控制线程。
    void planToPointAsync(double tx, double ty) {
        // ★先置"视觉在等规划"标志: 延长任务看到它就让路(不再每拍抢后台槽)。
        //   以前这里静默 return, 结果视觉任务永远提交不进去 → "有视觉数据却一直不飞视觉"。
        bg_vis_want_   = true;
        bg_vis_queued_ = true;                       // ★记下"这次没提交成功", 槽空了补投
        bg_pt_goal_    = Point(tx, ty);
        if (bg_fut_.valid()) {
            if (++bg_busy_log_ % 25 == 1)
                printf("[BG] 后台忙(kind=%d) → 视觉目标规划排队(槽空了自动补投, 不丢任务)\n", bg_kind_);
            return;                                  // 已有任务在算 → 排队, 由 pollBgPlan 补投
        }
        bg_pt_gen_    = vision_gen_;        // 记录提交时的任务代号
        bg_pt_result_ = 0;
        startBgPlan(2, Point(drone_.x, drone_.y), bg_pt_goal_);
        bg_vis_queued_ = false;
    }

    /// 控制线程每拍调用: 后台算好了就把结果应用进来(耗时只是取 future, 微秒级)
    void pollBgPlan() {
        if (!bg_fut_.valid()) {
            // ★视觉目标规划"补投": 提交那次可能正好被"路径延长"占着槽没成功 —— 槽一空
            //   立刻补上。否则节点只在锁定目标时提交一次(提交失败后再没人重试) →
            //   表现就是"视觉明明有数据, 却一直不飞视觉, 3s后放弃该目标"。
            if (bg_vis_queued_) {
                bg_vis_queued_ = false;
                bg_pt_gen_    = vision_gen_;
                if (bg_pt_result_ == 0)
                    startBgPlan(2, Point(drone_.x, drone_.y), bg_pt_goal_);
            }
            return;
        }
        if (bg_fut_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) return;
        BgPlan res = bg_fut_.get();
        bg_fut_ = std::future<BgPlan>();
        const int kind = bg_kind_; bg_kind_ = 0;
        if (kind == 1)      applyReplanHead(std::move(res));      // MOVE 头部重规划
        else if (kind == 2) { applyVisionPath(std::move(res)); bg_vis_want_ = false; bg_vis_queued_ = false; }
        else if (kind == 3) applyExtend(std::move(res));          // 轨迹延长
        else if (kind == 4 || kind == 5 || kind == 6) {           // 返航 leg / 走廊 / 新目标
            bg_gh_path_ = std::move(res.path);
            bg_gh_goal_ = res.goal;
            bg_gh_ok_   = res.ok;
            bg_gh_done_ = true;
        }
    }

    bool planToPoint(double tx, double ty) {
        path_ = planner_.plan(map_, Point(drone_.x, drone_.y), Point(tx, ty), true);
        if (path_.size() < 2) { path_.clear(); return false; }
        goal_ = Point(tx, ty);
        move_elapsed_ = 0.0;
        rebuildBspline(DRONE_SPEED);
        return true;
    }
    /// 沿 planToPoint 规划的路径跟踪一拍(位置+速度前馈+切线偏航)。
    Cmd trackPlanned(double dt) {
        move_elapsed_ += dt;
        return posTrack(DRONE_SPEED);
    }
    /// 当前规划路径是否仍然有内容(跟踪途中可判断)。
    bool hasPlannedPath() const { return path_.size() >= 2; }

private:
    // ================================================================
    //  Return-to-landing helpers
    // ================================================================
    /// Enter GO_HOME from exploration: always reset to the pad-A return stage.
    /// (pad A→pad B inside GO_HOME is advanced explicitly in the case body.)
    void beginGoHome() {
        if (hold_home_) {          // 视觉标靶任务还没做完 → 先不返航(外部会清掉)
            if (!hold_home_warned_) {
                hold_home_warned_ = true;
                printf("[FSM] hold home: 视觉标靶未处理完, 暂不返航 (explored %.1f%%)\n",
                       map_.exploredPercent());
            }
            return;
        }
        state_ = GO_HOME;
        return_stage_  = 0;
        return_cruise_ = go_home_speed_;
        home_planned_  = false;
        home_yaw_align_ = false;
        home_corner_done_ = false;   // 每次回家先重置"探索区左上角"子目标
        st1_goal_is_gate_ = false;
        st1_aligned_      = false;   // A→B: 抵达 padA 后先对准 -y 再规划
        st1_await_align_  = false;
        st1_last_y_      = 1e9;
        st1_s_           = 0.0;      // A→B 弧长设定点复位
    }

    /// A→B 返航窄口居中: 挡板是沿 y 的竖板, 窄不窄看"左右两侧障碍的 x 差"。
    /// 对路径每个点所在行(y)扫描 x 方向的自由区间(被 CS_OCCUPIED 列隔出的
    /// 连续空列); 当前点所在自由区间宽度 <1.0m → 窄通道, 把该点 x 拉到区间
    /// 正中; ≥1.0m(宽通道)不动。
    void centerNarrowGates(std::vector<Point>& path) const {
        const double NARROW_W = 1.0;      // 自由区间宽 <1.0m 判窄
        for (size_t i = 1; i + 1 < path.size(); ++i) {
            GridPos base = worldToGrid(path[i].x, path[i].y);
            if (!base.valid()) continue;
            const int qy = base.y;
            // 逐列标记占用
            std::vector<bool> occ(GRID_N, false);
            int lo = 0, hi = GRID_N - 1;
            for (int gx = lo; gx <= hi; ++gx)
                occ[gx] = (map_.get(gx, qy) == CS_OCCUPIED);
            // 找当前列所在自由区间
            const int gxc = base.x;
            if (occ[gxc]) continue;       // 当前在障碍上, 跳过
            int a = gxc, b = gxc;
            while (a - 1 >= lo && !occ[a - 1]) --a;
            while (b + 1 <= hi && !occ[b + 1]) ++b;
            double w = (double)(b - a + 1) * GRID_RES;   // 自由区间宽度(m)
            if (w < NARROW_W) {
                double xa = gridToWorld(a, qy).x;
                double xb = gridToWorld(b, qy).x;
                path[i].x = (xa + xb) * 0.5;             // 拉到口子正中
            }
        }
    }

    /// Smallest free lateral margin (beyond the DRONE_RADIUS body) along a path:
    /// distance from every path vertex to the nearest CS_OCCUPIED cell minus the
    /// body half-size.  ~0 → the drone just grazes obstacles; bigger = safer.
    double pathMinClearance(const std::vector<Point>& path) const {
        const int R = std::max(2, (int)std::round(0.7 / GRID_RES));   // ★±0.7m(按分辨率换算, 原写死7格)
        double min_d = 1e9;
        for (const Point& w : path) {
            GridPos g = worldToGrid(w.x, w.y);
            for (int dy = -R; dy <= R; ++dy)
                for (int dx = -R; dx <= R; ++dx) {
                    GridPos q{g.x + dx, g.y + dy};
                    if (!q.valid()) continue;
                    if (map_.get(q) != CS_OCCUPIED) continue;
                    double d = std::sqrt(double(dx * dx + dy * dy)) * GRID_RES;
                    if (d < min_d) min_d = d;
                }
        }
        double free_m = min_d - DRONE_RADIUS;  // net margin past the body edge
        return std::max(free_m, 0.0);
    }

    /// Minimum clearance across genuine TWO-SIDED gates only (obstacles close
    /// on BOTH sides of the path — a randomly placed baffle pair).  Walls that
    /// the corridor path legitimately hugs on ONE side do NOT count, so the
    /// A→B cruise is slowed only where a real narrow door must be threaded.
    double pathMinGateClearance(const std::vector<Point>& path) const {
        const double SEARCH = 1.0;
        const double NARROW = 1.10;          // 两侧格心距和 <1.1 ≈ 实际缝 <1.0m 判窄
        const int W = 8;                     // search window ±0.8 m
        double min_g = 1e9;
        for (size_t i = 1; i + 1 < path.size(); ++i) {
            double ux = path[i+1].x - path[i-1].x;
            double uy = path[i+1].y - path[i-1].y;
            double ul = std::sqrt(ux*ux + uy*uy);
            if (ul < 1e-6) continue;
            ux /= ul; uy /= ul;
            const double nx = -uy, ny = ux;
            double dp = 1e9, dm = 1e9;
            GridPos g = worldToGrid(path[i].x, path[i].y);
            if (!g.valid()) continue;
            for (int dy = -W; dy <= W; ++dy)
                for (int dx = -W; dx <= W; ++dx) {
                    GridPos q{g.x + dx, g.y + dy};
                    if (!q.valid()) continue;
                    if (map_.get(q) != CS_OCCUPIED) continue;
                    Point c = gridToWorld(q.x, q.y);
                    double lat = (c.x - path[i].x) * nx + (c.y - path[i].y) * ny;
                    if (lat > 0.01 && lat < dp) dp = lat;
                    if (lat < -0.01 && -lat < dm) dm = -lat;
                }
            if (dp < SEARCH && dm < SEARCH && (dp + dm) < NARROW) {
                double g_free = std::min(dp, dm) - DRONE_RADIUS;
                if (g_free < min_g) min_g = g_free;
            }
        }
        return std::max(min_g, 0.0);
    }

    /// Pro-active gate check for the return legs: true when the remaining path
    /// (up to look_m ahead) would come within DRONE_RADIUS of an obstacle, or
    /// run through an occupied cell — i.e. the plan would graze/cut a randomly
    /// placed baffle.  Re-planning THEN (early) gives the drone lateral room to
    /// clear the board's end before it is dragged down into the board band.
    bool pathGrazeAhead(double look_m) const {
        if (path_.size() < 2) return false;
        size_t s = 0; double best = 1e18;
        for (size_t i = 0; i < path_.size(); ++i) {
            double d2 = (path_[i].x - drone_.x) * (path_[i].x - drone_.x)
                      + (path_[i].y - drone_.y) * (path_[i].y - drone_.y);
            if (d2 < best) { best = d2; s = i; }
        }
        double walked = 0.0;
        const int R = 5;                        // search ±0.5 m for obstacles
        for (size_t i = s; i + 1 < path_.size(); ++i) {
            double dx = path_[i + 1].x - path_[i].x;
            double dy = path_[i + 1].y - path_[i].y;
            double len = std::sqrt(dx * dx + dy * dy);
            int steps = std::max(2, (int)(len / (GRID_RES * 0.4)));
            for (int k = 0; k <= steps; ++k) {
                double wx = path_[i].x + dx * k / steps;
                double wy = path_[i].y + dy * k / steps;
                if (k > 0 && k < steps) {
                    walked += len / steps;
                    if (walked > look_m) return false;
                }
                GridPos g = worldToGrid(wx, wy);
                if (!g.valid()) continue;
                for (int dy2 = -R; dy2 <= R; ++dy2)
                    for (int dx2 = -R; dx2 <= R; ++dx2) {
                        GridPos q{g.x + dx2, g.y + dy2};
                        if (!q.valid()) continue;
                        if (map_.get(q) != CS_OCCUPIED) continue;
                        double d = std::sqrt(double(dx2*dx2 + dy2*dy2)) * GRID_RES;
                        if (d < DRONE_RADIUS - 0.18)   // 真快撞(净距<~0.07)才触发
                            return true;               // 居中后 0.125m 余量不会误触发
                    }
            }
        }
        return false;
    }

    /// Cruise speed for a return leg given its minimum clearance: the tighter the
    /// gate, the slower the fly-through (less lateral tracking error / cut).
    /// mc = min(两侧净距) - DRONE_RADIUS: 路径居中过 0.8m 口时 ≈0.15。
    double gateCruise(double min_clear, double base) const {
        if (min_clear >= 0.30) return base;                    // 宽通道: 全速
        if (min_clear >= 0.18) return std::min(base, 0.40);
        if (min_clear >= 0.08) return std::min(base, 0.25);    // 正常窄口: 0.25
        return std::min(base, 0.15);                           // 极窄: 爬过去
    }

    /// A→B: 把整条(已优化)B 样条时间轴从 bsp_cruise_ 缩放到新巡航。
    /// 只缩放 interval —— 几何不变、避障结论沿用;move_elapsed_ 同比例
    /// 锚定,当前位置/前视点连续,只有速率变化。
    void retimeBspline(double cruise) {
        if (bspline_.numCtrl() < 4) { bsp_cruise_ = cruise; return; }
        if (bsp_cruise_ <= 1e-6)    { bsp_cruise_ = cruise; return; }
        double k = bsp_cruise_ / cruise;
        if (k < 0.05) k = 0.05;
        if (k > 20.0) k = 20.0;
        if (std::abs(k - 1.0) < 0.01) return;
        double t0 = move_elapsed_;
        bspline_ = UniformBspline2D(bspline_.ctrlPoints(), bspline_.interval() * k);
        move_elapsed_ = t0 * k;
        bsp_cruise_ = cruise;
    }

    /// A→B 前方弧长窗口 [skip_m, skip_m+look_m] 内“双侧窄门”的最小净距
    /// (总缝(格心距)<1.1 ≈ 实际 <1.0m 才算窄门;单侧贴墙不计)。窗口内无门返回 1e9。
    double pathGateClearanceAhead(double skip_m, double look_m) const {
        const double SEARCH = 1.0;
        const double NARROW = 1.10;          // 总缝(格心距)<1.1 ≈ 实际 <1.0m 判窄
        const int W = 8;                    // 每点 ±0.8 m 侧向搜索
        double min_g = 1e9;
        if (path_.size() < 2) return min_g;

        size_t ci = 0; double bd2 = 1e18;
        for (size_t i = 0; i < path_.size(); ++i) {
            double d2 = (path_[i].x-drone_.x)*(path_[i].x-drone_.x)
                      + (path_[i].y-drone_.y)*(path_[i].y-drone_.y);
            if (d2 < bd2) { bd2 = d2; ci = i; }
        }
        double walked = 0.0;
        for (size_t i = ci; i + 1 < path_.size(); ++i) {
            double ux = path_[i+1].x - path_[i].x;
            double uy = path_[i+1].y - path_[i].y;
            double ul = std::sqrt(ux*ux + uy*uy);
            if (ul < 1e-9) continue;
            ux /= ul; uy /= ul;
            double seg = 0.0;
            {   // 段弧长(与下方走步一致)
                seg = path_[i].dist(path_[i+1]);
            }
            double a0 = walked; walked += seg;
            if (walked < skip_m) continue;
            if (a0 > skip_m + look_m) break;
            const double nx = -uy, ny = ux;
            // 段中点近似做门检测
            const double wx = path_[i].x + ux * std::max(0.5*seg, skip_m - a0);
            const double wy = path_[i].y + uy * std::max(0.5*seg, skip_m - a0);
            GridPos g = worldToGrid(wx, wy);
            if (!g.valid()) continue;
            double dp = 1e9, dm = 1e9;
            for (int dy = -W; dy <= W; ++dy)
                for (int dx = -W; dx <= W; ++dx) {
                    GridPos q{g.x + dx, g.y + dy};
                    if (!q.valid()) continue;
                    if (map_.get(q) != CS_OCCUPIED) continue;
                    Point c = gridToWorld(q.x, q.y);
                    double lat = (c.x - wx)*nx + (c.y - wy)*ny;
                    if (lat > 0.01 && lat < dp) dp = lat;
                    if (lat < -0.01 && -lat < dm) dm = -lat;
                }
            if (dp < SEARCH && dm < SEARCH && (dp + dm) < NARROW) {
                double g_free = std::min(dp, dm) - DRONE_RADIUS;
                if (g_free < min_g) min_g = g_free;
            }
        }
        return min_g;
    }

    /// A→B 每拍速度决策: 开阔段 channel_speed_(0.4); 前方 ~1.2m 出现双侧窄门
    /// 按净距降到 0.3/0.2/0.15; 速率限幅(≈0.8 m/s²)后按需 retime 样条。
    /// A→B 段不做"末端收油"(到目标点就降落, 不收油), 所以窄口之外全程 0.4。
    void updateStage1Cruise(double dt_tick) {
        constexpr double LOOK     = 1.2;   // 减速判定窗: 提前 ~1.2m 开始降(原来2.5m
                                           // 时, 8m 走廊+两个口会让整段都处于慢态)
        constexpr double EXIT     = 1.5;   // 退出慢态的远窗
        constexpr double RAMP     = 0.8;   // m/s² 速率变化上限
        constexpr double NEAR_PAD = 0.5;   // 非 A→B 段: 目标点前收油距离

        // ★mode1(先x再y): 横移段(x 方向)必须限速 —— 横移末端要拐回 -y, 沿 x 冲太快
        //   刹不住(会冲过口子撞墙)。用线段投影找当前段, 横移段或 0.45m 内即将进横移段 → 限速。
        bool lat_cap = false;
        if (ab_path_mode_ == 1 && path_.size() >= 2) {
            size_t k = 0; double bd2 = 1e18; double tt_best = 0.0;
            for (size_t i = 0; i + 1 < path_.size(); ++i) {
                const Point& a = path_[i]; const Point& b = path_[i+1];
                double vx = b.x - a.x, vy = b.y - a.y, L2 = vx*vx + vy*vy;
                double tt = (L2 > 1e-12) ? ((drone_.x-a.x)*vx + (drone_.y-a.y)*vy) / L2 : 0.0;
                tt = std::clamp(tt, 0.0, 1.0);
                double px = a.x + tt*vx, py = a.y + tt*vy;
                double d2 = (px-drone_.x)*(px-drone_.x) + (py-drone_.y)*(py-drone_.y);
                if (d2 < bd2) { bd2 = d2; k = i; tt_best = tt; }
            }
            const Point& a = path_[k]; const Point& b = path_[k+1];
            const double dx = std::abs(b.x - a.x), dy = std::abs(b.y - a.y);
            const double seg_len = std::hypot(b.x - a.x, b.y - a.y);
            const bool lateral   = (dx > dy);              // 该段主要沿 x → 横移段
            const bool near_lat  = (seg_len * (1.0 - tt_best) < 0.45);  // 马上要拐进下一段
            lat_cap = (lateral || near_lat);                // 交给下面统一压 des(由斜坡减速)
        }

        double mc = pathGateClearanceAhead(0.10, LOOK);
        double des = channel_speed_;
        if (mc < 1e6) {
            des = gateCruise(mc, channel_speed_);
            slow_active_ = true;
        } else if (slow_active_) {
            double mcf = pathGateClearanceAhead(0.10, EXIT);
            if (mcf >= 1e6) slow_active_ = false;
            else des = gateCruise(mcf, channel_speed_);
        }
        // 到路径终点剩余弧长(A→B 段不收油: 到点即降落; 其它段末端收油便于到达判定)
        double rem = 1e18;
        if (!path_.empty()) {
            size_t ci2 = 0; double b2 = 1e18;
            for (size_t i = 0; i < path_.size(); ++i) {
                double d2 = (path_[i].x-drone_.x)*(path_[i].x-drone_.x)
                          + (path_[i].y-drone_.y)*(path_[i].y-drone_.y);
                if (d2 < b2) { b2 = d2; ci2 = i; }
            }
            rem = 0.0;
            for (size_t i = ci2; i + 1 < path_.size(); ++i)
                rem += path_[i].dist(path_[i+1]);
        }
        // ★mode1 横移段限速: 压"期望速度"(des), 由原有斜坡按 RAMP m/s² 平滑降到该值
        if (lat_cap && return_stage_ == 1) {
            des = std::min(des, ab_axis_x_speed_);
            if (++st1_latcap_log_ % 25 == 0)
                printf("[GH] 横移段限速 %.2f m/s (当前 %.2f)\n", ab_axis_x_speed_, leg_speed_);
        }
        if (return_stage_ != 1) {
            if (rem < NEAR_PAD) des = std::min(des, 0.20);
            if (rem < 0.25)     des = std::min(des, 0.12);
        }
        if (des < 0.08) des = 0.08;

        double lim = RAMP * std::max(dt_tick, 0.02);
        if (leg_speed_ < 1e-6) leg_speed_ = des;
        double next = (des > leg_speed_) ? std::min(des, leg_speed_ + lim)
                                         : std::max(des, leg_speed_ - lim);
        leg_speed_ = next;
        retimeBspline(leg_speed_);
    }

    /// A→B 前方窗口内窄门(两侧都近、总缝<1.1)的“偏心量” |dm-dp| 最大值。
    /// >0 表示路径没走在口子中间(挡板规划后才进地图时典型), 返回 0 表示
    /// 无窄门或已居中。
    double pathGateBiasAhead(double skip_m, double look_m) const {
        const double SEARCH = 1.0;
        const double NARROW = 1.10;
        const int W = 8;
        double max_bias = 0.0;
        if (path_.size() < 2) return 0.0;
        size_t ci = 0; double bd2 = 1e18;
        for (size_t i = 0; i < path_.size(); ++i) {
            double d2 = (path_[i].x-drone_.x)*(path_[i].x-drone_.x)
                      + (path_[i].y-drone_.y)*(path_[i].y-drone_.y);
            if (d2 < bd2) { bd2 = d2; ci = i; }
        }
        double walked = 0.0;
        for (size_t i = ci; i + 1 < path_.size(); ++i) {
            double ux = path_[i+1].x - path_[i].x;
            double uy = path_[i+1].y - path_[i].y;
            double ul = std::sqrt(ux*ux + uy*uy);
            if (ul < 1e-9) continue;
            ux /= ul; uy /= ul;
            double seg = path_[i].dist(path_[i+1]);
            double a0 = walked; walked += seg;
            if (walked < skip_m) continue;
            if (a0 > skip_m + look_m) break;
            const double nx = -uy, ny = ux;
            const double wx = path_[i].x + ux * 0.5 * seg;
            const double wy = path_[i].y + uy * 0.5 * seg;
            GridPos g = worldToGrid(wx, wy);
            if (!g.valid()) continue;
            double dp = 1e9, dm = 1e9;
            for (int dy = -W; dy <= W; ++dy)
                for (int dx = -W; dx <= W; ++dx) {
                    GridPos q{g.x + dx, g.y + dy};
                    if (!q.valid()) continue;
                    if (map_.get(q) != CS_OCCUPIED) continue;
                    Point c = gridToWorld(q.x, q.y);
                    double lat = (c.x - wx)*nx + (c.y - wy)*ny;
                    if (lat > 0.01 && lat < dp) dp = lat;
                    if (lat < -0.01 && -lat < dm) dm = -lat;
                }
            if (dp < SEARCH && dm < SEARCH && (dp + dm) < NARROW) {
                double bias = std::abs(dm - dp);
                if (bias > max_bias) max_bias = bias;
            }
        }
        return max_bias;
    }

    /// 某一行(y)上找 A→B 走廊里的“窄口”: 把该行在走廊横向带内的占用格合成
    /// 实心段(墙/板), 找“相邻两段”之间净空 ∈ [0.5, 1.0]m 的口子, 取净空最小
    /// 的一对, cx = 两段之间正中。中点只在两段实心占用之间, 不会落在障碍里。
    /// 板在左/在右/两侧各 0.35 都适用(找的是两墙之间 ≤1.0m 的空隙)。
    /// 检测带稍微扩过常驻墙实体但止于大厅挡板之前(x≥7.3), 大厅不参与。
    bool rowGateCenter(int qy, double& cx) const {
        auto colOf = [](double wx) {
            return std::clamp((int)std::floor((wx - MAP_X_MIN) / GRID_RES), 0, GRID_N - 1);
        };
        // ★★ 现场场地改造后: 通道是【斜的】(padA→padB 有 ~13° 倾角) ✗
        //    原来写死 corridor_x0_=9.1/x1_=10.34 是【旧场地】的 ✗ → 门检测扫错区域 ✗
        //    → 改成【从 padA/padB 自动推导横向带】✓ 斜通道/直通道都适用 ✓✓
        const double _bx_lo = std::min(pad_a_x_, pad_b_x_) - 0.35;
        const double _bx_hi = std::max(pad_a_x_, pad_b_x_) + 0.35;
        int gx0 = colOf(std::max(_bx_lo, corridor_x0_ - 0.10));   // 含西侧墙实体
        int gx1 = colOf(std::min(_bx_hi, corridor_x1_ + 0.15));   // 含东侧墙实体
        auto leftEdge  = [](int gx) { return MAP_X_MIN + gx * GRID_RES; };
        auto rightEdge = [](int gx) { return MAP_X_MIN + (gx + 1) * GRID_RES; };
        std::vector<std::pair<double, double>> iv;   // 占用实心段物理边界
        int s = -1;
        for (int gx = gx0; gx <= gx1; ++gx) {
            bool occ = (map_.get(gx, qy) == CS_OCCUPIED);
            if (occ) { if (s < 0) s = gx; }
            else if (s >= 0) { iv.push_back({leftEdge(s), rightEdge(gx - 1)}); s = -1; }
        }
        if (s >= 0) iv.push_back({leftEdge(s), rightEdge(gx1)});
        const double yWorld = gridToWorld(0, qy).y;
        // 相邻两实心段之间的净空 ∈[0.5,1.0], 取最小(最窄真口子)
        double bestGap = 1e9, bestMid = 0.0; bool found = false;
        for (size_t i = 0; i + 1 < iv.size(); ++i) {
            double gap = iv[i + 1].first - iv[i].second;   // 两墙之间净空
            if (gap < 0.5 || gap > 1.0) continue;
            double mid = (iv[i].second + iv[i + 1].first) * 0.5;
            GridPos gm = worldToGrid(mid, yWorld);
            if (!gm.valid()) continue;
            if (map_.get(gm) == CS_OCCUPIED) continue;     // 中点必在两墙之间的空里
            // 中点列在相邻行也空 → 通道真的沿 -y 穿透(排除单行伪缝)
            bool through = true;
            for (int yy = -2; yy <= 2; ++yy) {
                if (yy == 0) continue;
                if (map_.get(gm.x, qy + yy) == CS_OCCUPIED) { through = false; break; }
            }
            if (!through) continue;
            // ★★ A: 口子两侧必须是【沿 y 连续的真墙】(否则: 虚拟假墙劈出来的缝 ✗、
            //    裁判/人站在通道里形成的点团 ✗ 都会被当成口 ✗✓)
            //    判据: 两个边界墙在 ±2 行(±5cm) 内仍然存在 ✓
            bool walls_cont = true;
            for (int yy = -2; yy <= 2 && walls_cont; ++yy) {
                if (yy == 0) continue;
                int q2 = qy + yy;
                if (q2 < 0 || q2 >= GRID_N) { walls_cont = false; break; }
                auto occAt = [&](double wx) {
                    GridPos g = worldToGrid(wx, gridToWorld(0, q2).y);
                    return g.valid() && map_.get(g) == CS_OCCUPIED;
                };
                // 在两侧墙的【内侧边缘】各退 2 格检测(墙可能有 1~2 格厚度差)
                if (!occAt(iv[i].second - GRID_RES) || !occAt(iv[i+1].first + GRID_RES)) {
                    walls_cont = false;
                }
            }
            if (!walls_cont) continue;
            if (gap < bestGap) { bestGap = gap; bestMid = mid; found = true; }
        }
        if (found) {
            cx = bestMid;
            if (std::abs(yWorld - gh_gate_dbg_y_) > 0.3) {
                gh_gate_dbg_y_ = yWorld;
                printf("[GATE] y=%.2f drone_y=%.2f nseg=%zu gap=%.2f mid=%.2f occ=%d\n",
                       yWorld, drone_.y, iv.size(), bestGap, bestMid,
                       map_.get(worldToGrid(bestMid, yWorld)) == CS_OCCUPIED);
            }
        }
        return found;
    }

    /// A→B 实时“窄口中线”扫描: 沿 y 向前(look_m 内)找第一个窄墙对行,
    /// 返回两墙中点 x。找不到 → found=false。
    struct LaneAhead { bool found = false; double cx = 0.0; double rowY = 0.0; };
    LaneAhead corridorLaneAhead(double look_m) const {
        LaneAhead out;
        double startY = drone_.y - GRID_RES;           // 沿 −y 前进
        double endY   = drone_.y - look_m;
        for (double yy = startY; yy > endY; yy -= GRID_RES) {
            GridPos g0 = worldToGrid(0.0, yy);
            if (!g0.valid()) break;
            double cx = 0.0;
            if (rowGateCenter(g0.y, cx)) {
                out.found = true; out.cx = cx; out.rowY = yy;
                return out;
            }
        }
        return out;
    }

    /// 找整条走廊里每个窄口(两窄墙 x 间隔 ≤1.0m)。
    /// 逐行调用 rowGateCenter: 该行已保证“中点=x距离最小的两窄墙正中间,
    /// 且该中点格不是占用”。输出直接用它自己的 (该行cx, 该行y) ——
    /// x 和 y 来自同一行、且都验证过, 不会拼出墙里的点。不做带平均。
    /// A→B: 把走廊里每个窄口压成“一个”口中点返回(按 y 从近到远)。
    /// 连续的多行(isGate)属于同一物理口子 → 只输出一个代表点, 不会出现
    /// 一行一个点、几十个微变点串成弧线的现象。
    /// 代表点 = 该带内各行 cx 的中位数行; 输出点须不在占用格上。
    std::vector<Point> gateCentersBetween(double startY, double endY) const {
        std::vector<Point> gates;
        std::vector<Point> band;         // 当前口子带的候选行 (cx, 该行y)
        auto flushBand = [&]() {
            if (band.empty()) return;
            // 每行独立验证过 → 取带内 y 中位的那行作为口中点(与带外 y 不同源,
            // 但 cx 在带内几乎恒定, 不会拼出墙内点; 再查一次占用保险)
            size_t bm = band.size() / 2;
            Point best = band[bm];
            GridPos g = worldToGrid(best.x, best.y);
            if (g.valid() && map_.get(g) != CS_OCCUPIED)
                gates.push_back(best);
            band.clear();
        };
        for (double yy = startY; yy > endY; yy -= GRID_RES) {
            GridPos g0 = worldToGrid(0.0, yy);
            if (!g0.valid()) break;
            double cx = 0.0;
            if (!rowGateCenter(g0.y, cx)) { flushBand(); continue; }
            double midY = gridToWorld(0, g0.y).y;
            if (!band.empty() && std::abs(cx - band.back().x) > 0.35)
                flushBand();             // x 突变 → 新的口子
            band.push_back(Point(cx, midY));
        }
        flushBand();
        return gates;
    }

    /// 两点之间按 GRID_RES 步长内插的密直线(不含中间点之外的任何绕行)。
    std::vector<Point> denseLine(const Point& a, const Point& b) const {
        std::vector<Point> d;
        double seg = a.dist(b);
        int n = std::max(1, (int)(seg / GRID_RES));
        for (int k = 0; k < n; ++k)
            d.push_back(Point(a.x + (b.x - a.x) * k / n,
                              a.y + (b.y - a.y) * k / n));
        d.push_back(b);
        return d;
    }

    /// A→B 走廊路径: 起点 → 每个检测到的窄通道中点(按 y 顺序) → 终点。
    /// 不加任何"提前 N m 拐弯"假设 —— 比赛挡板间距未知, 提前量无法拍脑袋。
    /// 航点就是窄口本身的 (cx, 该口行y): 轨迹从起点直线飞向口中点, 到口子时
    /// 恰好居中穿过。加密成密折线后交样条/跟踪平滑, 但航点必须过口中线。
    std::vector<Point> buildCorridorPath(const Point& start, const Point& goal) const {
        std::vector<Point> via;
        via.push_back(start);
        std::vector<Point> gates = gateCentersBetween(start.y, goal.y);
        for (auto& g : gates) via.push_back(g);
        via.push_back(goal);
        std::vector<Point> dense;
        for (size_t i = 0; i + 1 < via.size(); ++i) {
            double seg = via[i].dist(via[i+1]);
            int n = std::max(1, (int)(seg / GRID_RES));
            for (int k = 0; k < n; ++k) {
                dense.push_back(Point(via[i].x + (via[i+1].x - via[i].x) * k / n,
                                      via[i].y + (via[i+1].y - via[i].y) * k / n));
            }
        }
        dense.push_back(goal);
        return dense;
    }

    /// EGO-style periodic LOCAL re-plan while MOVE: A* again from the CURRENT
    /// drone position to a look-ahead point ~lookahead_m along the existing
    /// path, then re-attach the untouched tail.  This keeps the sweeping intent
    /// of the (extended) path intact while the head route is continuously
    /// re-optimised / obstacle-aware in real time. Returns true if swapped.
    // ---------- 异步规划内部实现 ----------
    /// 一次性提交后台任务: kind 1 = MOVE头部重规划(带旧尾部 tail); kind 2 = 到目标点。
    /// A* + B样条EGO平滑全部在后台完成(都只用地图快照 → 不需要加锁)。
    void startBgPlan(int kind, Point a, Point b, std::vector<Point> tail = {}) {
        GridMap snap = map_;            // 地图快照(控制线程独占写地图, 这里拷一份; ~几十µs)
        const double cruise = DRONE_SPEED;
        bg_kind_ = kind;
        auto job = [snap, a, b, tail, cruise]() -> BgPlan {
                BgPlan r;
                PathPlanner pp;                     // 后台自己的实例(PathPlanner 无状态)
                std::vector<Point> raw = pp.plan(snap, a, b, true);
                if (raw.size() >= 2) {
                    if (!tail.empty()) {            // MOVE: 与旧路径尾部拼接
                        Point endp = raw.back();
                        size_t tj = 0; double td2 = 1e18;
                        for (size_t i = 0; i < tail.size(); ++i) {
                            double d2 = tail[i].distSq(endp);
                            if (d2 < td2) { td2 = d2; tj = i; }
                        }
                        r.path = raw;
                        r.path.reserve(raw.size() + (tail.size() - tj));
                        for (size_t i = tj; i < tail.size(); ++i) r.path.push_back(tail[i]);
                    } else {
                        r.path = std::move(raw);
                    }
                    if (r.path.size() >= 2) {
                        // EGO 平滑也在这里做完(原来在控制线程里做, 要 5~20ms)
                        r.bspline = UniformBspline2D::fromPath(r.path, cruise);
                        BsplineOptimizer2D::Params bsp;
                        bsp.max_vel   = cruise;
                        bsp.max_acc   = MAX_ACCEL;
                        bsp.safe_dist = (INFLATE_RADIUS + 4) * GRID_RES;
                        BsplineOptimizer2D::optimize(snap, r.bspline, bsp);
                        r.ok = true;
                    }
                }
                return r;
        };
        // ★同步模式(默认, 和"没有独立进程那版"行为一致): 提交时就地算完, 直接应用结果。
        //   结构上不可能出现"后台槽被占/排队/等结果悬停/规划超时放弃"这些问题;
        //   代价是这一拍会阻塞规划耗时(2Hz 重规划 ⇒ 每 500ms 阻塞一次)。
        // ★返航 leg/走廊(kind 4/5)一律走后台线程:
        //   (a) 它们是最重的规划(A* 全图+走廊), 同步会在主循环里卡住几十秒
        //       —— 现场"到左上角后一直悬停、连每秒状态行都没有"就是这个 ✗;
        //   (b) 同步分支里没有 kind4/5 的 apply(会被误当 applyExtend)。
        const bool heavy_gohome = (kind == 4 || kind == 5);
        if (bg_sync_ && !heavy_gohome && (kind != 6 || bg_sync_target_)) {
            BgPlan r = job();
            if      (kind == 1) applyReplanHead(std::move(r));
            else if (kind == 2) { applyVisionPath(std::move(r)); bg_vis_want_ = false; bg_vis_queued_ = false; }
            else                applyExtend(std::move(r));
            return;
        }
        bg_fut_ = std::async(std::launch::async, job);
    }
    /// 应用"MOVE 头部重规划"的结果(路径+B样条都已在后台算好, 这里只搬运)
    void applyReplanHead(BgPlan r) {
        if (!r.ok || r.path.size() < 2) return;
        if (state_ != MOVE) return;          // ★已离开 MOVE(如转返航) → 丢弃过期结果
        printf("[RPL] 重规划生效 @(%.2f,%.2f) → goal (%.2f,%.2f) path=%zu\n",
               drone_.x, drone_.y, r.path.back().x, r.path.back().y, r.path.size());
        path_ = std::move(r.path);
        bspline_ = std::move(r.bspline);
        bsp_cruise_ = DRONE_SPEED;
        goal_ = path_.back();
        move_elapsed_ = 0.0;
    }
    /// 应用"轨迹延长"的结果(路径+B样条后台算好)
    void applyExtend(BgPlan r) {
        if (state_ != MOVE) return;
        const double old_dur = bspline_.duration();     // 旧样条时长(下面夹时间参数用)
        ext_wait_ = 0.0;
        if (!r.ok || r.path.size() < 2) return;    // 没找到延长点 → 到末端走 DETECT
        path_ = std::move(r.path);
        bspline_ = std::move(r.bspline);
        bsp_cruise_ = DRONE_SPEED;
        goal_ = path_.back();
        stuck_timer_ = 1e-6;
        // ★换上更长的样条后, 把时间参数夹回"旧样条末端"对应的时间(悬停期间时钟已不再前进,
        //   这里只是兜底), 避免设定点前跳导致猛冲。
        if (move_elapsed_ > old_dur) move_elapsed_ = old_dur;
        printf("[EXT] 延长生效 → 路径 %zu 点, 新目标 (%.2f,%.2f)\n",
               path_.size(), goal_.x, goal_.y);
    }

    /// 应用"视觉目标点"的规划结果
    void applyVisionPath(BgPlan r) {
        if (!vision_active_ || bg_pt_gen_ != vision_gen_) { bg_pt_result_ = 0; return; }  // 过期结果丢弃
        if (!r.ok || r.path.size() < 2) {
            // ★重规划失败时【保留正在飞的那条路径】(它是之前规划出来、已验证可通行的):
            //   以前这里 path_.clear() → posTrack() 返回无效 → VIS_FLY 兜底把设定点直接
            //   指向目标 → 出现"直线飞目标、中途撞障碍"(小概率, 因为要正好赶上一次重规划
            //   失败 + 直线上有障碍)。
            if (path_.size() >= 2) { bg_pt_result_ = 1; return; }   // 继续沿旧路径飞
            path_.clear(); bg_pt_result_ = -1; return;              // 首次规划就失败 → 放弃目标
        }
        path_ = std::move(r.path);
        bspline_ = std::move(r.bspline);
        bsp_cruise_ = DRONE_SPEED;
        goal_ = bg_pt_goal_;
        move_elapsed_ = 0.0;
        bg_pt_result_ = 1;
    }
    /// ★新目标选择也交给后台: 原来在 DETECT/PLAN 里最多试 40 个候选点、每个都同步
    ///   跑一次 A*(最坏能卡 1 秒以上)。现在把"逐个候选试规划"整段放后台, 控制线程
    ///   在 PLAN 期间原地悬停(本来就在等), 结果到了再走原来的后续逻辑。
    void startTargetPlanJob(std::vector<Point> pool, Point dp) {
        if (bg_fut_.valid()) return;
        GridMap snap = map_;
        bg_kind_ = 6;
        // ★按值捕获打分要用的量(后台线程不能抓 this)
        const double cap_hp  = heading_penalty_;
        const double cap_hdg = drone_.heading;
        const int    cap_topn = std::max(1, far_first_topn_);
        const double cap_sc  = far_first_safe_clear_;
        const double cap_sw  = far_first_safe_w_;
        auto job = [snap, pool, dp, cap_hp, cap_hdg, cap_topn, cap_sc, cap_sw]() -> BgPlan {
            BgPlan r;
            PathPlanner pp;
            // ★★ 不再"第一个能通的就用", 而是评估候选池里前 topn 个:
            //   score = 实际A*路径长度 × (1 + 航向惩罚) × 安全惩罚
            //   安全惩罚: 路径沿途离障碍越近分越高(越不优先) —— 避免为了"选远处"而贴墙钻缝。
            const size_t max_try = std::min(pool.size(), (size_t)cap_topn);
            double best_sc = 1e30;
            for (size_t i = 0; i < max_try; ++i) {
                Point goal = pool[i];
                goal.x = std::clamp(goal.x, EXPLORE_X_MIN + EDGE_MARGIN, EXPLORE_X_MAX - EDGE_MARGIN);
                goal.y = std::clamp(goal.y, EXPLORE_Y_MIN + EDGE_MARGIN, EXPLORE_Y_MAX - EDGE_MARGIN);
                auto raw = pp.plan(snap, dp, goal, true);
                const double L = raw.empty() ? 0.0 : Px4ExplorationPlanner::pathLength(raw);
                if (raw.empty() || L < 0.5) continue;
                const double clr = Px4ExplorationPlanner::pathMinClearance(snap, raw);  // 沿途最小离障(m)
                const double aa  = std::atan2(goal.y - dp.y, goal.x - dp.x);
                double dh = std::abs(aa - cap_hdg);
                if (dh > PI) dh = 2*PI - dh;
                const double need = std::max(0.05, cap_sc);
                const double safe_pen = 1.0 + cap_sw
                                      * std::max(0.0, need - clr) / need;
                const double sc = L * (1.0 + cap_hp * (1.0 - std::cos(dh))) * safe_pen;
                if (sc < best_sc) {
                    best_sc = sc;
                    r.path = std::move(raw);
                    r.goal = goal;
                    r.ok   = true;
                }
            }
            // ★★ 兜底尝试【必须有上限】: 原来最多再试 38 个候选、每个一次完整 A* ✗
            //    → GRID_RES=2.5cm(格子×4) 时会卡好几秒 = "飞完一条轨迹悬停等规划" ✓
            //    候选池已按"远处优先(x≥1) + 距离/航向/密度"排好序 ✓ → 前几个不行就别死磕,
            //    直接返回"本轮没选到", 由外层下一拍/下一轮再选 ✓ (绝不长时间阻塞控制线程 ✓)
            if (!r.ok) {
                const size_t fallback_max = std::min(pool.size(), max_try + 3);
                for (size_t i = max_try; i < fallback_max; ++i) {
                    Point goal = pool[i];
                    goal.x = std::clamp(goal.x, EXPLORE_X_MIN + EDGE_MARGIN, EXPLORE_X_MAX - EDGE_MARGIN);
                    goal.y = std::clamp(goal.y, EXPLORE_Y_MIN + EDGE_MARGIN, EXPLORE_Y_MAX - EDGE_MARGIN);
                    auto raw = pp.plan(snap, dp, goal, true);
                    if (!raw.empty() && Px4ExplorationPlanner::pathLength(raw) >= 0.5) {
                        r.path = std::move(raw); r.goal = goal; r.ok = true; break;
                    }
                }
            }
            return r;
        };
        if (bg_sync_ && bg_sync_target_) {             // 同步: 就地算完

            BgPlan r = job();
            bg_gh_path_ = std::move(r.path);
            bg_gh_goal_ = r.goal;
            bg_gh_ok_   = r.ok;
            bg_gh_done_ = true;
            return;
        }
        bg_fut_ = std::async(std::launch::async, job);
    }

    /// ★返航段/走廊规划也交给后台: kind 4 = 探索区内 leg(A*), kind 5 = 走廊(planCorridorOn)。
    /// 返航段的 B 样条/速度/对准仍由原逻辑在控制线程做(它们依赖腿的速度与阶段判断),
    /// 这里只把最贵的 A* 挪走。
    void startGoHomePlanJob(int kind, Point from, Point to, int stride = 1) {
        GridMap snap = map_;
        const double margin = AB_EDGE_MARGIN;      // 本线程当前值(走廊"逐步放宽"的基准)
        const int    mode   = ab_path_mode_;       // 通道路径方式(0=A*, 1=先x再y)
        const double over   = ab_axis_overshoot_;
        const double lead   = ab_axis_lead_;      // ★提前并线距离
        std::vector<Point> gates;                  // mode1: 口中点在控制线程算(便宜), 随机任务带入
        if (kind == 5 && mode == 1 && from.y > to.y) gates = gateCentersBetween(from.y, to.y);
        bg_kind_ = kind;
        auto job = [snap, from, to, margin, kind, mode, over, gates, stride, lead]() -> BgPlan {
                BgPlan r;
                if (kind == 5) {
                    r.path = Px4ExplorationPlanner::planCorridorOn(snap, from, to, margin, mode, over, gates, lead);
                } else {
                    PathPlanner pp;
                    // ★★ 长距离返航用搜索步长 stride(=3格=7.5cm): A* 节点数少 9 倍 →
                    //    密集地图上从 0.5~0.8s 降到 ~50ms ✓ (碰撞检查仍逐格 2.5cm ✓ 不会穿墙)
                    //    A→B 走廊(kind5)保持 stride=1 全精度 ✓
                    r.path = pp.plan(snap, from, to, true, stride);
                    if (r.path.size() < 2) {
                        // ★常规规划(含探索区膨胀 0.55m)失败 → 退回"保守规划"(只用 ab_edge_margin,
                        //   0.30m)。探索区通往通道的入口往往只有 1.0~1.5m 宽, 膨胀光晕过不去 →
                        //   以前会重试 20 次后放弃并原地降落, 现在能过去。
                        double saved = AB_EDGE_MARGIN;
                        r.path = Px4ExplorationPlanner::planCorridorOn(snap, from, to, saved);
                        if (r.path.size() >= 2)
                            printf("[GH] 常规规划失败 → 保守规划(边距%.2f)成功 path=%zu\n",
                                   saved, r.path.size());
                    }
                }
                r.ok = (r.path.size() >= 2);
                return r;                              // 不建样条: 返航逻辑自己按腿速度建
        };
        if (bg_sync_) {                                // ★同步模式: 就地算完, 直接放结果槽
            BgPlan r = job();
            bg_gh_path_ = std::move(r.path);
            bg_gh_goal_ = r.goal;
            bg_gh_ok_   = r.ok;
            bg_gh_done_ = true;
            return;
        }
        bg_fut_ = std::async(std::launch::async, job);
    }

    /// ★轨迹延长也交给后台: 原来在 MOVE 里同步试最多10个候选点(A*+平滑),
    ///   最坏会卡 200~300ms。现在快照+后台算, 控制线程继续飞当前路径。
    void startExtendJob() {
        if (bg_fut_.valid()) return;
        GridMap snap = map_;
        std::vector<Point> cur_path = path_;
        const double yaw    = drone_.heading;
        const double cruise = DRONE_SPEED;
        bg_kind_ = 3;
        auto job = [snap, cur_path, yaw, cruise]() -> BgPlan {
                BgPlan r;
                if (cur_path.size() < 2) return r;
                const Point ep = cur_path.back();
                PathPlanner pp;
                std::vector<Point> best; Point tgt;
                int n_cand = 0, n_rej = 0;
                auto candidates = Px4ExplorationPlanner::findNearestUnvisitedN(snap, ep, yaw, 10);
                for (const auto& cand : candidates) {
                    if (ep.distSq(cand) < 0.5*0.5) continue;
                    Point target = cand;
                    target.x = std::clamp(target.x, EXPLORE_X_MIN + EDGE_MARGIN, EXPLORE_X_MAX - EDGE_MARGIN);
                    target.y = std::clamp(target.y, EXPLORE_Y_MIN + EDGE_MARGIN, EXPLORE_Y_MAX - EDGE_MARGIN);
                    auto ext = pp.plan(snap, ep, target, true);
                    ++n_cand;
                    if (ext.empty() || Px4ExplorationPlanner::pathLength(ext) < 0.5) { ++n_rej; continue; }
                    if (!Px4ExplorationPlanner::isValidExtension(ext, ep, cur_path, target)) { ++n_rej; continue; }
                    best = std::move(ext); tgt = target; break;
                }
                if (best.size() < 2 && n_cand > 0)      // ★汇总一行, 不再每个候选刷屏
                    printf("[EXT] 本批 %d 个候选全部无效(%d 被拒/不可达) → 等下次重试\n",
                           n_cand, n_rej);
                if (best.size() < 2) {          // 兜底: 机头方向的最近可去点
                    Point target = Px4ExplorationPlanner::findNearestUnvisited(snap, ep, yaw);
                    if (!(target.x == 0 && target.y == 0)) {
                        target.x = std::clamp(target.x, EXPLORE_X_MIN + EDGE_MARGIN, EXPLORE_X_MAX - EDGE_MARGIN);
                        target.y = std::clamp(target.y, EXPLORE_Y_MIN + EDGE_MARGIN, EXPLORE_Y_MAX - EDGE_MARGIN);
                        if (ep.distSq(target) > 0.5*0.5) {
                            auto ext = pp.plan(snap, ep, target, true);
                            if (!ext.empty() && Px4ExplorationPlanner::pathLength(ext) >= 0.5) {
                                best = std::move(ext); tgt = target;
                            }
                        }
                    }
                }
                if (best.size() < 2) return r;  // ok=false → 到末端后走 DETECT
                r.path = cur_path;
                r.path.reserve(cur_path.size() + best.size());
                for (size_t i = 1; i < best.size(); ++i) r.path.push_back(best[i]);
                r.bspline = UniformBspline2D::fromPath(r.path, cruise);
                BsplineOptimizer2D::Params bsp;
                bsp.max_vel = cruise; bsp.max_acc = MAX_ACCEL;
                bsp.safe_dist = (INFLATE_RADIUS + 4) * GRID_RES;
                BsplineOptimizer2D::optimize(snap, r.bspline, bsp);
                r.ok = true;
                return r;
        };
        if (bg_sync_) {                                // ★同步模式: 就地算完(候选通常1~3个)
            applyExtend(std::move(job()));
            return;
        }
        bg_fut_ = std::async(std::launch::async, job);
    }

    /// MOVE 里"头部重规划"的前半段(只做便宜的取点/检查, 真正的 A* 交给后台)
    void startMoveReplan() {
        if (bg_fut_.valid()) return;
        if (path_.size() < 2) return;
        if (map_.exploredPercent() >= home_pct_) return;
        Point cur(drone_.x, drone_.y);
        size_t ci = 0; double bd2 = 1e18;
        for (size_t i = 0; i < path_.size(); ++i) {
            double d2 = path_[i].distSq(cur);
            if (d2 < bd2) { bd2 = d2; ci = i; }
        }
        double old_rem = 0.0;
        for (size_t i = ci; i + 1 < path_.size(); ++i) old_rem += path_[i].dist(path_[i+1]);
        if (old_rem < 1.0) return;
        Point lp = findLookAheadFrom(path_, ci, 2.0);
        if (cur.distSq(lp) < 0.5*0.5) return;
        std::vector<Point> tail(path_.begin() + ci, path_.end());   // 旧尾部快照(后台拼接用)
        startBgPlan(1, cur, lp, std::move(tail));
    }

    bool tryReplanToGoal() {
        if (path_.size() < 2) return false;
        if (map_.exploredPercent() >= home_pct_) return false;
        Point cur(drone_.x, drone_.y);

        // Closest index on the old path to the current drone position.
        size_t ci = 0; double bd2 = 1e18;
        for (size_t i = 0; i < path_.size(); ++i) {
            double d2 = path_[i].distSq(cur);
            if (d2 < bd2) { bd2 = d2; ci = i; }
        }
        double old_rem = 0.0;
        for (size_t i = ci; i + 1 < path_.size(); ++i)
            old_rem += path_[i].dist(path_[i+1]);
        if (old_rem < 1.0) return false;   // near end — extension handles it

        // Look-ahead point on the OLD path (keeps the intended direction &
        // coverage tail beyond it). Skip points behind us / too close.
        double look = 2.0;   // metres of head we re-plan over
        Point lp = findLookAheadFrom(path_, ci, look);
        if (cur.distSq(lp) < 0.5*0.5) return false;

        auto head = planner_.plan(map_, cur, lp, true);
        if (head.size() < 2) return false;
        // Trim head: drop points already behind / duplicate of cur.
        // Attach tail: points on old path after the one nearest to head.back().
        Point endp = head.back();
        size_t tj = ci;
        double td2 = 1e18;
        for (size_t i = ci; i < path_.size(); ++i) {
            double d2 = path_[i].distSq(endp);
            if (d2 < td2) { td2 = d2; tj = i; }
        }
        std::vector<Point> merged;
        merged.reserve(head.size() + (path_.size() - tj));
        merged.insert(merged.end(), head.begin(), head.end());
        for (size_t i = tj; i < path_.size(); ++i) merged.push_back(path_[i]);
        if (merged.size() < 2) return false;

        path_ = std::move(merged);
        goal_ = path_.back();
        move_elapsed_ = 0.0;
        rebuildBspline(DRONE_SPEED);
        return true;
    }

    // ================================================================
    //  Target selection
    // ================================================================

    /// True if a world point is inside the exploration sub-region.
    /// Exploration targets & free-space candidates are ONLY taken from here;
    /// the landing corridor east of the internal wall (x>7.5) is outside it.
    /// A small EDGE_MARGIN keeps path endpoints off the wall / region edge.
    static bool inExplore(const Point& wp) {
        return wp.x >= EXPLORE_X_MIN + EDGE_MARGIN &&
               wp.x <= EXPLORE_X_MAX - EDGE_MARGIN &&
               wp.y >= EXPLORE_Y_MIN + EDGE_MARGIN &&
               wp.y <= EXPLORE_Y_MAX - EDGE_MARGIN;
    }

    /// True if a cell lies within INFLATE_RADIUS of a marked obstacle —
    /// such cells are inside the inflated corridor and are not safe targets.
    bool nearObstacle(int gx, int gy) const { return nearObstacle(map_, gx, gy); }
    /// ★带地图版本: 后台线程用快照调用(不再读成员 map_)
    static bool nearObstacle(const GridMap& m, int gx, int gy) {
        for (int dy = -INFLATE_RADIUS; dy <= INFLATE_RADIUS; ++dy)
            for (int dx = -INFLATE_RADIUS; dx <= INFLATE_RADIUS; ++dx)
                if (m.get(gx + dx, gy + dy) == CS_OCCUPIED) return true;
        return false;
    }

    /// Nearest CS_FREE cell, preferring the forward-180° sector (dot>=0) of the
    /// given heading so the drone never picks a target right behind it. Rear
    /// cells are only accepted when nothing lies ahead, and near-180° reversals
    /// (dot < -0.3) are always rejected. Cells hugging obstacles are excluded.
    /// 探索区内是否还剩可当目标的 CS_FREE(不受机头方向限制, 只排除越界/贴障)。
    bool anyFreeTargetLeft() const {
        for (int gy = 0; gy < GRID_N; ++gy)
            for (int gx = 0; gx < GRID_N; ++gx) {
                if (map_.get(gx, gy) != CS_FREE) continue;
                Point wp = gridToWorld(gx, gy);
                if (!inExplore(wp)) continue;
                if (nearObstacle(gx, gy)) continue;
                return true;
            }
        return false;
    }

    Point findNearestUnvisited(const Point& pos, double heading) const {
        return findNearestUnvisited(map_, pos, heading);
    }
    /// ★带地图版本(后台线程用快照)
    static Point findNearestUnvisited(const GridMap& m, const Point& pos, double heading) {
        Point best_fwd(0, 0), best_rear(0, 0);
        double bd_fwd = 1e18, bd_rear = 1e18;
        const double ch = std::cos(heading), sh = std::sin(heading);
        for (int gy = 0; gy < GRID_N; ++gy)
            for (int gx = 0; gx < GRID_N; ++gx) {
                if (m.get(gx, gy) != CS_FREE) continue;
                Point wp = gridToWorld(gx, gy);
                if (!inExplore(wp)) continue;
                if (nearObstacle(m, gx, gy)) continue;
                double d2 = pos.distSq(wp);
                double dx = wp.x - pos.x, dy = wp.y - pos.y;
                double l = std::sqrt(dx * dx + dy * dy);
                if (l < 1e-6) continue;
                double dot = (dx * ch + dy * sh) / l;
                if (dot >= 0.0) {
                    if (d2 < bd_fwd) { bd_fwd = d2; best_fwd = wp; }
                } else if (dot >= -0.3) {
                    if (d2 < bd_rear) { bd_rear = d2; best_rear = wp; }
                }
            }
        if (bd_fwd < 1e18) return best_fwd;    // prefer forward target
        if (bd_rear < 1e18) return best_rear;  // fall back to near-rear only
        // “不允许找身后~180°”太严可能导致角上找不到 → 全向兜底最近可去点
        Point best_any(0, 0); double bd_any = 1e18;
        for (int gy = 0; gy < GRID_N; ++gy)
            for (int gx = 0; gx < GRID_N; ++gx) {
                if (m.get(gx, gy) != CS_FREE) continue;
                Point wp = gridToWorld(gx, gy);
                if (!inExplore(wp)) continue;
                if (nearObstacle(m, gx, gy)) continue;
                double d2 = pos.distSq(wp);
                if (d2 < bd_any) { bd_any = d2; best_any = wp; }
            }
        return best_any;                        // may still be (0,0)
    }

    /// Up to N nearest CS_FREE cells in front of heading (forward-180° first),
    /// then rear cells (excluding the near-180° dead zone) only to fill the
    /// batch. Used for path extension so the next segment keeps moving forward
    /// instead of doing a 180° U-turn.
    std::vector<Point> findNearestUnvisitedN(const Point& pos, double heading,
                                             int n = 10) const {
        return findNearestUnvisitedN(map_, pos, heading, n);
    }
    /// ★带地图版本(后台线程用快照)
    static std::vector<Point> findNearestUnvisitedN(const GridMap& m, const Point& pos,
                                                    double heading, int n = 10) {
        std::vector<std::pair<double, Point>> fwd, rear;
        const double ch = std::cos(heading), sh = std::sin(heading);
        for (int gy = 0; gy < GRID_N; ++gy)
            for (int gx = 0; gx < GRID_N; ++gx) {
                if (m.get(gx, gy) != CS_FREE) continue;
                Point wp = gridToWorld(gx, gy);
                if (!inExplore(wp)) continue;
                if (nearObstacle(m, gx, gy)) continue;
                double d2 = pos.distSq(wp);
                if (d2 < 0.01) continue;  // skip the query point itself
                double dx = wp.x - pos.x, dy = wp.y - pos.y;
                double l = std::sqrt(dx * dx + dy * dy);
                if (l < 1e-6) continue;
                double dot = (dx * ch + dy * sh) / l;
                if (dot >= 0.0) fwd.push_back({d2, wp});
                else if (dot >= -0.3) rear.push_back({d2, wp});  // reject ~180° U-turn
            }
        auto by_dist = [](const auto& a, const auto& b) { return a.first < b.first; };
        std::sort(fwd.begin(), fwd.end(), by_dist);
        std::sort(rear.begin(), rear.end(), by_dist);
        std::vector<Point> result;
        for (auto& c : fwd)
            if ((int)result.size() < n) result.push_back(c.second);
        for (auto& c : rear)
            if ((int)result.size() < n) result.push_back(c.second);
        return result;
    }

    /// Point-to-segment squared distance.
    static double pointToSegmentDist2(const Point& p, const Point& a, const Point& b) {
        double abx = b.x - a.x, aby = b.y - a.y;
        double apx = p.x - a.x, apy = p.y - a.y;
        double denom = abx * abx + aby * aby + 1e-12;
        double t = (apx * abx + apy * aby) / denom;
        if (t <= 0.0) return apx * apx + apy * apy;
        if (t >= 1.0) return (p.x - b.x) * (p.x - b.x) + (p.y - b.y) * (p.y - b.y);
        double projx = a.x + t * abx, projy = a.y + t * aby;
        return (p.x - projx) * (p.x - projx) + (p.y - projy) * (p.y - projy);
    }

    /// Check if new_seg loops back and comes too close to the existing path.
    bool pathSelfIntersects(const std::vector<Point>& existing,
                            const std::vector<Point>& new_seg,
                            double clearance_m = 0.8) const {
        return pathSelfIntersectsS(existing, new_seg, clearance_m);
    }
    static bool pathSelfIntersectsS(const std::vector<Point>& existing,
                                    const std::vector<Point>& new_seg,
                                    double clearance_m = 0.8) {
        if (existing.size() < 5 || new_seg.size() < 2) return false;
        double c2 = clearance_m * clearance_m;
        // Exclude last 5 points — they connect to new_seg
        size_t check_end = (existing.size() > 5) ? existing.size() - 5 : 0;
        for (size_t i = 1; i < new_seg.size(); ++i) {
            for (size_t j = 0; j + 1 < check_end; ++j) {
                if (pointToSegmentDist2(new_seg[i], existing[j], existing[j+1]) < c2)
                    return true;
            }
        }
        return false;
    }

    /// Smoothed direction at path end (weighted average of last ~6 segments).
    Point pathEndDirection() const { return pathEndDirection(path_); }
    static Point pathEndDirection(const std::vector<Point>& path_) {
        if (path_.size() < 2) return Point(1, 0);
        size_t n = std::min(path_.size() - 1, (size_t)6);
        Point dir(0, 0);
        double total_w = 0.0;
        for (size_t i = path_.size() - n; i < path_.size() - 1; ++i) {
            double w = 1.0 + (double)(i - (path_.size() - n)); // ramp up toward end
            dir.x += w * (path_[i+1].x - path_[i].x);
            dir.y += w * (path_[i+1].y - path_[i].y);
            total_w += w;
        }
        double len = std::sqrt(dir.x*dir.x + dir.y*dir.y);
        if (len < 1e-6) return Point(1, 0);
        return Point(dir.x / len, dir.y / len);
    }

    /// Validate a path extension: reject if it detours excessively,
    /// goes against the path direction, or self-intersects.
    bool isValidExtension(const std::vector<Point>& new_seg, const Point& target,
                          double max_detour_ratio = 3.0) const {
        return isValidExtension(new_seg, path_.back(), path_, target, max_detour_ratio);
    }
    /// ★带参考点+旧路径版本(后台线程不再读成员 path_/map_)
    static bool isValidExtension(const std::vector<Point>& new_seg, const Point& ref,
                                 const std::vector<Point>& old_path,
                                 const Point& target, double max_detour_ratio = 3.0) {
        double euclidean = ref.dist(target);

        // 1) Detour-ratio: path length vs straight-line distance
        if (euclidean > 0.3) {
            double plen = 0.0;
            for (size_t i = 1; i < new_seg.size(); ++i)
                plen += new_seg[i].dist(new_seg[i-1]);
            if (plen > max_detour_ratio * euclidean + 0.5) {
                printf("[EXT] Reject: detour %.2fm / straight %.2fm = %.1fx\n",
                       plen, euclidean, plen / euclidean);
                return false;
            }
        }

        // 2) Forward-direction: don't go backwards relative to path heading
        if (euclidean > 1.0) {
            Point dir = pathEndDirection(old_path);
            Point to_t(target.x - ref.x, target.y - ref.y);
            double tl = std::sqrt(to_t.x*to_t.x + to_t.y*to_t.y);
            to_t.x /= tl; to_t.y /= tl;
            double dot = dir.x * to_t.x + dir.y * to_t.y;
            if (dot < -0.15) {
                printf("[EXT] Reject: target behind path direction (dot=%.2f)\n", dot);
                return false;
            }
        }

        // 3) Self-intersection: new segment must not cross or hug old path
        if (pathSelfIntersectsS(old_path, new_seg)) {
            printf("[EXT] Reject: self-intersects with existing path\n");
            return false;
        }

        return true;
    }

    // ================================================================
    //  Position tracking  (EGOv2 style: time-driven B-spline evaluation)
    // ================================================================
    /// Evaluate the optimised B-spline at t = move_elapsed_.
    /// pos = bspline_.pos(t)  — where the drone SHOULD be
    /// dir = bspline_.vel(t)  — trajectory tangent (smooth, continuous)
    Cmd posTrack(double cruise_speed) {
        Cmd cmd;
        if (path_.size() < 2) return cmd;
        if (bspline_.numCtrl() < 4) return cmd;

        double total_t = bspline_.duration();
        if (total_t < 0.01) return cmd;

        double t = move_elapsed_;
        if (t > total_t) t = total_t;

        // Constant position lead (in metres, not seconds — scales with speed)
        double lead_m = 0.25;
        double t_lead = lead_m / cruise_speed;
        double t_pos = std::min(t + t_lead, total_t);
        Point pos_target = bspline_.pos(t_pos);
        // ★末端钉在精确终点: 均匀B样条在端点只是"逼近"控制点, pos(total_t) 会比
        //   path_.back() 内缩(实测 0.2m 量级) → 到达判定(距 path_.back()<10cm) 永远
        //   不满足 → 飞机到位后停住原地悬停(返航最后一段最明显)。
        //   所以最后一段把设定点平滑地收到 path_.back() 上。
        {
            const double rem_t = total_t - t_pos;
            const double fade_t = std::max(0.25, 0.6 / std::max(cruise_speed, 0.05));
            if (rem_t < fade_t && !path_.empty()) {
                const double a = std::clamp(1.0 - rem_t / fade_t, 0.0, 1.0);
                const Point& e = path_.back();
                pos_target.x = pos_target.x + a * (e.x - pos_target.x);
                pos_target.y = pos_target.y + a * (e.y - pos_target.y);
            }
        }

        // ★设定点限幅: 距机位不得超过 LEAD_MAX(0.5m)。悬停等延长/重规划、或换新路径后
        //   move_elapsed_ 可能已经涨过头 → 设定点一下跳到机位前方很远 → PX4 位置环把
        //   位置误差当速度猛追 → 实际速度超过 max_speed(实测 0.8 限速飞 >1m/s)。
        //   正常巡航前视只有 0.25m, 不受影响。
        {
            constexpr double LEAD_MAX = 0.30;   // ★只允许超前 0.3m: PX4 位置环的"速度=Kp*误差"
                                                 //   叠加在速度前馈上, 误差越大总速度越超 max_speed
            const double ex = pos_target.x - drone_.x, ey = pos_target.y - drone_.y;
            const double err = std::sqrt(ex*ex + ey*ey);
            if (err > LEAD_MAX) {
                pos_target.x = drone_.x + ex * (LEAD_MAX / err);
                pos_target.y = drone_.y + ey * (LEAD_MAX / err);
            }
        }

        // Velocity feedforward comes from the B-spline derivative, but the
        // optimiser / knot lower-bounds can push the speed ABOVE cruise_speed.
        // Clamp the feedforward magnitude to cruise_speed so the commanded
        // speed never exceeds the configured cruise value.
        Point raw_vel = bspline_.vel(t);
        double sp = std::sqrt(raw_vel.x*raw_vel.x + raw_vel.y*raw_vel.y);
        double ux, uy;
        if (sp > 0.05) { ux = raw_vel.x / sp; uy = raw_vel.y / sp; }
        else {
            double dx = path_[1].x - path_[0].x;
            double dy = path_[1].y - path_[0].y;
            double dl = std::sqrt(dx*dx + dy*dy);
            ux = (dl > 0.01) ? dx / dl : 1.0;
            uy = (dl > 0.01) ? dy / dl : 0.0;
        }
        double sp_cmd = std::min(sp, cruise_speed);

        // ★★ 加速度前馈(惯性补偿): 对 B 样条速度做数值微分 ✓
        //    原来 type_mask 屏蔽了 AFX/AFY → 只有速度前馈(匀速够用 ✓ 但纠偏/扰动时
        //    位置环跟不上 → 通道里摆 ±0.3m → 蹭墙 ✓)。这里把加速度补上 ✓
        {
            const double hA = 0.002;                       // 2ms 中心差分
            double tA1 = t + hA, tA0 = t - hA;
            if (tA1 > bspline_.duration()) tA1 = bspline_.duration();
            if (tA0 < 0.0)                 tA0 = 0.0;
            if (tA1 > tA0 + 1e-6) {
                Point v1 = bspline_.vel(tA1), v0 = bspline_.vel(tA0);
                double ax = (v1.x - v0.x) / (tA1 - tA0);
                double ay = (v1.y - v0.y) / (tA1 - tA0);
                const double AM = MAX_ACCEL;                // 限幅(不超过配置的最大加速度)
                if (ax >  AM) ax =  AM;  if (ax < -AM) ax = -AM;
                if (ay >  AM) ay =  AM;  if (ay < -AM) ay = -AM;
                cmd.acc_x = ax; cmd.acc_y = ay;
            }
        }

        cmd.pos_x = pos_target.x;
        cmd.pos_y = pos_target.y;
        cmd.vel_x = ux * sp_cmd;
        cmd.vel_y = uy * sp_cmd;
        // ★★ 过【探索区角点】之后(以及 A→B 走廊全程)机头固定朝 -y:
        //    原来这里跟【飞行方向】走 → 角点后飞 padA 那一小段方向是斜的 → 机头突然摆一下 ✗
        //    (到了 padA 进 stage1 才被强制 -y, 所以看起来"摆一下又正常" ✓)
        if (home_corner_done_ || return_stage_ >= 1)
            cmd.yaw = -M_PI / 2.0;
        else
            cmd.yaw = std::atan2(uy, ux);
        yaw_last_ = cmd.yaw;      // ★记住指令yaw, 悬停时重复它(不再跟随测量值)
        cmd.vel_z = 0;
        cmd.valid = true;

        drone_.speed = sp_cmd;
        drone_.moving = true;
        return cmd;
    }

    /// A→B 折线跟踪(按弧长推进的设定点 + 切向速度前馈), 不用 B 样条:
    /// path_ = A* 规划出的走廊折线(5cm 加密)。
    ///
    /// 为什么不用"机位前方 lookahead 当设定点": 那样速度前馈一旦清零, 实际速度
    /// 只剩 PX4 位置环的 P 项 v ≈ P * lookahead(实测 P*0.18 ≈ 0.17 m/s),
    /// channel_speed 参数完全不生效。这里改为维护沿路径的弧长设定点 st1_s_:
    ///   st1_s_ += speed*dt        (设定点自己按参数速度前进)
    ///   并夹在 [机位投影-0.05, 机位投影+0.80] 内(落后就吸附, 防重规划后跳变)
    /// 设定点 = 路径上 st1_s_ 处, 速度前馈 = 该处切向 * speed
    ///   → 稳态下位置误差≈0(前馈速度=设定点前进速度), 实际速度就是 speed,
    ///     位置环只负责修横向偏差; 且设定点始终在折线上 → 不抄近道、不切内角。
    /// 末端 0.3m 内前馈线性衰减到 0, 让飞机能停在终点(10cm 到达判定)。
    Cmd polylineTrack(double speed, double dt) {
        Cmd cmd;
        if (path_.size() < 2) return cmd;
        // 累积弧长(点数很少, 每拍重算; 换路径后自动同步)
        std::vector<double> arc(path_.size(), 0.0);
        for (size_t i = 1; i < path_.size(); ++i)
            arc[i] = arc[i - 1] + path_[i].dist(path_[i - 1]);
        const double total = arc.back();
        // ★机位在路径上的投影弧长 —— 必须投影到【线段】上, 不能用"最近点":
        //   mode1 的"先x再y"折线只有 2~6 个点, 最近点投影会一直落在端点上,
        //   设定点被下面的 "s_d + 0.80" 限幅卡住 → 飞机走 0.8m 就不动了。
        double s_d = 0.0; double bd2 = 1e18;
        for (size_t i = 0; i + 1 < path_.size(); ++i) {
            const Point& a = path_[i]; const Point& b = path_[i+1];
            double vx = b.x - a.x, vy = b.y - a.y;
            double L2 = vx*vx + vy*vy;
            double tt = (L2 > 1e-12) ? ((drone_.x-a.x)*vx + (drone_.y-a.y)*vy) / L2 : 0.0;
            tt = std::clamp(tt, 0.0, 1.0);
            double px = a.x + tt*vx, py = a.y + tt*vy;
            double d2 = (px-drone_.x)*(px-drone_.x) + (py-drone_.y)*(py-drone_.y);
            if (d2 < bd2) { bd2 = d2; s_d = arc[i] + tt * std::sqrt(L2); }
        }
        // 设定点前进 + 限幅
        st1_s_ += speed * std::max(dt, 1e-3);
        st1_s_ = std::max(st1_s_, s_d - 0.05);          // 落后于机位 → 吸附回来
        st1_s_ = std::min(st1_s_, s_d + 0.80);          // 别超前太多(防起步冲出去)
        st1_s_ = std::clamp(st1_s_, 0.0, total);
        // 取 st1_s_ 处的点 + 切向
        size_t k = 0;
        while (k + 1 < path_.size() && arc[k + 1] < st1_s_) ++k;
        double seg = (k + 1 < path_.size()) ? (arc[k + 1] - arc[k]) : 0.0;
        double f   = (seg > 1e-9) ? (st1_s_ - arc[k]) / seg : 0.0;
        Point pt, tan;
        if (k + 1 < path_.size()) {
            Point a = path_[k], b = path_[k + 1];
            pt  = Point(a.x + f * (b.x - a.x), a.y + f * (b.y - a.y));
            double len = std::max(1e-9, a.dist(b));
            tan = Point((b.x - a.x) / len, (b.y - a.y) / len);
        } else {
            pt  = path_.back();
            Point a = path_[path_.size() - 2], b = path_.back();
            double len = std::max(1e-9, a.dist(b));
            tan = Point((b.x - a.x) / len, (b.y - a.y) / len);
        }
        // 末端收前馈: 最后 0.3m 内线性衰减到 0(否则会一直推着飞机越过终点)
        double ff = speed;
        const double END_FADE = 0.30;
        if (total - st1_s_ < END_FADE) ff *= std::max(0.0, (total - st1_s_) / END_FADE);
        ff *= st1_vel_ff_;
        cmd.pos_x = pt.x;
        cmd.pos_y = pt.y;
        cmd.vel_x = tan.x * ff;
        cmd.vel_y = tan.y * ff;
        cmd.yaw   = -M_PI / 2.0;
        cmd.vel_z = 0;
        cmd.valid = true;
        return cmd;
    }

    /// Build an optimised B-spline from the current discrete path_.
    /// @param cruise_speed  desired cruise speed for this trajectory (m/s)
    void rebuildBspline(double cruise_speed) {
        if (path_.size() < 2) return;
        // ★★ 现场要求: 【返航去探索区角点的过程中, 中间路径点必须留在探索区内】✓✓
        //    只夹中间点(1..n-2) ✓ 首点(当前位置)和末点(你给的角点)不夹 ✓
        //    为什么: 中间点若跑到探索区外(如 y→3.95) ✗ → B样条还会往外鼓(实测指令 3.99 ✗)
        //            → 飞机跟着飞出地图(4.255 ✗) ✓
        //    生效条件: 返航 GO_HOME 且【还没过角点】(= stage0 去角点那一段) ✓
        //             通道段(padA/padB)不夹 ✓ 它们本来就在探索区外 ✓
        if (state_ == GO_HOME && !home_corner_done_ && path_.size() >= 3) {
            const double ex0 = EXPLORE_X_MIN + 0.2, ex1 = EXPLORE_X_MAX - 0.2;
            const double ey0 = EXPLORE_Y_MIN + 0.2, ey1 = EXPLORE_Y_MAX - 0.2;
            for (size_t i = 1; i + 1 < path_.size(); ++i) {
                if (path_[i].x < ex0) path_[i].x = ex0;
                if (path_[i].x > ex1) path_[i].x = ex1;
                if (path_[i].y < ey0) path_[i].y = ey0;
                if (path_[i].y > ey1) path_[i].y = ey1;
            }
        }
        bspline_ = UniformBspline2D::fromPath(path_, cruise_speed);
        BsplineOptimizer2D::Params bsp;
        bsp.max_vel   = cruise_speed;
        bsp.max_acc   = MAX_ACCEL;
        bsp.safe_dist = (INFLATE_RADIUS + 4) * GRID_RES;
        BsplineOptimizer2D::optimize(map_, bspline_, bsp);
        bsp_cruise_ = cruise_speed;   // 记录建速, A→B retime 的基准
    }

    /// A→B 专用: 不重跑 EGO 优化器 —— 其安全距离(~0.8m)大于窄口半宽
    /// (~0.37m), 在口子里做不到会不断把居中的 B 样条顶回/挤到一侧, 造成
    /// "规划在中间、实际轨迹擦边"。路径已由 A* 无碰撞 + centerNarrowGates
    /// 居中保证, 直接均匀时间参数化即可。
    void rebuildBsplineSimple(double cruise_speed) {
        if (path_.size() < 2) return;
        bspline_ = UniformBspline2D::fromPath(path_, cruise_speed);
        bsp_cruise_ = cruise_speed;
    }

    Point findNearestUnknown(const Point& pos) const {
        Point best(0, 0);
        double best_dist = 1e9;
        bool found = false;
        for (int gy = 0; gy < GRID_N; ++gy) {
            for (int gx = 0; gx < GRID_N; ++gx) {
                if (map_.get(gx, gy) != CS_UNKNOWN) continue;
                Point wp = gridToWorld(gx, gy);
                if (wp.x < MAP_X_MIN + EDGE_MARGIN || wp.x > MAP_X_MAX - EDGE_MARGIN ||
                    wp.y < MAP_Y_MIN + EDGE_MARGIN || wp.y > MAP_Y_MAX - EDGE_MARGIN) continue;
                double dist = pos.dist(wp);
                if (dist > 5.0 || dist >= best_dist) continue;
                if (lineOfSightClear(pos, wp)) {
                    best_dist = dist;
                    best = wp;
                    found = true;
                }
            }
        }
        if (!found) {
            for (int gy = 0; gy < GRID_N; gy += 2) {
                for (int gx = 0; gx < GRID_N; gx += 2) {
                    if (map_.get(gx, gy) != CS_UNKNOWN) continue;
                    Point wp = gridToWorld(gx, gy);
                    if (wp.x < MAP_X_MIN + 0.2 || wp.x > MAP_X_MAX - 0.2 ||
                        wp.y < MAP_Y_MIN + 0.2 || wp.y > MAP_Y_MAX - 0.2) continue;
                    double dist = pos.dist(wp);
                    if (dist < best_dist) {
                        best_dist = dist;
                        best = wp;
                        found = true;
                    }
                }
            }
        }
        return found ? best : Point(MAP_X_MIN - 1, MAP_Y_MIN - 1);
    }

    bool lineOfSightClear(const Point& a, const Point& b) const {
        double dx = b.x - a.x, dy = b.y - a.y;
        double dist = std::sqrt(dx*dx + dy*dy);
        int steps = std::max(4, (int)(dist / (GRID_RES * 0.5)));
        for (int i = 1; i < steps; ++i) {
            double t = (double)i / steps;
            GridPos gp = worldToGrid(a.x + t*dx, a.y + t*dy);
            if (map_.get(gp) == CS_OCCUPIED) return false;
        }
        return true;
    }

    Point findLookAheadFrom(const std::vector<Point>& path, size_t start_i,
                            double lookahead) const {
        if (path.empty()) return Point(drone_.x, drone_.y);
        if (path.size() == 1 || start_i >= path.size()) return path.back();
        double accum = 0.0;
        for (size_t i = start_i; i + 1 < path.size(); ++i) {
            double seg = path[i].dist(path[i+1]);
            if (accum + seg >= lookahead) {
                double f = (lookahead - accum) / seg;
                return Point(path[i].x + f*(path[i+1].x-path[i].x),
                             path[i].y + f*(path[i+1].y-path[i].y));
            }
            accum += seg;
        }
        return path.back();
    }

    bool nearEndOfPath() const {
        if (path_.empty()) return true;
        double d2 = (drone_.x-path_.back().x)*(drone_.x-path_.back().x)
                  + (drone_.y-path_.back().y)*(drone_.y-path_.back().y);
        return d2 < waypoint_tol_ * waypoint_tol_;
    }

    static double pathLength(const std::vector<Point>& path) {
        double len = 0.0;
        for (size_t i = 1; i < path.size(); ++i)
            len += path[i].dist(path[i-1]);
        return len;
    }

    bool newObstacleOnPath() {
        size_t start_i = 0;
        double best_d2 = std::numeric_limits<double>::max();
        for (size_t i = 0; i+1 < path_.size(); ++i) {
            double d2 = (path_[i].x-drone_.x)*(path_[i].x-drone_.x)
                      + (path_[i].y-drone_.y)*(path_[i].y-drone_.y);
            if (d2 < best_d2) { best_d2 = d2; start_i = i; }
        }
        for (size_t i = start_i; i+1 < path_.size(); ++i) {
            double dx = path_[i+1].x - path_[i].x;
            double dy = path_[i+1].y - path_[i].y;
            double len = std::sqrt(dx*dx+dy*dy);
            int steps = std::max(2, (int)(len/(GRID_RES*0.5)));
            for (int s = 0; s <= steps; ++s) {
                double t = (double)s/steps;
                if (map_.get(worldToGrid(path_[i].x+t*dx, path_[i].y+t*dy)) == CS_OCCUPIED)
                    return true;
            }
        }
        return false;
    }

    GridMap          map_;
    FrontierDetector fd_;
    PathPlanner      planner_;
    std::vector<Point> path_, viewpoints_;
    Point goal_;

    UniformBspline2D bspline_;

    DroneState drone_;
    double last_yaw_ = 0.0, last_yawdot_ = 0.0;
    double sense_timer_ = 0.0, stuck_timer_ = 0.0;
    double obstacle_check_timer_ = 0.0;  // throttles the obstacle re-plan check
    double replan_timer_ = 0.0;          // EGO-style periodic re-plan throttle
    double move_start_x_ = 0.0, move_start_y_ = 0.0;
    int fail_count_ = 0;
    int consecutive_fail_ = 0;
    bool recovery_used_ = false;
    double scan_turn_accum_ = 0.0;
    double scan_dir_ = 1.0;
    int    scan_settle_ = 0;
    int    scan_no_target_ = 0;   // DETECT 连续找不到目标次数(旋转重扫用)
    State state_ = INIT;
    bool home_planned_ = false;
    bool home_yaw_align_ = false;  // GO_HOME: align yaw before flying at high speed
    bool yaw_align_phase_ = false;
    // A→B: 到达 padA 后先 yaw 对准 -y(前180°点云模式下必须先对准, 点云才
    // 覆盖走廊方向), 对准完成后才检测窄墙中点并规划路径。
    bool st1_await_align_ = false;   // 正在等对准完成
    bool st1_aligned_     = false;   // 已完成对准(之后才允许用点云检测口子)
    // Return-to-landing leg state: return_stage_ 0 = pad A, 1 = pad B (mode 1).
    // return_cruise_ = cruise speed of the currently-flying return leg.
    int    return_stage_   = 0;
    double return_cruise_  = 2.0;
    bool   home_corner_done_ = false; // mode1: 是否已先到探索区角点(整定/刹车点)
    double home_corner_x_ = -999.0;   // ★左上角点(可 launch 指定; -999=自动)
    double home_corner_y_ = -999.0;
    double bsp_cruise_  = 0.0;   // 当前 B 样条时间参数化建速(A→B 变速用)
    double leg_speed_   = 0.0;   // A→B 当前巡航(开阔 0.4 / 窄口自动降)
    double st1_last_y_  = 1e9;   // A→B 上次重建路径时的 y(推进式重扫去抖)
    Point  st1_leg_goal_ = Point(0, 0);  // A→B 当前段目标(窄口中点或padB)
    bool   st1_goal_is_gate_ = false;    // A→B 目标是否为"mode2 第一个口-0.5m"(false=padB)
    // ===== 异步规划成员(见类顶部 struct BgPlan) =====
    std::future<BgPlan> bg_fut_;               // 后台规划任务
    int    bg_kind_ = 0;                       // 0=无 1=MOVE头部重规划 2=到目标点(视觉)
    Point  bg_pt_goal_{0, 0};                  // 视觉目标点
    int    bg_pt_result_ = 0;                  // -1=失败 0=在算/未提交 1=成功
    double ext_wait_ = 0.0;                     // 到末端等后台延长的累计时间(s)
    // ★通道路径方式(launch ab_path_mode): 0=保守A*(斜线直接穿口中点) 1=先x再y轴对齐(过口+overshoot)
    int    ab_path_mode_ = 0;
    double ab_axis_overshoot_ = 0.7;
    double ab_axis_lead_      = 1.2;   // ★提前并线距离(m): 离口子这么远就开始斜着并到中线 ✓
    double ab_axis_x_speed_  = 0.25;   // mode1: 横移(x)段限速(m/s), 否则拐回-y刹不住
    std::vector<Point> ab_axis_gates_;          // 提交后台任务时带上的口中点
    std::vector<Point> bg_gh_path_;             // 后台结果槽(返航 leg/走廊/新目标)
    Point  bg_gh_goal_{0, 0};
    bool   bg_gh_done_ = false;                 // 结果已就绪待消费
    bool   bg_gh_ok_   = false;                 // 结果里是否有可用路径
    int    bg_gh_hold_log_ = 0;                 // 返航等规划日志节流
    int    st1_latcap_log_ = 0;                 // mode1 横移限速日志节流
    int    diag_pick_ = 0, diag_pick_ok_ = 0, diag_pick_fail_ = 0, diag_nocand_ = 0;  // 选点诊断节流
    double yaw_last_ = 0.0;             // ★上一次"指令yaw"(悬停时重复它, 不跟随测量yaw)
    bool   yaw_last_init_ = false;
    bool   bg_vis_want_ = false;        // ★视觉在等规划(延长任务必须让路)
    bool   bg_vis_queued_ = false;      // ★视觉目标规划没提交成功, 等槽空了补投
public:
    /// ★规划模式: true=同步(默认, 提交即就地计算 —— 和"没有独立进程那版"行为完全一致,
    /// 结构上不存在"后台槽被占/排队/等结果悬停/规划超时放弃"); false=后台线程(控制拍不被
    /// 规划阻塞, 但需要处理排队/让路)。2Hz 重规划下同步模式每 500ms 阻塞一次规划耗时。
    /// ★路径沿途最小离障距离(m): 逐点在小窗口内找最近的 CS_OCCUPIED 格, 取全程最小,
    ///   上限 cap(超过 cap 视为"很安全", 不再区分)。用于选点时做安全惩罚。
    static double pathMinClearance(const GridMap& m, const std::vector<Point>& path, double cap = 0.8) {
        const int W = (int)std::ceil(cap / GRID_RES);   // 按 cap(m) 换算格数 ✓ (自动随分辨率)
        double best = cap;
        for (size_t i = 0; i < path.size(); ++i) {
            GridPos g = worldToGrid(path[i].x, path[i].y);
            if (!g.valid()) continue;
            for (int dy = -W; dy <= W; ++dy)
                for (int dx = -W; dx <= W; ++dx) {
                    GridPos q{g.x + dx, g.y + dy};
                    if (!q.valid()) continue;
                    if (m.get(q) != CS_OCCUPIED) continue;
                    Point c = gridToWorld(q.x, q.y);
                    const double d = std::hypot(c.x - path[i].x, c.y - path[i].y);
                    if (d < best) best = d;
                }
            if (best <= GRID_RES) break;
        }
        return best;
    }

    /// ★探索策略: 目标点 x >= v 的一律优先探索, x 更小的(起飞点附近)放最后。v<-100 关闭
    void setFarFirstX(double v) { far_first_x_ = v; }
    /// ★选点安全/绕远参数: 评估候选个数 + 期望离障余量(m) + 安全惩罚权重
    void setPickSafety(int topn, double safe_clear, double safe_w) {
        far_first_topn_ = std::max(1, topn);
        far_first_safe_clear_ = safe_clear;
        far_first_safe_w_ = safe_w;
    }

    void setSyncPlanning(bool s) { bg_sync_ = s;
        printf("[PLAN] 规划模式 = %s\n", s ? "同步(提交即计算)" : "后台线程"); }
    /// ★选目标(kind 6)是否也用同样的同步/异步模式
    void setSyncPlanningTarget(bool s) { bg_sync_target_ = s; }
    bool syncPlanning() const { return bg_sync_; }
private:
    double far_first_x_ = -1e9;      // ★x>=此值的目标优先探索(起飞点附近放最后)
    int    far_first_topn_ = 6;      // ★选点时实际评估的前 N 个候选(A*+离障)
    double far_first_safe_clear_ = 0.40;  // ★期望沿途离障余量(m), 低于它加权惩罚
    double far_first_safe_w_ = 0.0;       // ★安全惩罚权重(0=不用: 膨胀+机体半径已保证安全)
    bool   bg_sync_ = true;
    bool   bg_sync_target_ = true;      // 新目标选点(kind 6)也同步 = 默认完全不使用后台线程(与旧版一致)
                                        // 若现场出现选点耗时过长(控制拍卡顿), 可设 false 退回后台
    double ext_retry_t_ = 0.0;          // 延长任务节流计时(s)
    int    bg_busy_log_ = 0;            // 后台忙导致视觉排队日志节流
    bool   vision_active_ = false;              // 是否有视觉任务(节点每拍同步)
    long   vision_gen_    = 0;                  // 视觉任务代号(结束即+1)
    long   bg_pt_gen_     = 0;                  // 本次提交时的任务代号
    bool   hold_home_        = false;    // true = 视觉标靶没做完, 暂缓"进度达标自动返航"
    bool   hold_home_warned_ = false;    // 上面的提示只打一次
    double st1_s_            = 0.0;      // A→B 沿折线的弧长设定点(m), polylineTrack 维护
    double st1_vel_ff_       = 1.0;      // A→B 速度前馈比例(0..1.5, launch ab_vel_ff)
    bool   slow_active_ = false; // A→B 窄口慢态锁存
    int gh_dbg_ = 0;   // TEMP debug counters (remove after diagnosis)
    mutable double gh_gate_dbg_y_ = -1e9;   // 上次 [GATE] 打印的行 y(节流)
    int gh_dbg2_ = 0;
    int gh_plan_fail_ = 0;   // consecutive GO_HOME empty-plan retries
    double gh_near_t_ = 0.0;    // ★返航终点"已到位但没进10cm"的停稳计时(s)
    int    gh_near_log_ = 0;
    int    gh_direct_log_ = 0;   // "轨迹走完→位置环直飞目标点"日志节流
    double gh_near_tol_ = 0.80;  // 当前段"到位"容差(角点0.80/其它0.25)
    double move_elapsed_ = 0.0;  // time since current trajectory started (for time-based posTrack)
};
