#pragma once

#include "Types.hpp"
#include "GridMap.hpp"
#include "BsplineTrajectory2D.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

/// Simplified EGO-style B-spline trajectory optimiser for 2D.
///
/// Given an initial B-spline (e.g. from A* waypoints), runs gradient descent
/// to minimise a combined cost:
///   total = λ₁·f_smooth  +  λ₂·f_collision  +  λ₃·f_feasibility
///
/// All gradients are analytic (no finite-differencing).  Uses Armijo
/// backtracking line search for step-size selection.
///
/// Usage:
///   BsplineOptimizer2D::Params p;
///   BsplineOptimizer2D::optimize(map, bspline, p);
struct BsplineOptimizer2D {
    struct Params {
        double lambda_smooth    = 1.0;   // jerk minimisation weight
        double lambda_collision = 5.0;   // obstacle repulsion (stronger for 0.4m cube drone)
        double lambda_feas      = 1.0;   // vel / acc limit weight
        double safe_dist        = 0.4;   // m, desired clearance from obstacles (set by PLAN)
        double max_vel          = 0.7;   // m/s
        double max_acc          = 1.5;   // m/s²
        int    max_iters        = 500;   // more iterations for better convergence
        double init_step        = 0.04;
    };

    /// Run optimisation.  The B-spline is modified in-place.
    static int optimize(const GridMap& map, UniformBspline2D& bspline,
                        const Params& p)
    {
        auto& ctrl = bspline.ctrlPoints();
        const int N = static_cast<int>(ctrl.size());
        if (N < 6) return 0;  // need at least a few free points

        double dt = bspline.interval();
        const int SR = INFLATE_RADIUS + 5;  // wider search: 1.1m radius for 0.4m cube drone

        // ---- helpers ----

        auto nearestObs = [&](double wx, double wy) -> std::pair<double, Point> {
            GridPos gp = worldToGrid(wx, wy);
            double best_d = 1e9;
            Point grad_dir(0, 0);
            for (int dy = -SR; dy <= SR; ++dy) {
                for (int dx = -SR; dx <= SR; ++dx) {
                    if (dx == 0 && dy == 0) continue;
                    if (map.get(gp.x + dx, gp.y + dy) == CS_OCCUPIED) {
                        double d = std::sqrt(1.0*dx*dx + 1.0*dy*dy) * GRID_RES;
                        if (d < best_d) {
                            best_d = d;
                            grad_dir = Point(-dx * GRID_RES / std::max(d, 1e-6),
                                             -dy * GRID_RES / std::max(d, 1e-6));
                        }
                    }
                }
            }
            return {best_d, grad_dir};
        };

        // Compute all costs + gradients.  Fills grad array (N entries).
        auto computeAll = [&](std::vector<Point>& grad, double& total) {
            total = 0.0;
            for (int i = 0; i < N; ++i) grad[i] = Point(0, 0);

            // ---- (a) smoothness cost (jerk minimisation) ----
            // jerk_i = P_{i+3} - 3P_{i+2} + 3P_{i+1} - P_i   (for i = 0 .. N-4)
            double f_smooth = 0.0;
            for (int i = 0; i < N - 3; ++i) {
                double jx = ctrl[i+3].x - 3.0*ctrl[i+2].x + 3.0*ctrl[i+1].x - ctrl[i].x;
                double jy = ctrl[i+3].y - 3.0*ctrl[i+2].y + 3.0*ctrl[i+1].y - ctrl[i].y;
                f_smooth += jx*jx + jy*jy;
                double gx = 2.0 * jx, gy = 2.0 * jy;
                grad[i  ].x -= gx;   grad[i  ].y -= gy;
                grad[i+1].x += 3.0*gx; grad[i+1].y += 3.0*gy;
                grad[i+2].x -= 3.0*gx; grad[i+2].y -= 3.0*gy;
                grad[i+3].x += gx;     grad[i+3].y += gy;
            }
            total += p.lambda_smooth * f_smooth;

            // ---- (b) collision cost (quadratic penalty — stronger gradient at small errors) ----
            // Clearance applies fully INSIDE the exploration sub-region; in the
            // landing corridor (outside EXPLORE_*) keep only a small body margin
            // so the return trajectory can follow the narrow corridor.
            // ★按米: 原先 DRONE_RADIUS+3*GRID_RES 在 10cm 时=0.55m; 分辨率 2.5cm 后按格算
            //   会缩到 0.325m → 平滑路径贴墙。保持 0.55m 不变。
            const double body_demarc = (DRONE_RADIUS + 0.30);
            double f_coll = 0.0;
            for (int i = 1; i < N - 1; ++i) {
                double demarc = inExploreRegion(ctrl[i].x, ctrl[i].y)
                              ? p.safe_dist : body_demarc;
                auto [d, dir] = nearestObs(ctrl[i].x, ctrl[i].y);
                if (d < demarc && d > 1e-8) {
                    double err = demarc - d;
                    f_coll += err * err;           // quadratic: gradient = 2*err (stronger than cubic's 3*err²)
                    double g = -2.0 * err;          // pushes harder at shallow penetrations
                    grad[i].x += p.lambda_collision * g * dir.x;
                    grad[i].y += p.lambda_collision * g * dir.y;
                }
            }
            total += p.lambda_collision * f_coll;

            // ---- (c) feasibility cost (vel & acc limits) ----
            double f_feas = 0.0;
            double ts_inv  = 1.0 / dt;
            double ts_inv2 = ts_inv * ts_inv;

            // Velocity (i = 0 .. N-2):  vi = (P_{i+1} - P_i) / dt
            for (int i = 0; i < N - 1; ++i) {
                double vx = (ctrl[i+1].x - ctrl[i].x) * ts_inv;
                double vy = (ctrl[i+1].y - ctrl[i].y) * ts_inv;
                double vn2 = vx*vx + vy*vy;
                double vlim2 = p.max_vel * p.max_vel;
                if (vn2 > vlim2) {
                    double vn = std::sqrt(vn2);
                    double diff = vn - p.max_vel;
                    f_feas += diff * diff * ts_inv2;  // scale similar to acc
                    double g = 2.0 * diff / std::max(vn, 0.01) * ts_inv * ts_inv2;
                    double gx = g * vx, gy = g * vy;
                    grad[i  ].x -= gx; grad[i  ].y -= gy;
                    grad[i+1].x += gx; grad[i+1].y += gy;
                }
            }

            // Acceleration (i = 0 .. N-3):  ai = (P_{i+2} - 2P_{i+1} + P_i) / dt²
            for (int i = 0; i < N - 2; ++i) {
                double ax = (ctrl[i+2].x - 2.0*ctrl[i+1].x + ctrl[i].x) * ts_inv2;
                double ay = (ctrl[i+2].y - 2.0*ctrl[i+1].y + ctrl[i].y) * ts_inv2;
                double an2 = ax*ax + ay*ay;
                double alim2 = p.max_acc * p.max_acc;
                if (an2 > alim2) {
                    double an = std::sqrt(an2);
                    double diff = an - p.max_acc;
                    f_feas += diff * diff;
                    double g = 2.0 * diff / std::max(an, 0.01) * ts_inv2;
                    double gx = g * ax, gy = g * ay;
                    grad[i  ].x += gx;     grad[i  ].y += gy;
                    grad[i+1].x -= 2.0*gx; grad[i+1].y -= 2.0*gy;
                    grad[i+2].x += gx;     grad[i+2].y += gy;
                }
            }
            total += p.lambda_feas * f_feas;
        };

        // ---- gradient descent with Armijo line search ----
        double step = p.init_step;
        double cost_k = 0.0;
        {
            std::vector<Point> g(N);
            computeAll(g, cost_k);
        }

        for (int iter = 0; iter < p.max_iters; ++iter) {
            std::vector<Point> grad(N);
            double cost;
            computeAll(grad, cost);

            // Compute gradient norm² for Armijo condition
            double gnorm2 = 0.0;
            for (int i = 0; i < N; ++i)
                gnorm2 += grad[i].x*grad[i].x + grad[i].y*grad[i].y;
            if (gnorm2 < 1e-12) break;

            // Line search
            bool accepted = false;
            double step_try = step;
            std::vector<Point> next_ctrl(N);
            for (int ls = 0; ls < 12 && !accepted; ++ls) {
                for (int i = 1; i < N - 1; ++i) {
                    next_ctrl[i].x = ctrl[i].x - step_try * grad[i].x;
                    next_ctrl[i].y = ctrl[i].y - step_try * grad[i].y;
                    // Clamp to map bounds
                    next_ctrl[i].x = std::clamp(next_ctrl[i].x, MAP_X_MIN + 0.05, MAP_X_MAX - 0.05);
                    next_ctrl[i].y = std::clamp(next_ctrl[i].y, MAP_Y_MIN + 0.05, MAP_Y_MAX - 0.05);
                }
                next_ctrl[0] = ctrl[0];
                next_ctrl[N-1] = ctrl[N-1];

                // Swap in, evaluate, swap back
                std::swap(ctrl, next_ctrl);
                std::vector<Point> g_next(N);
                double cost_next;
                computeAll(g_next, cost_next);
                std::swap(ctrl, next_ctrl);  // restore

                if (cost_next <= cost - 1e-4 * step_try * gnorm2) {
                    ctrl = next_ctrl;  // accept
                    cost_k = cost_next;
                    step = std::min(step_try * 1.5, 0.2);
                    accepted = true;
                } else {
                    step_try *= 0.5;
                }
            }
            if (!accepted || cost_k < 0.001) break;
        }

        // Report
        std::vector<Point> g_final(N);
        double final_cost;
        computeAll(g_final, final_cost);
        return 0;  // iterations done
    }
};
