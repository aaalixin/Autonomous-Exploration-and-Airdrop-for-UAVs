#pragma once

#include "Types.hpp"
#include "GridMap.hpp"
#include "Px4ExplorationPlanner.hpp"
#include <SDL2/SDL.h>
#include <cmath>
#include <mutex>
#include <thread>
#include <atomic>
#include <sstream>
#include <iomanip>

static constexpr SDL_Color CLR_UNKNOWN   = { 45,  45,  45,  255};
static constexpr SDL_Color CLR_FREE      = {170, 215, 170,  255};  // LiDAR free (green)
static constexpr SDL_Color CLR_FOV_FREE  = {100, 220, 220,  255};  // camera explored (teal)
static constexpr SDL_Color CLR_INFLATED  = { 80, 180,  80,  255};  // inflated halo (green)
static constexpr SDL_Color CLR_OCCUPIED  = {140,  80,  40,  255};  // walls/obstacles (brown)
static constexpr SDL_Color CLR_FRONTIER  = {255,  80,  80,  255};
static constexpr SDL_Color CLR_DRONE     = { 30, 144, 255,  255};
static constexpr SDL_Color CLR_HEADING   = {255, 255,   0,  255};
static constexpr SDL_Color CLR_PATH      = {255, 255,   0,  255};
static constexpr SDL_Color CLR_VIEWPOINT = {255, 140,   0,  255};
static constexpr SDL_Color CLR_GOAL      = {255,  50,  50,  255};

class GuiRenderer {
public:
    GuiRenderer(double mx1, double mx2, double my1, double my2, int cp = 2)   // ★6→2: 12x8m场地 → 480x400(原来6=2880px 超屏 ✗)
        : x1_(mx1), x2_(mx2), y1_(my1), y2_(my2), cell_px_(cp)
    {
        x_range_ = x2_ - x1_; y_range_ = y2_ - y1_;
        win_x_ = (int)(x_range_ / GRID_RES) * cell_px_;
        win_y_ = (int)(y_range_ / GRID_RES) * cell_px_;
        if (win_x_ < 400) win_x_ = 400;
        if (win_y_ < 300) win_y_ = 300;
    }
    ~GuiRenderer() { stop(); }

    void setFovParams(double angle_rad, double range_m) { fov_angle_ = angle_rad; fov_range_ = range_m; }

    /// Re-anchor the drawn frame (equal shift keeps window size unchanged).
    void setBounds(double mx1, double mx2, double my1, double my2) {
        x1_ = mx1; x2_ = mx2; y1_ = my1; y2_ = my2;
        x_range_ = x2_ - x1_; y_range_ = y2_ - y1_;
    }

    void start(Px4ExplorationPlanner* p, double* dx, double* dy, double* dyaw, std::mutex* mtx) {
        planner_ = p; drone_x_ = dx; drone_y_ = dy; drone_yaw_ = dyaw; mtx_ = mtx;
        running_ = true;
        thread_ = std::thread(&GuiRenderer::loop, this);
    }
    void stop() { running_ = false; if (thread_.joinable()) thread_.join(); }

private:
    void slamToScreen(double wx, double wy, int& sx, int& sy) const {
        sx = (int)((y2_ - wy) / y_range_ * win_y_);
        sy = win_x_ - 1 - (int)((wx - x1_) / x_range_ * win_x_);
    }

    void loop() {
        SDL_Init(SDL_INIT_VIDEO);
        window_ = SDL_CreateWindow("PX4 Exploration",
            SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
            win_y_, win_x_, SDL_WINDOW_SHOWN);
        renderer_ = SDL_CreateRenderer(window_, -1,
            SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
        while (running_) {
            SDL_Event ev; while (SDL_PollEvent(&ev)) {
                if (ev.type == SDL_QUIT) running_ = false;
            }
            render(); SDL_Delay(33);
        }
        if (renderer_) SDL_DestroyRenderer(renderer_);
        if (window_)   SDL_DestroyWindow(window_);
        SDL_Quit();
    }

    void render() {
        SDL_SetRenderDrawColor(renderer_, 0,0,0,255); SDL_RenderClear(renderer_);
        std::lock_guard<std::mutex> lock(*mtx_);
        if (!planner_) return;

        const GridMap& map = planner_->map();
        const auto& ftrs = planner_->frontiers();
        const auto& path = planner_->path();
        const auto& vps  = planner_->viewpoints();
        double dx = *drone_x_, dy = *drone_y_, dyaw = *drone_yaw_;

        // Grid map — GUI window is anchored in the shared world frame
        // (map_*/EXPLORE_*/pads already carry the origin shift).
        int nx = (int)(x_range_ / GRID_RES), ny = (int)(y_range_ / GRID_RES);
        for (int gx = 0; gx < nx; ++gx) for (int gy = 0; gy < ny; ++gy) {
            double wx = x1_ + (gx+0.5)*GRID_RES, wy = y1_ + (gy+0.5)*GRID_RES;
            GridPos gp = worldToGrid(wx, wy);
            CellState s = map.get(gp);
            SDL_Color c = CLR_UNKNOWN;
            if (s == CS_FOV_FREE)
                // Green inflation halo is shown ONLY inside the exploration
                // sub-region (yellow box); corridor cells stay plain teal.
                c = (inExploreRegion(wx, wy) && isInflated(map, gp))
                    ? CLR_INFLATED : CLR_FOV_FREE;
            else if (s == CS_OCCUPIED) c = CLR_OCCUPIED;  // walls/obstacles solid brown
            // CS_FREE (LiDAR) not rendered — used internally for path planning only
            SDL_SetRenderDrawColor(renderer_, c.r,c.g,c.b,c.a);
            int sx, sy; slamToScreen(wx, wy, sx, sy);
            SDL_Rect r = {sx, sy, cell_px_, cell_px_}; SDL_RenderFillRect(renderer_, &r);
        }

        // Frontiers
        SDL_SetRenderDrawColor(renderer_, CLR_FRONTIER.r,CLR_FRONTIER.g,CLR_FRONTIER.b,CLR_FRONTIER.a);
        for (auto& fp : ftrs) {
            Point wp = gridToWorld(fp.x, fp.y);
            int sx, sy; slamToScreen(wp.x, wp.y, sx, sy);
            SDL_Rect r = {sx, sy, cell_px_, cell_px_}; SDL_RenderFillRect(renderer_, &r);
        }

        // Actual flown trajectory = B-spline (sampled).  This is the line the
        // drone follows, so the drone marker stays on it.  Drawn first (grey)
        // when present.
        {
            std::vector<Point> spline = planner_->sampledTrajectory(60);
            if (!spline.empty()) {
                SDL_SetRenderDrawColor(renderer_, 200, 200, 200, 255);
                for (size_t i = 0; i + 1 < spline.size(); ++i) {
                    int sx1, sy1, sx2, sy2;
                    slamToScreen(spline[i].x,  spline[i].y,  sx1, sy1);
                    slamToScreen(spline[i+1].x, spline[i+1].y, sx2, sy2);
                    SDL_RenderDrawLine(renderer_, sx1, sy1, sx2, sy2);
                }
            }
        }

        // Planned path polyline (A* reference), drawn on top in yellow.
        if (!path.empty()) {
            SDL_SetRenderDrawColor(renderer_, CLR_PATH.r,CLR_PATH.g,CLR_PATH.b,CLR_PATH.a);
            for (size_t i = 0; i+1 < path.size(); ++i) {
                int sx1,sy1,sx2,sy2;
                slamToScreen(path[i].x,path[i].y,sx1,sy1);
                slamToScreen(path[i+1].x,path[i+1].y,sx2,sy2);
                SDL_RenderDrawLine(renderer_, sx1,sy1,sx2,sy2);
            }
        }

        // Viewpoints
        for (auto& vp : vps) { int sx,sy; slamToScreen(vp.x,vp.y,sx,sy); fillCircle(sx,sy,3,CLR_VIEWPOINT); }

        // Exploration sub-region boundary (maps the EXPLORE_* rectangle,
        // already in the shared world frame).
        {
            int sxa,sya,sxb,syb;
            slamToScreen(EXPLORE_X_MIN, EXPLORE_Y_MIN, sxa,sya);
            slamToScreen(EXPLORE_X_MAX, EXPLORE_Y_MAX, sxb,syb);
            SDL_SetRenderDrawColor(renderer_, 255, 255, 0, 255);   // yellow dashed box
            // draw 4 edges (simple solid outline)
            int x0=std::min(sxa,sxb), x1=std::max(sxa,sxb);
            int y0=std::min(sya,syb), y1=std::max(sya,syb);
            SDL_RenderDrawLine(renderer_, x0,y0, x1,y0);
            SDL_RenderDrawLine(renderer_, x1,y0, x1,y1);
            SDL_RenderDrawLine(renderer_, x1,y1, x0,y1);
            SDL_RenderDrawLine(renderer_, x0,y1, x0,y0);
        }

        // Landing pads A (entry / mode-0 landing) and B (mode-1 landing).
        {
            Point a = planner_->padA(), b = planner_->padB();
            int xa,ya,xb,yb;
            slamToScreen(a.x, a.y, xa,ya);
            slamToScreen(b.x, b.y, xb,yb);
            fillCircle(xa,ya, std::max(3,cell_px_/2), CLR_GOAL);   // A = red X later
            fillCircle(xb,yb, std::max(3,cell_px_/2), CLR_VIEWPOINT); // B = orange
            // A: draw as cross, B: as diamond, to tell them apart
            SDL_SetRenderDrawColor(renderer_, CLR_GOAL.r,CLR_GOAL.g,CLR_GOAL.b,CLR_GOAL.a);
            SDL_RenderDrawLine(renderer_, xa-5,ya-5,xa+5,ya+5);
            SDL_RenderDrawLine(renderer_, xa-5,ya+5,xa+5,ya-5);
            SDL_SetRenderDrawColor(renderer_, CLR_VIEWPOINT.r,CLR_VIEWPOINT.g,CLR_VIEWPOINT.b,CLR_VIEWPOINT.a);
            SDL_RenderDrawLine(renderer_, xb,yb-6, xb+5,yb);
            SDL_RenderDrawLine(renderer_, xb+5,yb, xb,yb+6);
            SDL_RenderDrawLine(renderer_, xb,yb+6, xb-5,yb);
            SDL_RenderDrawLine(renderer_, xb-5,yb, xb,yb-6);
        }

        // Goal
        if (planner_->state() == Px4ExplorationPlanner::MOVE
            || planner_->state() == Px4ExplorationPlanner::GO_HOME) {
            Point g = planner_->currentGoal();
            int sx,sy; slamToScreen(g.x,g.y,sx,sy);
            SDL_SetRenderDrawColor(renderer_, CLR_GOAL.r,CLR_GOAL.g,CLR_GOAL.b,CLR_GOAL.a);
            SDL_RenderDrawLine(renderer_, sx-5,sy-5,sx+5,sy+5);
            SDL_RenderDrawLine(renderer_, sx-5,sy+5,sx+5,sy-5);
        }

        // Drone
        drawDrone(dx, dy, dyaw);

        // FOV sector overlay
        drawFovSector(dx, dy, dyaw);

        // Info
        std::ostringstream oss;
        oss << "Exp: " << std::fixed << std::setprecision(1) << planner_->exploredPct()
            << "% | Ftrs: " << planner_->frontiers().size()
            << " | mode:" << planner_->flightMode();
        SDL_SetWindowTitle(window_, oss.str().c_str());
        SDL_RenderPresent(renderer_);
    }

    void drawDrone(double dx, double dy, double h) {
        int cx,cy; slamToScreen(dx,dy,cx,cy);
        int r = std::max(4, cell_px_/2);
        fillCircle(cx,cy,r,CLR_DRONE);
        int ax = cx - (int)(r*2*std::sin(h)), ay = cy - (int)(r*2*std::cos(h));
        SDL_SetRenderDrawColor(renderer_, CLR_HEADING.r,CLR_HEADING.g,CLR_HEADING.b,CLR_HEADING.a);
        SDL_RenderDrawLine(renderer_, cx,cy,ax,ay);
    }

    void drawFovSector(double dx, double dy, double h) {
        int cx, cy; slamToScreen(dx, dy, cx, cy);
        double half = fov_angle_ / 2.0, rmax = fov_range_;
        const GridMap& map = planner_->map();
        const int N = 32;

        // Per-ray occlusion (mirrors markForwardFov): march each ray outward and
        // stop before the first CS_OCCUPIED cell, so the overlay never paints
        // through walls/obstacles.
        auto rayEnd = [&](double a, double& wx, double& wy) {
            const double ca = std::cos(a), sa = std::sin(a);
            double last_r = 0.0;
            for (double rr = GRID_RES; rr <= rmax + 1e-6; rr += GRID_RES * 0.5) {
                double px = dx + ca * rr, py = dy + sa * rr;
                if (px < MAP_X_MIN || px > MAP_X_MAX || py < MAP_Y_MIN || py > MAP_Y_MAX)
                    break;
                GridPos gp = worldToGrid(px, py);
                if (!gp.valid()) break;
                if (map.get(gp) == CS_OCCUPIED) break;   // blocked by obstacle
                last_r = rr;
            }
            wx = dx + ca * last_r;
            wy = dy + sa * last_r;
        };

        // Build triangle-fan vertices: centre + (occluded) arc points
        std::vector<SDL_Vertex> verts;
        verts.reserve(N + 2);
        SDL_Color fill = {0, 220, 140, 60};  // semi-transparent green
        verts.push_back({{(float)cx, (float)cy}, {fill.r,fill.g,fill.b,fill.a}, {0,0}});

        std::vector<std::pair<float,float>> arc;  // screen-space arc points
        for (int i = 0; i <= N; ++i) {
            double a = h - half + fov_angle_ * i / N;
            double wx, wy; rayEnd(a, wx, wy);
            int sx, sy; slamToScreen(wx, wy, sx, sy);
            arc.emplace_back((float)sx, (float)sy);
            verts.push_back({{arc.back().first, arc.back().second},
                             {fill.r,fill.g,fill.b,fill.a}, {0,0}});
        }

        // Build indices for triangle fan: (0, i, i+1)
        std::vector<int> idx; idx.reserve(N * 3);
        for (int i = 0; i < N; ++i) {
            idx.push_back(0); idx.push_back(i+1); idx.push_back(i+2);
        }

        SDL_SetRenderDrawBlendMode(renderer_, SDL_BLENDMODE_BLEND);
        SDL_RenderGeometry(renderer_, nullptr,
            verts.data(), (int)verts.size(),
            idx.data(), (int)idx.size());
        SDL_SetRenderDrawBlendMode(renderer_, SDL_BLENDMODE_NONE);

        // Outline edges + arc
        SDL_SetRenderDrawColor(renderer_, 0, 255, 140, 180);
        for (int i = 0; i < N; ++i) {
            SDL_RenderDrawLine(renderer_,
                (int)verts[1+i].position.x, (int)verts[1+i].position.y,
                (int)verts[2+i].position.x, (int)verts[2+i].position.y);
        }
        SDL_RenderDrawLine(renderer_, cx, cy,
            (int)verts[1].position.x, (int)verts[1].position.y);
        SDL_RenderDrawLine(renderer_, cx, cy,
            (int)verts[N+1].position.x, (int)verts[N+1].position.y);
    }

    void fillCircle(int cx, int cy, int r, SDL_Color c) {
        SDL_SetRenderDrawColor(renderer_, c.r,c.g,c.b,c.a);
        for (int dy = -r; dy <= r; ++dy) { int w = (int)std::sqrt(r*r-dy*dy); SDL_RenderDrawLine(renderer_, cx-w,cy+dy,cx+w,cy+dy); }
    }

    /// True if a free/FOV cell sits within INFLATE_RADIUS of an obstacle
    /// (i.e. inside the inflated halo). Caller decides explore-region gating.
    static bool isInflated(const GridMap& m, GridPos p) {
        if (m.get(p) != CS_FREE && m.get(p) != CS_FOV_FREE) return false;
        for (int dy = -INFLATE_RADIUS; dy <= INFLATE_RADIUS; ++dy)
            for (int dx = -INFLATE_RADIUS; dx <= INFLATE_RADIUS; ++dx)
                if (dx || dy) if (m.get(p.x + dx, p.y + dy) == CS_OCCUPIED) return true;
        return false;
    }

    Px4ExplorationPlanner* planner_ = nullptr;
    double *drone_x_=nullptr, *drone_y_=nullptr, *drone_yaw_=nullptr;
    std::mutex* mtx_ = nullptr;
    SDL_Window* window_ = nullptr; SDL_Renderer* renderer_ = nullptr;
    std::thread thread_; std::atomic<bool> running_{false};
    double x1_,x2_,y1_,y2_,x_range_,y_range_; int win_x_,win_y_,cell_px_;
    double fov_angle_ = 0, fov_range_ = 0;  // FOV sector display
};
