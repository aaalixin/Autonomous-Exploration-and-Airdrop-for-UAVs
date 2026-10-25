#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>
#include <utility>
#include <set>
#include <limits>

// ===== Map bounds (set at runtime from launch params) =====
extern double MAP_X_MIN, MAP_X_MAX, MAP_Y_MIN, MAP_Y_MAX;
extern double EDGE_MARGIN;  // min distance from path endpoint to map edge

// ===== Exploration sub-region =====
// Exploration (free-space marking, coverage %, viewpoint candidates) happens
// ONLY inside this sub-rectangle of the full map.  The full MAP_* rectangle is
// the whole flyable frame (grid/A*/GUI), which also contains the landing
// corridor east of the internal wall (x>7.5).  The corridor must NOT be marked
// free / explored — the drone only flies through it (allowUnknown) when
// returning to a landing pad after exploration is complete.
//   x < 7.5  : explore the big hall west of the internal wall
//   x > 7.5  : landing corridor (never an exploration target)
extern double EXPLORE_X_MIN, EXPLORE_X_MAX, EXPLORE_Y_MIN, EXPLORE_Y_MAX;

/// True if a world point lies inside the exploration sub-region.
/// Obstacle inflation is ONLY applied inside this region: the landing corridor
/// east of the internal wall stays un-inflated so the drone can fly down the
/// narrow corridor to the pad on its return leg.  Walls/obstacles themselves
/// (CS_OCCUPIED) still block everywhere — only the inflated halo is dropped.
inline bool inExploreRegion(double wx, double wy) {
    return wx >= EXPLORE_X_MIN && wx <= EXPLORE_X_MAX &&
           wy >= EXPLORE_Y_MIN && wy <= EXPLORE_Y_MAX;
}

// ===== Map constants =====
constexpr double MAP_SIZE      = 10.0;   // fallback / GUI default
constexpr double GRID_RES      = 0.025;  // ★2.5cm per cell (窄通道/薄墙更细; 覆盖范围不变=GRID_N*GRID_RES)
constexpr int    GRID_N        = 600;    // max cells per axis (covers 15m @2.5cm)
constexpr int    WIN_SIZE      = 800;    // window pixels
constexpr double SCALE         = WIN_SIZE / MAP_SIZE;  // pixels per meter
constexpr int    CELL_PX       = WIN_SIZE / GRID_N;    // pixels per cell (5cm → 2px)

// ===== Path planning =====
extern int INFLATE_RADIUS;  // default 3; can be set at runtime via launch param
/// A→B(走廊段, 即非探索区)离障碍物/墙的边缘距离(m), launch 参数 ab_edge_margin。
/// 探索区内仍然用 INFLATE_RADIUS*GRID_RES + DRONE_RADIUS(膨胀光晕 + 机身);
/// 探索区外(走廊/pad 区)不加膨胀光晕, 只用这个边缘距离 —— 走廊本来就窄,
/// 加光晕会过不去。只作用于规划, 不改变机身尺寸 DRONE_RADIUS。
extern thread_local double AB_EDGE_MARGIN;   // default 0.25 ★thread_local: 后台规划线程有独立副本(走廊规划会临时改它)

// ===== Sensor constants =====
constexpr double SENSOR_FOV_DEG  = 120.0;
constexpr double SENSOR_FOV_RAD  = SENSOR_FOV_DEG * M_PI / 180.0;
constexpr double SENSOR_RANGE    = 3.0;
constexpr int    SENSOR_NUM_RAYS = 120;

// ===== Drone constants =====
extern double DRONE_SPEED;        // m/s (cruise speed), set from launch
extern double MAX_ACCEL;          // m/s², set from launch
extern double MAX_YAW_RATE;       // rad/s, set from launch
constexpr double DRONE_RADIUS    = 0.25;  // m — half width of the 50×50cm airframe
                                          // (body circle collision radius).  The
                                          // planner must keep this clear of walls.
constexpr double WAYPOINT_TOL    = 0.1;   // m, when to consider waypoint reached
constexpr double SENSE_INTERVAL  = 0.15;  // seconds between sensor updates
constexpr double LOOKAHEAD_DIST  = 0.6;   // m — pure-pursuit look-ahead
constexpr double SPLINE_RES      = 0.04;  // m — Catmull-Rom interpolation spacing

// ===== Obstacle generation =====
constexpr int    OBS_MIN_COUNT   = 8;
constexpr int    OBS_MAX_COUNT   = 12;
constexpr double OBS_MIN_RADIUS  = 0.15;
constexpr double OBS_MAX_RADIUS  = 0.30;

// ===== Math =====
constexpr double PI = M_PI;
constexpr double DEG2RAD = PI / 180.0;
constexpr double RAD2DEG = 180.0 / PI;

// ===== Cell state =====
enum CellState : uint8_t {
    CS_UNKNOWN  = 0,
    CS_FREE     = 1,    // LiDAR-known free
    CS_OCCUPIED = 2,    // obstacle
    CS_FOV_FREE = 3     // camera-FOV explored (counts toward exploration %)
};

// ===== 2D Point (world coordinates) =====
struct Point {
    double x, y;
    Point() : x(0), y(0) {}
    Point(double x_, double y_) : x(x_), y(y_) {}

    double distSq(const Point& o) const {
        double dx = x - o.x, dy = y - o.y;
        return dx*dx + dy*dy;
    }
    double dist(const Point& o) const {
        return std::sqrt(distSq(o));
    }
};

// ===== Circular obstacle (ground truth) =====
struct Obstacle {
    double cx, cy, r;
    Obstacle(double cx_, double cy_, double r_) : cx(cx_), cy(cy_), r(r_) {}

    bool contains(double px, double py) const {
        double dx = px - cx, dy = py - cy;
        return dx*dx + dy*dy <= r*r;
    }
};

// ===== Grid coordinate =====
struct GridPos {
    int x, y;  // x: 0..GRID_N-1 (right), y: 0..GRID_N-1 (up)
    GridPos() : x(0), y(0) {}
    GridPos(int x_, int y_) : x(x_), y(y_) {}

    bool operator==(const GridPos& o) const { return x == o.x && y == o.y; }
    bool operator!=(const GridPos& o) const { return !(*this == o); }
    bool operator<(const GridPos& o) const {
        return x < o.x || (x == o.x && y < o.y);
    }

    bool valid() const {
        return x >= 0 && x < GRID_N && y >= 0 && y < GRID_N;
    }
};

// Convert world -> grid (uses runtime map bounds)
inline GridPos worldToGrid(double wx, double wy) {
    int gx = std::clamp(static_cast<int>((wx - MAP_X_MIN) / GRID_RES), 0, GRID_N - 1);
    int gy = std::clamp(static_cast<int>((wy - MAP_Y_MIN) / GRID_RES), 0, GRID_N - 1);
    return GridPos(gx, gy);
}

// Convert grid (center) -> world
inline Point gridToWorld(int gx, int gy) {
    return Point(MAP_X_MIN + (gx + 0.5) * GRID_RES,
                 MAP_Y_MIN + (gy + 0.5) * GRID_RES);
}

// Convert world -> screen pixel
inline void worldToScreen(double wx, double wy, int& sx, int& sy) {
    sx = static_cast<int>(wx * SCALE);
    sy = WIN_SIZE - 1 - static_cast<int>(wy * SCALE);  // flip Y for screen
}

// Convert screen -> world
inline Point screenToWorld(int sx, int sy) {
    return Point(sx / SCALE, (WIN_SIZE - 1 - sy) / SCALE);
}

// ===== Drone state for path following (PX4 version) =====
struct DroneState {
    double x, y;       // world position
    double vx, vy;     // actual velocity from odometry (m/s)
    double heading;    // radians (yaw)
    double speed;      // desired speed (m/s)
    double yaw_rate;   // current yaw rate (rad/s)
    Point  target;     // current navigation target (world)
    bool   moving = false;

    DroneState() : x(0), y(0), vx(0), vy(0), heading(0), speed(0), yaw_rate(0), target(0, 0) {}
};
