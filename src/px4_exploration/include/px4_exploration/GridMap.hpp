#pragma once

#include "Types.hpp"
#include <cstring>
#include <vector>

/// Occupancy grid: the drone's incremental knowledge of the world.
class GridMap {
public:
    GridMap() { clear(); }

    void clear() {
        std::memset(data_, static_cast<uint8_t>(CS_UNKNOWN), sizeof(data_));
        changed_.clear();
    }

    CellState get(int gx, int gy) const {
        if (gx < 0 || gx >= GRID_N || gy < 0 || gy >= GRID_N) return CS_UNKNOWN;
        return static_cast<CellState>(data_[gy][gx]);
    }
    CellState get(GridPos p) const { return get(p.x, p.y); }

    void set(int gx, int gy, CellState s) {
        if (gx < 0 || gx >= GRID_N || gy < 0 || gy >= GRID_N) return;
        if (data_[gy][gx] != s) {
            data_[gy][gx] = static_cast<uint8_t>(s);
            changed_.emplace_back(gx, gy);
        }
    }
    void set(GridPos p, CellState s) { set(p.x, p.y, s); }

    /// Returns cells changed since last clearChanges().
    const std::vector<GridPos>& getChanges() const { return changed_; }
    void clearChanges() { changed_.clear(); }

    /// Clear all CS_OCCUPIED cells back to UNKNOWN (for full re-mark from cloud).
    void clearObstacles() {
        for (int y = 0; y < GRID_N; ++y)
            for (int x = 0; x < GRID_N; ++x)
                if (data_[y][x] == CS_OCCUPIED) {
                    data_[y][x] = CS_UNKNOWN;
                    changed_.emplace_back(x, y);
                }
    }

    /// Clear CS_OCCUPIED only within a forward-facing sector (±half_angle around drone_yaw).
    /// Used when use_global_cloud=0: rear-hemisphere obstacles persist across frames.
    void clearObstaclesInSector(double drone_x, double drone_y,
                                double drone_yaw, double half_angle = M_PI / 2.0) {
        for (int y = 0; y < GRID_N; ++y)
            for (int x = 0; x < GRID_N; ++x) {
                if (data_[y][x] != CS_OCCUPIED) continue;
                double wx = MAP_X_MIN + (x + 0.5) * GRID_RES;
                double wy = MAP_Y_MIN + (y + 0.5) * GRID_RES;
                double ang = std::atan2(wy - drone_y, wx - drone_x);
                double diff = ang - drone_yaw;
                while (diff > M_PI) diff -= 2.0 * M_PI;
                while (diff < -M_PI) diff += 2.0 * M_PI;
                if (std::abs(diff) <= half_angle) {
                    data_[y][x] = CS_UNKNOWN;
                    changed_.emplace_back(x, y);
                }
            }
    }

    /// Is a grid cell traversable (free and has margin from obstacles)?
    bool isTraversable(int gx, int gy, int margin_cells = 1) const {
        for (int dy = -margin_cells; dy <= margin_cells; ++dy) {
            for (int dx = -margin_cells; dx <= margin_cells; ++dx) {
                if (get(gx + dx, gy + dy) != CS_FREE) return false;
            }
        }
        return true;
    }

    /// Percent of the EXPLORATION sub-region that the camera FOV has covered.
    /// Denominator = CS_FREE + CS_FOV_FREE cells whose CENTRE lies strictly
    /// inside [EXPLORE_*].  Cells outside the exploration sub-region (e.g. the
    /// landing corridor) are never counted, even if a FOV sweep marked them.
    /// CS_OCCUPIED excluded — the camera can't "cover" an obstacle.
    double exploredPercent() const {
        // Iterate the whole grid; count a cell only when its centre point is
        // inside the exploration rectangle.  Using grid-centre membership (not
        // a boundary-derived index range) guarantees no off-region FOV counts.
        int fov = 0, free = 0;
        for (int y = 0; y < GRID_N; ++y)
            for (int x = 0; x < GRID_N; ++x) {
                Point c = gridToWorld(x, y);
                if (c.x < EXPLORE_X_MIN || c.x > EXPLORE_X_MAX ||
                    c.y < EXPLORE_Y_MIN || c.y > EXPLORE_Y_MAX)
                    continue;
                if (data_[y][x] == CS_FOV_FREE) ++fov;
                else if (data_[y][x] == CS_FREE) ++free;
            }
        int coverable = fov + free;
        if (coverable <= 0) return 0.0;
        return 100.0 * fov / coverable;
    }

private:
    uint8_t data_[GRID_N][GRID_N];
    std::vector<GridPos> changed_;
};

// (DepthSensor removed – PX4 version uses real depth camera, not simulation)
