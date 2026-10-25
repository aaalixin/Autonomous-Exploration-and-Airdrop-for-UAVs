#pragma once

#include "Types.hpp"
#include "GridMap.hpp"
#include <set>
#include <vector>
#include <queue>

/// Maintains frontier cells incrementally (FUEL's FIS concept).
class FrontierDetector {
public:
    /// Update frontier set based on cells that just changed.
    void update(const GridMap& map, const std::vector<GridPos>& changed) {
        // Process each changed cell and its neighbors
        for (const auto& p : changed) {
            CellState s = map.get(p);
            if (s == CS_FREE) {   // ★A: FOV覆盖过的格子不再算frontier(现场要求 ✓)
                // Check if this cell is now a frontier
                if (hasUnknownNeighbor(map, p)) {
                    frontiers_.insert(p);
                } else {
                    frontiers_.erase(p);
                }
                // Check neighbors: they may have lost/gained frontier status
                for (const auto& nb : neighbors8(p)) {
                    CellState nbs = map.get(nb);
                    if (nbs == CS_FREE && hasUnknownNeighbor(map, nb))
                        frontiers_.insert(nb);
                    else
                        frontiers_.erase(nb);
                }
            } else if (s == CS_OCCUPIED) {
                frontiers_.erase(p);
                for (const auto& nb : neighbors8(p)) {
                    CellState ns = map.get(nb);
                    if (ns == CS_FREE) {
                        if (hasUnknownNeighbor(map, nb))
                            frontiers_.insert(nb);
                        else
                            frontiers_.erase(nb);
                    }
                }
            }
        }
    }

    /// Full scan – used for initial detection.
    void detectAll(const GridMap& map) {
        frontiers_.clear();
        for (int y = 0; y < GRID_N; ++y)
            for (int x = 0; x < GRID_N; ++x) {
                CellState s = map.get(x, y);
                if (s == CS_FREE
                    && hasUnknownNeighbor(map, GridPos(x, y)))
                    frontiers_.insert(GridPos(x, y));
            }
    }

    const std::set<GridPos>& cells() const { return frontiers_; }
    bool empty() const { return frontiers_.empty(); }
    size_t count() const { return frontiers_.size(); }

    /// Cluster frontier cells using flood-fill (BFS).
    /// Returns list of clusters, each as a list of grid positions.
    std::vector<std::vector<GridPos>> cluster(int min_cluster_size = 3) const {
        std::set<GridPos> visited;
        std::vector<std::vector<GridPos>> clusters;

        for (const auto& seed : frontiers_) {
            if (visited.count(seed)) continue;

            std::vector<GridPos> cluster;
            std::queue<GridPos> q;
            q.push(seed);
            visited.insert(seed);

            while (!q.empty()) {
                GridPos cur = q.front(); q.pop();
                cluster.push_back(cur);

                for (const auto& nb : neighbors8(cur)) {
                    if (frontiers_.count(nb) && !visited.count(nb)) {
                        visited.insert(nb);
                        q.push(nb);
                    }
                }
            }

            if (static_cast<int>(cluster.size()) >= min_cluster_size)
                clusters.push_back(std::move(cluster));
        }
        return clusters;
    }

private:
    static bool hasUnknownNeighbor(const GridMap& map, const GridPos& p) {
        for (const auto& nb : neighbors8(p))
            if (map.get(nb) == CS_UNKNOWN) return true;
        return false;
    }

    static std::vector<GridPos> neighbors8(const GridPos& p) {
        std::vector<GridPos> nbs;
        for (int dy = -1; dy <= 1; ++dy)
            for (int dx = -1; dx <= 1; ++dx) {
                if (dx == 0 && dy == 0) continue;
                GridPos nb(p.x + dx, p.y + dy);
                if (nb.valid()) nbs.push_back(nb);
            }
        return nbs;
    }

    std::set<GridPos> frontiers_;
};
