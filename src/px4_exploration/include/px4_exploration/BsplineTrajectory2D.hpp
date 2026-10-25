#pragma once

#include "Types.hpp"
#include <vector>
#include <algorithm>
#include <cmath>

/// 2D uniform cubic B-spline — minimal, no external dependencies.
///
/// Control points P₀...Pₙ, order p=3, uniform knot interval dt.
/// Valid parameter range: t ∈ [3·dt, (n−2)·dt].
///
/// This is the core primitive that makes EGO trajectories smooth:
///   - C² continuous (pos, vel, acc all continuous)
///   - Time-parameterised: each t has a definite pos/vel/acc
///   - Derivative splines give exact vel/acc without finite-differencing
class UniformBspline2D {
public:
    UniformBspline2D() : dt_(0.1), n_(0), m_(0) {}

    /// @param ctrl_pts  World-frame control points (at least 4 for cubic)
    /// @param dt        Knot interval in seconds
    UniformBspline2D(const std::vector<Point>& ctrl_pts, double dt)
        : ctrl_(ctrl_pts), dt_(dt)
    {
        n_ = static_cast<int>(ctrl_.size()) - 1;
        m_ = n_ + ORDER + 1;

        knots_.resize(m_ + 1);
        for (int i = 0; i <= m_; ++i)
            knots_[i] = static_cast<double>(-ORDER + i) * dt_;
    }

    // ---- queries ----

    double duration() const {
        if (n_ < ORDER) return 0.0;
        return knots_[m_ - ORDER] - knots_[ORDER];
    }

    double tmin() const { return ctrl_.size() < 4 ? 0.0 : knots_[ORDER]; }
    double tmax() const { return ctrl_.size() < 4 ? 0.0 : knots_[m_ - ORDER]; }

    /// Position at time t (de Boor)
    Point pos(double t) const {
        double u = clamp(t);
        int k = span(u);
        std::vector<Point> d(ORDER + 1);
        for (int i = 0; i <= ORDER; ++i) d[i] = ctrl_[k - ORDER + i];
        deBoorStep(d, u, k, ORDER);
        return d[ORDER];
    }

    /// Velocity (first derivative) at time t
    Point vel(double t) const {
        auto d = derivCtrl(1);
        if (d.empty()) return Point(0, 0);
        return evalDeriv(d, ORDER - 1, t);
    }

    /// Acceleration (second derivative) at time t
    Point acc(double t) const {
        auto d = derivCtrl(2);
        if (d.empty()) return Point(0, 0);
        return evalDeriv(d, ORDER - 2, t);
    }

    // ---- accessors ----

    const std::vector<Point>& ctrlPoints() const { return ctrl_; }
    std::vector<Point>&       ctrlPoints()       { return ctrl_; }
    double interval() const { return dt_; }
    int    numCtrl()   const { return n_ + 1; }

    /// Number of spline segments with valid parameters
    int validSegments() const { return std::max(0, n_ - ORDER + 1); }

    // ---- factory ----

    /// Build initial B-spline from a geometric path (e.g. A* waypoints).
    /// Places control points evenly along the path, sets dt from path length
    /// and desired speed. The result is a reasonable starting point for
    /// gradient-descent optimisation.
    static UniformBspline2D fromPath(const std::vector<Point>& path,
                                      double cruise_speed)
    {
        if (path.size() < 2) {
            std::vector<Point> c(4, path.empty() ? Point(0,0) : path.front());
            if (path.size() == 2) { c[2] = path[1]; c[3] = path[1]; }
            return UniformBspline2D(c, 0.1);
        }

        // Total path length
        double total = 0.0;
        for (size_t i = 1; i < path.size(); ++i)
            total += path[i].dist(path[i - 1]);

        double speed = std::max(cruise_speed, 0.1);
        double duration = total / speed;
        int segs = std::max(3, std::min(20, static_cast<int>(path.size()) + 4));
        double dt = duration / segs;
        if (dt < 0.03) dt = 0.03;

        // Place N+1 control points (segs + ORDER = segs + 3)
        int N = segs + ORDER;  // number of control points - 1
        std::vector<Point> ctrl(N + 1);

        // First control points: repeat start (boundary condition)
        ctrl[0] = path.front();
        ctrl[1] = path.front();

        // Middle: sample path evenly
        for (int i = 2; i <= N - 2; ++i) {
            double frac = static_cast<double>(i - 1) / (N - 2);
            double target = frac * total;
            double acc = 0.0;
            size_t j = 0;
            for (j = 1; j < path.size() && acc < target; ++j)
                acc += path[j].dist(path[j - 1]);
            j = std::min(j, path.size() - 1);
            // Linear interpolation within segment
            double seg_start = acc - path[j].dist(path[j - 1]);
            double seg_len = path[j].dist(path[j - 1]);
            double alpha = (seg_len > 0.001) ? (target - seg_start) / seg_len : 0.0;
            alpha = std::clamp(alpha, 0.0, 1.0);
            ctrl[i].x = path[j - 1].x + alpha * (path[j].x - path[j - 1].x);
            ctrl[i].y = path[j - 1].y + alpha * (path[j].y - path[j - 1].y);
        }

        // Last control points: repeat end
        ctrl[N - 1] = path.back();
        ctrl[N]     = path.back();

        return UniformBspline2D(ctrl, dt);
    }

private:
    static constexpr int ORDER = 3;

    double clamp(double t) const {
        double lo = knots_[ORDER], hi = knots_[m_ - ORDER];
        return std::clamp(t, lo, hi);
    }

    int span(double u) const {
        if (u >= knots_[m_ - ORDER]) return m_ - ORDER - 1;
        int lo = ORDER, hi = m_ - ORDER;
        while (lo < hi) {
            int mid = (lo + hi + 1) / 2;
            if (knots_[mid] <= u) lo = mid; else hi = mid - 1;
        }
        return lo;
    }

    /// One pass of de Boor recursion (in-place on d[0..order])
    void deBoorStep(std::vector<Point>& d, double u, int k, int p) const {
        for (int r = 1; r <= p; ++r) {
            for (int i = p; i >= r; --i) {
                double denom = knots_[i + 1 + k - r] - knots_[i + k - p];
                double alpha = (denom > 1e-12) ? (u - knots_[i + k - p]) / denom : 0.0;
                d[i].x = (1.0 - alpha) * d[i - 1].x + alpha * d[i].x;
                d[i].y = (1.0 - alpha) * d[i - 1].y + alpha * d[i].y;
            }
        }
    }

    /// Compute control points for the d-th derivative
    std::vector<Point> derivCtrl(int d) const {
        std::vector<Point> cur = ctrl_;
        int p = ORDER;
        for (int r = 0; r < d; ++r) {
            if (cur.size() < 2) return {};
            std::vector<Point> nxt(cur.size() - 1);
            for (size_t i = 0; i < nxt.size(); ++i) {
                double denom = knots_[i + p + 1] - knots_[i + 1];
                if (denom < 1e-12) denom = dt_;
                nxt[i].x = p * (cur[i + 1].x - cur[i].x) / denom;
                nxt[i].y = p * (cur[i + 1].y - cur[i].y) / denom;
            }
            cur = nxt;
            --p;
        }
        return cur;
    }

    /// Evaluate derivative spline given its control points and reduced order
    Point evalDeriv(const std::vector<Point>& dctrl, int p, double t) const {
        if (dctrl.empty()) return Point(0, 0);
        int nm = static_cast<int>(dctrl.size()) - 1 + p + 1;
        std::vector<double> dk(nm + 1);
        for (int i = 0; i <= nm; ++i)
            dk[i] = static_cast<double>(-p + i) * dt_;

        double lo = dk[p], hi = dk[nm - p];
        double u = std::clamp(t, lo, hi);
        int k = p;
        while (k < nm - p && !(dk[k] <= u && u < dk[k + 1])) ++k;
        if (k >= nm - p) k = nm - p - 1;

        std::vector<Point> dd(p + 1);
        for (int i = 0; i <= p; ++i) {
            int idx = k - p + i;
            dd[i] = (idx >= 0 && idx < static_cast<int>(dctrl.size())) ? dctrl[idx] : Point(0, 0);
        }
        for (int r = 1; r <= p; ++r) {
            for (int i = p; i >= r; --i) {
                double denom = dk[i + 1 + k - r] - dk[i + k - p];
                double alpha = (denom > 1e-12) ? (u - dk[i + k - p]) / denom : 0.0;
                dd[i].x = (1.0 - alpha) * dd[i - 1].x + alpha * dd[i].x;
                dd[i].y = (1.0 - alpha) * dd[i - 1].y + alpha * dd[i].y;
            }
        }
        return dd[p];
    }

    std::vector<Point>  ctrl_;
    std::vector<double> knots_;
    double dt_;
    int n_, m_;
};
