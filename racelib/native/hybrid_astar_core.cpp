#include <cmath>
#include <cstdint>
#include <vector>
#include <queue>
#include <algorithm>
#include <chrono>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {

inline float wrap_pi(float a) {
    while (a >= static_cast<float>(M_PI)) a -= static_cast<float>(2.0 * M_PI);
    while (a < -static_cast<float>(M_PI)) a += static_cast<float>(2.0 * M_PI);
    return a;
}

struct Pose {
    float x, y, yaw;
};

inline Pose integrate_pose(const Pose& p, float distance, float curvature) {
    float da = distance * curvature;
    if (std::abs(curvature) < 1e-8f) {
        return {p.x + distance * std::cos(p.yaw),
                p.y + distance * std::sin(p.yaw),
                p.yaw};
    }
    return {p.x + (std::sin(p.yaw + da) - std::sin(p.yaw)) / curvature,
            p.y + (std::cos(p.yaw) - std::cos(p.yaw + da)) / curvature,
            wrap_pi(p.yaw + da)};
}

struct Point2D {
    float x, y;
};

struct Obstacle {
    float x, y, r;
};

// Check point inside convex polygon using cross products
inline bool inside_polygon(const Point2D* poly, int n, float px, float py) {
    if (n < 3) return true;
    for (int i = 0; i < n; ++i) {
        int next = (i + 1 == n) ? 0 : i + 1;
        float dx = poly[next].x - poly[i].x;
        float dy = poly[next].y - poly[i].y;
        float vx = px - poly[i].x;
        float vy = py - poly[i].y;
        // z-component of cross product (dx, dy) x (vx, vy)
        if (dx * vy - dy * vx < -1e-5f) {
            return false;
        }
    }
    return true;
}

class CollisionChecker {
public:
    float field_len;
    float field_wid;
    const Point2D* boundary_pts;
    int num_boundary;
    std::vector<Obstacle> cones;
    float hl;
    float hw;
    float bound;

    CollisionChecker(float flen, float fwid,
                     const Point2D* bpts, int nb,
                     const Obstacle* obs, int nobs,
                     float clen, float cwid, float margin)
        : field_len(flen), field_wid(fwid),
          boundary_pts(bpts), num_boundary(nb),
          hl(clen * 0.5f + margin), hw(cwid * 0.5f + margin) {
        bound = std::hypot(hl, hw);
        cones.assign(obs, obs + nobs);
    }

    inline bool clear(const Pose& p) const {
        float ca = std::cos(p.yaw);
        float sa = std::sin(p.yaw);
        float ex = std::abs(ca) * hl + std::abs(sa) * hw;
        float ey = std::abs(sa) * hl + std::abs(ca) * hw;

        if (p.x < ex || p.y < ey || p.x > field_len - ex || p.y > field_wid - ey) {
            return false;
        }

        if (num_boundary >= 3 && boundary_pts != nullptr) {
            const float dxs[4] = {hl, hl, -hl, -hl};
            const float dys[4] = {hw, -hw, hw, -hw};
            for (int k = 0; k < 4; ++k) {
                float cx = p.x + ca * dxs[k] - sa * dys[k];
                float cy = p.y + sa * dxs[k] + ca * dys[k];
                if (!inside_polygon(boundary_pts, num_boundary, cx, cy)) {
                    return false;
                }
            }
        }

        for (const auto& c : cones) {
            float dx = c.x - p.x;
            float dy = c.y - p.y;
            float r_bound = bound + c.r;
            if (std::abs(dx) > r_bound || std::abs(dy) > r_bound) continue;

            float bx = ca * dx + sa * dy;
            float by = -sa * dx + ca * dy;
            float qx = std::max(std::abs(bx) - hl, 0.0f);
            float qy = std::max(std::abs(by) - hw, 0.0f);
            if (qx * qx + qy * qy <= c.r * c.r) {
                return false;
            }
        }
        return true;
    }

    inline bool edge(const Pose& p, float distance, float curvature) const {
        int n = std::max(1, static_cast<int>(std::abs(distance) / 0.04f) + 1);
        for (int i = 0; i <= n; ++i) {
            Pose sub = integrate_pose(p, distance * static_cast<float>(i) / static_cast<float>(n), curvature);
            if (!clear(sub)) return false;
        }
        return true;
    }
};

struct SearchNode {
    Pose pose;
    int8_t gear;
    int32_t parent;
    float cost;
    float k;
    int16_t reversed_steps;
};

struct QueueItem {
    float priority;
    int32_t node_idx;

    bool operator>(const QueueItem& other) const {
        return priority > other.priority;
    }
};

// 2D Dijkstra Heuristic Grid
class HeuristicGrid {
public:
    int nx, ny;
    float resolution;
    std::vector<float> dist;
    Point2D goal_pt;

    HeuristicGrid(float flen, float fwid, const Point2D& goal,
                  const std::vector<Obstacle>& cones, float res = 0.12f)
        : resolution(res), goal_pt(goal) {
        nx = static_cast<int>(flen / res) + 1;
        ny = static_cast<int>(fwid / res) + 1;
        int total = nx * ny;
        std::vector<bool> occupied(total, false);

        for (int ix = 0; ix < nx; ++ix) {
            float gx = ix * res;
            for (int iy = 0; iy < ny; ++iy) {
                float gy = iy * res;
                for (const auto& c : cones) {
                    float r_inflated = c.r + 0.19f;
                    float dx = gx - c.x;
                    float dy = gy - c.y;
                    if (dx * dx + dy * dy < r_inflated * r_inflated) {
                        occupied[ix * ny + iy] = true;
                        break;
                    }
                }
            }
        }

        dist.assign(total, 1e9f);
        int g_ix = std::min(nx - 1, std::max(0, static_cast<int>(std::round(goal.x / res))));
        int g_iy = std::min(ny - 1, std::max(0, static_cast<int>(std::round(goal.y / res))));
        int goal_cell = g_ix * ny + g_iy;
        dist[goal_cell] = 0.0f;

        struct CellItem {
            float d;
            int x, y;
            bool operator>(const CellItem& o) const { return d > o.d; }
        };
        std::priority_queue<CellItem, std::vector<CellItem>, std::greater<CellItem>> pq;
        pq.push({0.0f, g_ix, g_iy});

        const int dxs[8] = {1, -1, 0, 0, 1, 1, -1, -1};
        const int dys[8] = {0, 0, 1, -1, 1, -1, 1, -1};
        const float dcosts[8] = {
            res, res, res, res,
            res * 1.41421356f, res * 1.41421356f, res * 1.41421356f, res * 1.41421356f
        };

        while (!pq.empty()) {
            auto top = pq.top();
            pq.pop();
            int idx = top.x * ny + top.y;
            if (top.d > dist[idx] + 1e-6f) continue;

            for (int k = 0; k < 8; ++k) {
                int xx = top.x + dxs[k];
                int yy = top.y + dys[k];
                if (xx < 0 || xx >= nx || yy < 0 || yy >= ny) continue;
                int nidx = xx * ny + yy;
                if (occupied[nidx]) continue;

                float nd = top.d + dcosts[k];
                if (nd < dist[nidx]) {
                    dist[nidx] = nd;
                    pq.push({nd, xx, yy});
                }
            }
        }
    }

    inline float get(float x, float y) const {
        float euc = std::hypot(x - goal_pt.x, y - goal_pt.y);
        int ix = std::min(nx - 1, std::max(0, static_cast<int>(std::round(x / resolution))));
        int iy = std::min(ny - 1, std::max(0, static_cast<int>(std::round(y / resolution))));
        float d = dist[ix * ny + iy];
        if (d < 1e8f) {
            return std::max(euc, d);
        }
        return euc;
    }
};

// Spatial index table for fast O(1) state visitation lookup
constexpr int X_BINS = 120; // 0..10.8m @ 0.09m
constexpr int Y_BINS = 70;  // 0..6.3m @ 0.09m
constexpr int A_BINS = 48;  // 2pi / (pi/24) = 48
constexpr int GEAR_BINS = 2; // 1, -1
constexpr int REV_BINS = 10; // reversed step count
constexpr size_t TOTAL_STATE_BINS = static_cast<size_t>(REV_BINS) * GEAR_BINS * A_BINS * Y_BINS * X_BINS;

inline uint32_t compute_bin_index(float x, float y, float yaw, int8_t gear, int16_t rev) {
    int ix = static_cast<int>(std::round(x / 0.09f));
    int iy = static_cast<int>(std::round(y / 0.09f));
    ix = std::max(0, std::min(X_BINS - 1, ix));
    iy = std::max(0, std::min(Y_BINS - 1, iy));
    int ia = static_cast<int>(std::round(wrap_pi(yaw) / (static_cast<float>(M_PI) / 24.0f))) % 48;
    if (ia < 0) ia += 48;
    int ig = (gear == 1) ? 0 : 1;
    int ir = std::max(0, std::min(REV_BINS - 1, static_cast<int>(rev)));
    return ((((ir * GEAR_BINS + ig) * A_BINS + ia) * Y_BINS + iy) * X_BINS + ix);
}

} // namespace

extern "C" {

int c_hybrid_astar_plan(
    float start_x, float start_y, float start_yaw,
    float goal_x, float goal_y,
    float field_length, float field_width,
    const float* boundary_xy, int num_boundary_pts,
    const float* obs_xyr, int num_obs,
    float car_len, float car_wid, float margin,
    float wheelbase, float steer_limit,
    int allow_reverse, int max_nodes, float timeout_sec,
    float max_reverse_m, int prefer_forward,
    float* out_x, float* out_y, float* out_yaw, int8_t* out_gear,
    int max_out_len
) {
    if (max_reverse_m < 0.0f) return -1;

    std::vector<Point2D> b_pts;
    if (boundary_xy && num_boundary_pts >= 3) {
        b_pts.resize(num_boundary_pts);
        for (int i = 0; i < num_boundary_pts; ++i) {
            b_pts[i] = {boundary_xy[2 * i], boundary_xy[2 * i + 1]};
        }
    }

    std::vector<Obstacle> obs;
    if (obs_xyr && num_obs > 0) {
        obs.resize(num_obs);
        for (int i = 0; i < num_obs; ++i) {
            obs[i] = {obs_xyr[3 * i], obs_xyr[3 * i + 1], std::max(0.115f, obs_xyr[3 * i + 2])};
        }
    }

    CollisionChecker world(field_length, field_width,
                           b_pts.empty() ? nullptr : b_pts.data(), static_cast<int>(b_pts.size()),
                           obs.data(), static_cast<int>(obs.size()),
                           car_len, car_wid, margin);

    Pose start_pose = {start_x, start_y, start_yaw};
    Point2D goal_pt = {goal_x, goal_y};

    // Forward-first search
    if (allow_reverse && prefer_forward) {
        auto t_start = std::chrono::steady_clock::now();
        int fwd_count = c_hybrid_astar_plan(
            start_x, start_y, start_yaw,
            goal_x, goal_y,
            field_length, field_width,
            boundary_xy, num_boundary_pts,
            obs_xyr, num_obs,
            car_len, car_wid, margin,
            wheelbase, steer_limit,
            0, max_nodes, timeout_sec * 0.6f,
            max_reverse_m, 0,
            out_x, out_y, out_yaw, out_gear, max_out_len
        );
        if (fwd_count > 0) {
            return fwd_count;
        }
        auto t_end = std::chrono::steady_clock::now();
        float elapsed = std::chrono::duration<float>(t_end - t_start).count();
        timeout_sec -= elapsed;
        if (timeout_sec <= 0.0f) return 0;
    }

    HeuristicGrid grid(field_length, field_width, goal_pt, obs);

    float curvature = std::tan(steer_limit) / wheelbase;
    float step = 0.22f;
    int reverse_budget = static_cast<int>((max_reverse_m + 1e-8f) / step);

    static thread_local std::vector<float> best_cost_table;
    static thread_local std::vector<uint32_t> dirty_bins;
    if (best_cost_table.size() != TOTAL_STATE_BINS) {
        best_cost_table.assign(TOTAL_STATE_BINS, 1e9f);
    } else {
        for (uint32_t idx : dirty_bins) {
            best_cost_table[idx] = 1e9f;
        }
        dirty_bins.clear();
    }

    std::vector<SearchNode> nodes;
    nodes.reserve(std::min(max_nodes, 40000));
    nodes.push_back({start_pose, 1, -1, 0.0f, 0.0f, 0});

    uint32_t start_bin = compute_bin_index(start_pose.x, start_pose.y, start_pose.yaw, 1, 0);
    best_cost_table[start_bin] = 0.0f;
    dirty_bins.push_back(start_bin);

    std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> open_set;
    open_set.push({grid.get(start_pose.x, start_pose.y), 0});

    auto start_time = std::chrono::steady_clock::now();
    int goal_node_idx = -1;

    const float k_branches[5] = {
        -curvature, -0.5f * curvature, 0.0f, 0.5f * curvature, curvature
    };
    const int8_t dir_branches[2] = {1, -1};
    int num_dirs = allow_reverse ? 2 : 1;

    while (!open_set.empty() && static_cast<int>(nodes.size()) < max_nodes) {
        if (nodes.size() % 128 == 0) {
            auto now = std::chrono::steady_clock::now();
            float elapsed = std::chrono::duration<float>(now - start_time).count();
            if (elapsed > timeout_sec) break;
        }

        auto top = open_set.top();
        open_set.pop();
        int32_t cur_idx = top.node_idx;
        const auto& cur = nodes[cur_idx];

        uint32_t cur_bin = compute_bin_index(cur.pose.x, cur.pose.y, cur.pose.yaw, cur.gear, cur.reversed_steps);
        if (cur.cost > best_cost_table[cur_bin] + 1e-6f) {
            continue;
        }

        float h = grid.get(cur.pose.x, cur.pose.y);
        if (h < 0.12f) {
            goal_node_idx = cur_idx;
            break;
        }

        for (int d_i = 0; d_i < num_dirs; ++d_i) {
            int8_t dir = dir_branches[d_i];
            int16_t next_reversed = cur.reversed_steps + (dir < 0 ? 1 : 0);
            if (next_reversed > reverse_budget) continue;

            for (float k : k_branches) {
                if (!world.edge(cur.pose, dir * step, k)) continue;

                Pose nxt = integrate_pose(cur.pose, dir * step, k);
                float nc = cur.cost + step * (dir == 1 ? 1.0f : 1.5f);
                nc += (dir != cur.gear ? 0.45f : 0.0f) + 0.025f * std::abs(k - cur.k);

                uint32_t nxt_bin = compute_bin_index(nxt.x, nxt.y, nxt.yaw, dir, next_reversed);
                if (nc + 1e-6f >= best_cost_table[nxt_bin]) {
                    continue;
                }

                best_cost_table[nxt_bin] = nc;
                dirty_bins.push_back(nxt_bin);

                int32_t nidx = static_cast<int32_t>(nodes.size());
                nodes.push_back({nxt, dir, cur_idx, nc, k, next_reversed});

                float nh = grid.get(nxt.x, nxt.y);
                open_set.push({nc + 1.8f * nh, nidx});
            }
        }
    }

    if (goal_node_idx < 0) {
        return 0; // Failed
    }

    // Backtrack and interpolate
    std::vector<Pose> path_poses;
    std::vector<int8_t> path_gears;

    int32_t curr = goal_node_idx;
    while (curr >= 0) {
        const auto& node = nodes[curr];
        int32_t p_idx = node.parent;
        if (p_idx >= 0) {
            const auto& p_node = nodes[p_idx];
            // 6-point sub-interpolation
            for (int s = 0; s < 6; ++s) {
                float frac = static_cast<float>(s) / 5.0f;
                float d = (step * (1.0f - frac) + 0.04f * frac) * node.gear;
                Pose sub = integrate_pose(p_node.pose, d, node.k);
                path_poses.push_back(sub);
                path_gears.push_back(node.gear);
            }
        }
        curr = p_idx;
    }

    path_poses.push_back(start_pose);
    path_gears.push_back(path_gears.empty() ? 1 : path_gears.back());

    std::reverse(path_poses.begin(), path_poses.end());
    std::reverse(path_gears.begin(), path_gears.end());

    int total_pts = static_cast<int>(path_poses.size());
    if (total_pts > max_out_len) total_pts = max_out_len;

    for (int i = 0; i < total_pts; ++i) {
        out_x[i] = path_poses[i].x;
        out_y[i] = path_poses[i].y;
        out_yaw[i] = path_poses[i].yaw;
        out_gear[i] = path_gears[i];
    }

    return total_pts;
}

} // extern "C"
