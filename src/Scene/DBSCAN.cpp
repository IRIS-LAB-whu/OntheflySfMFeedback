#include "DBSCAN.h"

DBSCAN::DBSCAN(float eps, int minPts)
    : eps(eps), epsSquared(eps* eps), minPts(minPts) {
    if (eps <= 0.0f || minPts <= 0) {
        throw std::invalid_argument("eps must be > 0 and minPts must be > 0");
    }
}

std::vector<int> DBSCAN::fit(const std::vector<Eigen::Vector3f>& points) {
    if (points.empty()) return {};

    int n = static_cast<int>(points.size());
    visited.assign(n, false);
    labels.assign(n, -1);

    int clusterId = 0;

    for (int i = 0; i < n; ++i) {
        if (visited[i]) continue;

        visited[i] = true;
        std::vector<int> neighbors = regionQuery(points, i);

        if (neighbors.size() < minPts) {
            labels[i] = -1; // noise
        }
        else {
            expandCluster(points, i, clusterId);
            ++clusterId;
        }
    }

    return labels;
}

void DBSCAN::expandCluster(const std::vector<Eigen::Vector3f>& points, int idx, int clusterId) {
    std::vector<int> seeds = regionQuery(points, idx);
    labels[idx] = clusterId;

    size_t i = 0;
    while (i < seeds.size()) {
        int current = seeds[i];

        if (!visited[current]) {
            visited[current] = true;
            std::vector<int> current_neighbors = regionQuery(points, current);

            if (current_neighbors.size() >= minPts) {
                // 合并当前点的邻居，不使用 insert 防止重复
                for (int n : current_neighbors) {
                    if (std::find(seeds.begin(), seeds.end(), n) == seeds.end()) {
                        seeds.push_back(n);
                    }
                }
            }
        }

        if (labels[current] == -1) {
            labels[current] = clusterId;
        }

        ++i;
    }
}

std::vector<int> DBSCAN::regionQuery(const std::vector<Eigen::Vector3f>& points, int idx) const {
    std::vector<int> result;

    const Eigen::Vector3f& query = points[idx];

    for (size_t i = 0; i < points.size(); ++i) {
        if ((points[i] - query).squaredNorm() <= epsSquared) {
            result.push_back(static_cast<int>(i));
        }
    }

    return result;
}
