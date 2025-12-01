#pragma once
#include <eigen3/Eigen/Dense>
#include <vector>
#include <cmath>
#include <stdexcept>

class DBSCAN {
public:
    DBSCAN(float eps, int minPts);
    std::vector<int> fit(const std::vector<Eigen::Vector3f>& points);

private:
    float eps;
    float epsSquared;
    int minPts;

    std::vector<bool> visited;
    std::vector<int> labels;

    std::vector<int> regionQuery(const std::vector<Eigen::Vector3f>& points, int idx) const;
    void expandCluster(const std::vector<Eigen::Vector3f>& points, int idx, int clusterId);
};
