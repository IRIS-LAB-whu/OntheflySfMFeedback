#include "mesh_operate.h"
#include <map>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <unordered_map>

void MeshOperator::FilterMesh(std::shared_ptr<PlyMesh> mesh, const FilterConfig& config) {
    if (!mesh || mesh->faces.empty()) {
        return;
    }

    // 计算粗差阈值
    double outlier_threshold = CalculateOutlierThreshold(mesh, config);

    // 迭代过滤
    for (int iteration = 0; iteration < config.max_iterations; ++iteration) {
        // 找到当前最外层的三角形
        auto boundary_triangles = FindBoundaryTriangles(mesh);

        if (boundary_triangles.empty()) {
            break; // 没有边界三角形，停止迭代
        }

        // 过滤粗差三角形
        auto outlier_triangles = FilterOutlierTriangles(mesh, boundary_triangles, outlier_threshold);

        if (outlier_triangles.empty()) {
            // 没有粗差三角形，移除当前最外层三角形
            RemoveTriangles(mesh, boundary_triangles);
        }
        else {
            // 移除粗差三角形
            RemoveTriangles(mesh, outlier_triangles);
        }

        // 如果没有面了，停止迭代
        if (mesh->faces.empty()) {
            break;
        }
    }

    // 清理未使用的顶点
    CleanupUnusedVertices(mesh);
}

void MeshOperator::MergeMesh(std::shared_ptr<PlyMesh> mesh,
    const std::shared_ptr<PlyMesh> mesh_block,
    const std::unordered_set<point3D_t>& modified_points) {
    if (!mesh || !mesh_block) {
        return;
    }

    // 创建顶点索引映射，用于合并时的索引转换
    std::unordered_map<size_t, size_t> vertex_index_map;
    size_t current_vertex_count = mesh->vertices.size();

    // 移除mesh中涉及modified_points的三角形
    std::vector<PlyMeshFace> filtered_faces;
    for (const auto& face : mesh->faces) {
        bool should_remove = false;

        // 检查三角形的三个顶点是否在modified_points中
        if (face.vertex_idx1 < mesh->vertices.size() &&
            modified_points.count(mesh->vertices[face.vertex_idx1].point3D_idx) > 0) {
            should_remove = true;
        }
        if (face.vertex_idx2 < mesh->vertices.size() &&
            modified_points.count(mesh->vertices[face.vertex_idx2].point3D_idx) > 0) {
            should_remove = true;
        }
        if (face.vertex_idx3 < mesh->vertices.size() &&
            modified_points.count(mesh->vertices[face.vertex_idx3].point3D_idx) > 0) {
            should_remove = true;
        }

        if (!should_remove) {
            filtered_faces.push_back(face);
        }
    }
    mesh->faces = std::move(filtered_faces);

    // 添加mesh_block中的顶点和面
    for (size_t i = 0; i < mesh_block->vertices.size(); ++i) {
        vertex_index_map[i] = current_vertex_count + i;
        mesh->vertices.push_back(mesh_block->vertices[i]);
    }

    // 添加mesh_block中的面，更新顶点索引
    for (const auto& face : mesh_block->faces) {
        PlyMeshFace new_face = face;
        new_face.vertex_idx1 = vertex_index_map[face.vertex_idx1];
        new_face.vertex_idx2 = vertex_index_map[face.vertex_idx2];
        new_face.vertex_idx3 = vertex_index_map[face.vertex_idx3];
        mesh->faces.push_back(new_face);
    }

    // 清理未使用的顶点
    CleanupUnusedVertices(mesh);
}

std::array<double, 3> MeshOperator::CalculateEdgeLengths(const PlyMeshFace& face,
    const std::vector<PlyMeshVertex>& vertices) {
    std::array<double, 3> lengths;

    // 边1: vertex_idx1 -> vertex_idx2
    lengths[0] = CalculateDistance(vertices[face.vertex_idx1], vertices[face.vertex_idx2]);

    // 边2: vertex_idx2 -> vertex_idx3
    lengths[1] = CalculateDistance(vertices[face.vertex_idx2], vertices[face.vertex_idx3]);

    // 边3: vertex_idx3 -> vertex_idx1
    lengths[2] = CalculateDistance(vertices[face.vertex_idx3], vertices[face.vertex_idx1]);

    return lengths;
}

double MeshOperator::CalculateMaxEdgeLength(const PlyMeshFace& face,
    const std::vector<PlyMeshVertex>& vertices) {
    auto lengths = CalculateEdgeLengths(face, vertices);
    return *std::max_element(lengths.begin(), lengths.end());
}

double MeshOperator::CalculateOutlierThreshold(const std::shared_ptr<PlyMesh> mesh,
    const FilterConfig& config) {
    if (mesh->faces.empty()) {
        return 0.0;
    }

    // 收集所有三角形的最长边长度
    std::vector<double> max_edge_lengths;
    max_edge_lengths.reserve(mesh->faces.size());

    for (const auto& face : mesh->faces) {
        double max_length = CalculateMaxEdgeLength(face, mesh->vertices);
        max_edge_lengths.push_back(max_length);
    }

    if (config.use_iqr_method) {
        // 使用IQR方法计算粗差阈值
        std::sort(max_edge_lengths.begin(), max_edge_lengths.end());

        size_t n = max_edge_lengths.size();
        double q1 = max_edge_lengths[n / 4];
        double q3 = max_edge_lengths[3 * n / 4];
        double iqr = q3 - q1;

        return q3 + config.iqr_multiplier * iqr;
    }
    else {
        // 使用标准差方法计算粗差阈值
        double mean = std::accumulate(max_edge_lengths.begin(), max_edge_lengths.end(), 0.0) / max_edge_lengths.size();

        double variance = 0.0;
        for (double length : max_edge_lengths) {
            variance += (length - mean) * (length - mean);
        }
        variance /= max_edge_lengths.size();
        double std_dev = std::sqrt(variance);

        return mean + config.outlier_threshold_multiplier * std_dev;
    }
}

std::unordered_set<size_t> MeshOperator::FindBoundaryTriangles(const std::shared_ptr<PlyMesh> mesh) {
    if (mesh->faces.empty()) {
        return {};
    }

    // 统计每条边被多少个三角形共享
    std::map<std::pair<size_t, size_t>, std::vector<size_t>> edge_to_faces;

    for (size_t i = 0; i < mesh->faces.size(); ++i) {
        const auto& face = mesh->faces[i];

        // 确保边的顶点按顺序排列（小的在前）
        std::vector<std::pair<size_t, size_t>> edges = {
            {std::min(face.vertex_idx1, face.vertex_idx2), std::max(face.vertex_idx1, face.vertex_idx2)},
            {std::min(face.vertex_idx2, face.vertex_idx3), std::max(face.vertex_idx2, face.vertex_idx3)},
            {std::min(face.vertex_idx3, face.vertex_idx1), std::max(face.vertex_idx3, face.vertex_idx1)}
        };

        for (const auto& edge : edges) {
            edge_to_faces[edge].push_back(i);
        }
    }

    // 找到边界三角形（包含只被一个三角形共享的边）
    std::unordered_set<size_t> boundary_triangles;
    for (const auto& [edge, face_indices] : edge_to_faces) {
        if (face_indices.size() == 1) {
            boundary_triangles.insert(face_indices[0]);
        }
    }

    return boundary_triangles;
}

std::unordered_set<size_t> MeshOperator::FilterOutlierTriangles(const std::shared_ptr<PlyMesh> mesh,
    const std::unordered_set<size_t>& triangle_indices,
    double threshold) {
    std::unordered_set<size_t> outlier_triangles;

    for (size_t face_idx : triangle_indices) {
        if (face_idx >= mesh->faces.size()) {
            continue;
        }

        double max_edge_length = CalculateMaxEdgeLength(mesh->faces[face_idx], mesh->vertices);

        if (max_edge_length > threshold) {
            outlier_triangles.insert(face_idx);
        }
    }

    return outlier_triangles;
}

void MeshOperator::RemoveTriangles(std::shared_ptr<PlyMesh> mesh,
    const std::unordered_set<size_t>& triangles_to_remove) {
    if (triangles_to_remove.empty()) {
        return;
    }

    std::vector<PlyMeshFace> new_faces;
    new_faces.reserve(mesh->faces.size() - triangles_to_remove.size());

    for (size_t i = 0; i < mesh->faces.size(); ++i) {
        if (triangles_to_remove.count(i) == 0) {
            new_faces.push_back(mesh->faces[i]);
        }
    }

    mesh->faces = std::move(new_faces);
}

void MeshOperator::CleanupUnusedVertices(std::shared_ptr<PlyMesh> mesh) {
    if (mesh->vertices.empty() || mesh->faces.empty()) {
        return;
    }

    // 找到所有被使用的顶点
    std::unordered_set<size_t> used_vertices;
    for (const auto& face : mesh->faces) {
        used_vertices.insert(face.vertex_idx1);
        used_vertices.insert(face.vertex_idx2);
        used_vertices.insert(face.vertex_idx3);
    }

    // 创建新的顶点数组和索引映射
    std::vector<PlyMeshVertex> new_vertices;
    std::unordered_map<size_t, size_t> vertex_index_map;

    size_t new_index = 0;
    for (size_t old_index = 0; old_index < mesh->vertices.size(); ++old_index) {
        if (used_vertices.count(old_index) > 0) {
            vertex_index_map[old_index] = new_index;
            new_vertices.push_back(mesh->vertices[old_index]);
            ++new_index;
        }
    }

    // 更新面的顶点索引
    for (auto& face : mesh->faces) {
        face.vertex_idx1 = vertex_index_map[face.vertex_idx1];
        face.vertex_idx2 = vertex_index_map[face.vertex_idx2];
        face.vertex_idx3 = vertex_index_map[face.vertex_idx3];
    }

    mesh->vertices = std::move(new_vertices);
}

double MeshOperator::CalculateDistance(const PlyMeshVertex& v1, const PlyMeshVertex& v2) {
    double dx = v1.x - v2.x;
    double dy = v1.y - v2.y;
    double dz = v1.z - v2.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}