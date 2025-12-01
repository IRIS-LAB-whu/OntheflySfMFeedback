#pragma once
#include "../Base/ply.h"
#include <unordered_set>
#include <memory>

/**
 * @brief Mesh操作类，提供mesh过滤和合并功能
 */
class MeshOperator {
public:
    /**
     * @brief 过滤配置参数
     */
    struct FilterConfig {
        double outlier_threshold_multiplier = 2.0;  // 粗差阈值倍数（标准差的倍数）
        int max_iterations = 5;                      // 最大迭代层数
        bool use_iqr_method = false;                 // 是否使用IQR方法进行粗差检测
        double iqr_multiplier = 1.5;                 // IQR方法的倍数
    };

    /**
     * @brief 对mesh进行过滤操作，剔除外层的粗差三角形
     * @param mesh 待过滤的mesh（原地修改）
     * @param config 过滤配置参数
     */
    static void FilterMesh(std::shared_ptr<PlyMesh> mesh, const FilterConfig& config = FilterConfig{});

    /**
     * @brief 合并两个mesh
     * @param mesh 主mesh（原地修改，作为合并结果）
     * @param mesh_block 辅助mesh
     * @param modified_points 需要更新的点集合
     */
    static void MergeMesh(std::shared_ptr<PlyMesh> mesh,
        const std::shared_ptr<PlyMesh> mesh_block,
        const std::unordered_set<point3D_t>& modified_points);

private:
    /**
     * @brief 计算三角形的边长
     * @param face 三角形面
     * @param vertices 顶点数组
     * @return 三条边的长度数组
     */
    static std::array<double, 3> CalculateEdgeLengths(const PlyMeshFace& face,
        const std::vector<PlyMeshVertex>& vertices);

    /**
     * @brief 计算三角形的最长边长度
     * @param face 三角形面
     * @param vertices 顶点数组
     * @return 最长边长度
     */
    static double CalculateMaxEdgeLength(const PlyMeshFace& face,
        const std::vector<PlyMeshVertex>& vertices);

    /**
     * @brief 统计所有三角形的最长边并计算粗差阈值
     * @param mesh mesh数据
     * @param config 配置参数
     * @return 粗差阈值
     */
    static double CalculateOutlierThreshold(const std::shared_ptr<PlyMesh> mesh,
        const FilterConfig& config);

    /**
     * @brief 找到mesh的最外层三角形索引
     * @param mesh mesh数据
     * @return 最外层三角形的索引集合
     */
    static std::unordered_set<size_t> FindBoundaryTriangles(const std::shared_ptr<PlyMesh> mesh);

    /**
     * @brief 从给定的三角形集合中过滤掉粗差三角形
     * @param mesh mesh数据
     * @param triangle_indices 待检查的三角形索引
     * @param threshold 粗差阈值
     * @return 需要移除的三角形索引集合
     */
    static std::unordered_set<size_t> FilterOutlierTriangles(const std::shared_ptr<PlyMesh> mesh,
        const std::unordered_set<size_t>& triangle_indices,
        double threshold);

    /**
     * @brief 移除指定的三角形
     * @param mesh mesh数据（原地修改）
     * @param triangles_to_remove 要移除的三角形索引集合
     */
    static void RemoveTriangles(std::shared_ptr<PlyMesh> mesh,
        const std::unordered_set<size_t>& triangles_to_remove);

    /**
     * @brief 清理未使用的顶点
     * @param mesh mesh数据（原地修改）
     */
    static void CleanupUnusedVertices(std::shared_ptr<PlyMesh> mesh);

    /**
     * @brief 计算两点间的距离
     * @param v1 顶点1
     * @param v2 顶点2
     * @return 距离
     */
    static double CalculateDistance(const PlyMeshVertex& v1, const PlyMeshVertex& v2);
};