#pragma once

#include "../Base/ply.h"
#include "glog/logging.h"
#include "Reconstruction.h"
#include "RenderData.h"
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Delaunay_triangulation_cell_base_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Triangulation_vertex_base_with_info_3.h>

#include <omp.h>
#include <tbb/global_control.h>
#include <unordered_map>
#include <unordered_set>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

typedef CGAL::Exact_predicates_inexact_constructions_kernel         K;
typedef CGAL::Triangulation_vertex_base_with_info_3<size_t, K>    Vb;
typedef CGAL::Delaunay_triangulation_cell_base_3<K>                 Cb;
typedef CGAL::Triangulation_data_structure_3<Vb, Cb>                Tds;
typedef CGAL::Delaunay_triangulation_3<K, Tds, CGAL::Fast_location> Delaunay;

typedef Delaunay::Vertex_handle Vertex_handle;
typedef Delaunay::Cell_handle Cell_handle;

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point;
typedef CGAL::Surface_mesh<Point> CMesh;

class StackTimer {
public:
	// 开始计时，传入描述字符串
	void start(const std::string& description) {
		size_t id = nextId++;
		descriptions[id] = description;
		startTime[id] = std::chrono::high_resolution_clock::now();
		activeIds.push(id);
	}

	// 停止最近启动的计时任务
	void stop() {
		if (activeIds.empty()) {
			throw std::runtime_error("No active timer to stop.");
		}
		size_t id = activeIds.top();
		activeIds.pop();

		auto endTime = std::chrono::high_resolution_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime[id]);
		executionTime[id] = duration.count();
		startTime.erase(id);
	}

	//  打印各任务计时结果
	void printExecutionTimes() const {
		LOG(INFO) << "--------------Execution Times (in milliseconds)--------------";
		for (const auto& [id, time] : executionTime) {
			LOG(INFO) << descriptions.at(id) << ": " << time << "ms";
		}
		//for (const auto& [id , time] : executionTime) {
		//    LOG(INFO) << descriptions.at(id) << ": " << time << " ms" << std::endl;
		//}
		LOG(INFO) << "---------------------Execution Times End---------------------";
	}

private:
	std::map<size_t, std::chrono::time_point<std::chrono::high_resolution_clock>> startTime;
	std::map<size_t, long long> executionTime;
	std::map<size_t, std::string> descriptions;
	std::stack<size_t> activeIds;
	size_t nextId = 0;
};

namespace {
	K::Point_3 EigenToCGAL(const Eigen::Vector3f& point) {
		return K::Point_3(point.x(), point.y(), point.z());
	}

	Eigen::Vector3f CGALToEigen(const K::Point_3& point) {
		return Eigen::Vector3f(point.x(), point.y(), point.z());
	}

	struct TrackPoint2D {
		Eigen::Vector2f position = Eigen::Vector2f::Zero();
		image_t image_id = 0;
		uint32_t point2d_idx = 0;
	};
}

struct DelaunayMeshingOptions {
	// Unify input points into one cell in the Delaunay triangulation that fall
	// within a reprojected radius of the given pixels.
	double max_proj_dist = 20.0;

	// Maximum relative depth difference between input point and a vertex of an
	// existing cell in the Delaunay triangulation, otherwise a new vertex is
	// created in the triangulation.
	double max_depth_dist = 0.05;

	// The standard deviation of wrt. the number of images seen by each point.
	// Increasing this value decreases the influence of points seen in few images.
	double visibility_sigma = 3.0;

	// The factor that is applied to the computed distance sigma, which is
	// automatically computed as the 25th percentile of edge lengths. A higher
	// value will increase the smoothness of the surface.
	double distance_sigma_factor = 1.0;

	// A higher quality regularization leads to a smoother surface.
	double quality_regularization = 1.0;

	// Filtering thresholds for outlier surface mesh faces. If the longest side of
	// a mesh face (longest out of 3) exceeds the side lengths of all faces at a
	// certain percentile by the given factor, then it is considered an outlier
	// mesh face and discarded.
	double max_side_length_factor = 25.0;
	double max_side_length_percentile = 95.0;

	// The number of threads to use for reconstruction. Default is all threads.
	int num_threads = -1;
};

namespace std {

	template <>
	struct hash<Vertex_handle> {
		size_t operator()(const Vertex_handle& handle) const {
			return reinterpret_cast<size_t>(&*handle);
		}
	};

	template <>
	struct hash<const Vertex_handle> {
		size_t operator()(const Vertex_handle& handle) const {
			return reinterpret_cast<size_t>(&*handle);
		}
	};

	template <>
	struct hash<Cell_handle> {
		size_t operator()(const Cell_handle& handle) const {
			return reinterpret_cast<size_t>(&*handle);
		}
	};

	template <>
	struct hash<const Cell_handle> {
		size_t operator()(const Cell_handle& handle) const {
			return reinterpret_cast<size_t>(&*handle);
		}
	};
}

template <typename T>
class KeyTracker {
private:
	std::unordered_set<T> current_keys_;
	std::vector<T> added_keys_;
	std::vector<T> removed_keys_;
	bool initialized_ = false;

public:
	KeyTracker() = default;

	template <typename V>
	explicit KeyTracker(const std::unordered_map<T, V>& initial) {
		init(initial);
	}

	template <typename V>
	void init(const std::unordered_map<T, V>& initial) {
		current_keys_.clear();
		added_keys_.clear();
		removed_keys_.clear();
		current_keys_.reserve(initial.size());
		for (const auto& [key, _] : initial) {
			current_keys_.insert(key);
		}
		added_keys_.reserve(initial.size() / 10);
		removed_keys_.reserve(initial.size() / 10);
		initialized_ = true;
	}
	const std::vector<T>& get_added_keys() const { return added_keys_; }
	const std::vector<T>& get_removed_keys() const { return removed_keys_; }
	void clear_changes() { added_keys_.clear(); removed_keys_.clear(); }

	template <typename V>
	void track_changes(const std::unordered_map<T, V>& new_data) {
		if (!initialized_) throw std::runtime_error("KeyTracker not initialized");

		std::vector<T> new_added;
		new_added.reserve(new_data.size() / 10);
		std::vector<T> new_keys;
		new_keys.reserve(new_data.size());

		for (const auto& [key, _] : new_data) {
			new_keys.push_back(key);
		}

#pragma omp parallel for
		for (int i = 0; i < new_keys.size(); ++i) {
			const auto& key = new_keys[i];
			if (!current_keys_.count(key)) {
#pragma omp critical
				new_added.push_back(key);
			}
		}

		std::vector<T> new_removed;
		new_removed.reserve(current_keys_.size() / 10);
		std::vector<T> current_keys_vec(current_keys_.begin(), current_keys_.end());

#pragma omp parallel for
		for (int i = 0; i < current_keys_vec.size(); ++i) {
			const auto& key = current_keys_vec[i];
			if (!new_data.count(key)) {
#pragma omp critical
				new_removed.push_back(key);
			}
		}

		added_keys_ = std::move(new_added);
		removed_keys_ = std::move(new_removed);

		current_keys_.clear();
		current_keys_.reserve(new_data.size());
		for (const auto& [key, _] : new_data) {
			current_keys_.insert(key);
		}
	}

	inline bool has_key(const T& key) {
		return current_keys_.find(key) != current_keys_.end();
	}
};

class DelaunayMeshingInput {
public:
	struct Image {
		camera_t camera_id = kInvalidCameraId;
		Eigen::Matrix3x4f proj_matrix = Eigen::Matrix3x4f::Identity();
		Eigen::Vector3f proj_center = Eigen::Vector3f::Zero();
		std::vector<point3D_t> point_ids;
	};

	struct Point {
		Eigen::Vector3f position = Eigen::Vector3f::Zero();
		uint32_t num_visible_images = 0;
		double pro_error = 0.0;
		std::vector<image_t> visible_images_ids;
		std::vector<TrackPoint2D> point2ds;
	};

	std::unordered_map<camera_t, Camera> cameras;
	std::unordered_map<image_t, Image> images; // Current model images
	std::unordered_map<point3D_t, Point> points; // Current model points
	KeyTracker<point3D_t> pointsTracker;
	KeyTracker<image_t> imagesTracker;
	KeyTracker<point3D_t> delaunayVerticesTracker; // Track Delaunay modified vertices

	std::unordered_set<image_t> modified_images; // images to generate mesh
	std::unordered_set<point3D_t> modified_points; // points to genertate mesh

	void ReadSparseReconstruction(const std::string& path);
	void CopyFromSparseReconstruction(const Reconstruction& reconstruction);

	void UpdateFromSparseReconstruction(const Reconstruction& reconstruction);

	Delaunay CreateDelaunayTriangulation() const;

	void CreateSubSampledDelaunayTriangulation(
		const float max_proj_dist, const float max_depth_dist, Delaunay& triangulation);

	void ClearCaches();
};

struct DelaunayMeshingEdgeWeightComputer {
	DelaunayMeshingEdgeWeightComputer(const Delaunay& triangulation,
		const double visibility_sigma,
		const double distance_sigma_factor)
		: visibility_threshold_(5 * visibility_sigma),
		visibility_normalization_(-0.5 /
			(visibility_sigma * visibility_sigma)) {
		std::vector<float> edge_lengths;
		edge_lengths.reserve(triangulation.number_of_finite_edges());

		for (auto it = triangulation.finite_edges_begin();
			it != triangulation.finite_edges_end();
			++it) {
			edge_lengths.push_back((it->first->vertex(it->second)->point() -
				it->first->vertex(it->third)->point())
				.squared_length());
		}

		distance_sigma_ = distance_sigma_factor *
			std::max(std::sqrt(Percentile(edge_lengths, 25.0)), 1e-7f);
		distance_threshold_ = 5 * distance_sigma_;
		distance_normalization_ = -0.5 / (distance_sigma_ * distance_sigma_);
	}

	double DistanceSigma() const { return distance_sigma_; }

	double ComputeVisibilityProb(const double visibility_squared) const {
		if (visibility_squared < visibility_threshold_) {
			return std::max(
				0.0, 1.0 - std::exp(visibility_squared * visibility_normalization_));
		}
		else {
			return 1.0;
		}
	}

	double ComputeDistanceProb(const double distance_squared) const {
		if (distance_squared < distance_threshold_) {
			return std::max(
				0.0, 1.0 - std::exp(distance_squared * distance_normalization_));
		}
		else {
			return 1.0;
		}
	}

private:
	double visibility_threshold_;
	double visibility_normalization_;
	double distance_sigma_;
	double distance_threshold_;
	double distance_normalization_;
};


// Ray caster through the cells of a Delaunay triangulation. The tracing locates
// the cell of the ray origin and then iteratively intersects the ray with all
// facets of the current cell and advances to the neighboring cell of the
// intersected facet. Note that the ray can also pass through outside of the
// hull of the triangulation, i.e. lie within the infinite cells/facets.
// The ray caster collects the intersected facets along the ray.
struct DelaunayTriangulationRayCaster {
	struct Intersection {
		Delaunay::Facet facet;
		double target_distance_squared = 0.0;
	};

	explicit DelaunayTriangulationRayCaster(const Delaunay& triangulation)
		: triangulation_(triangulation) {
		FindHullFacets();
	}

	void CastRaySegment(const K::Segment_3& ray_segment,
		std::vector<Intersection>* intersections) const {
		intersections->clear();

		Cell_handle next_cell =
			triangulation_.locate(ray_segment.start());

		bool next_cell_found = true;
		while (next_cell_found) {
			next_cell_found = false;

			if (triangulation_.is_infinite(next_cell)) {
				// Linearly check all hull facets for intersection.

				for (const auto& hull_facet : hull_facets_) {
					// Check if the ray origin is infront of the facet.
					const K::Triangle_3 triangle = triangulation_.triangle(hull_facet);
					if (CGAL::orientation(
						triangle[0], triangle[1], triangle[2], ray_segment.start()) ==
						K::Orientation::NEGATIVE) {
						continue;
					}

					// Check if the segment intersects the facet.
					K::Point_3 intersection_point;
					if (!CGAL::assign(intersection_point,
						CGAL::intersection(ray_segment, triangle))) {
						continue;
					}

					// Make sure the next intersection is closer to target than previous.
					const double target_distance_squared =
						(intersection_point - ray_segment.end()).squared_length();
					if (!intersections->empty() &&
						intersections->back().target_distance_squared <
						target_distance_squared) {
						continue;
					}

					Intersection intersection;
					intersection.facet =
						Delaunay::Facet(hull_facet.first, hull_facet.second);
					intersection.target_distance_squared = target_distance_squared;
					intersections->push_back(intersection);

					next_cell = hull_facet.first->neighbor(hull_facet.second);
					next_cell_found = true;

					break;
				}
			}
			else {
				// Check all neighboring finite facets for intersection.

				for (int i = 0; i < 4; ++i) {
					// Check if the ray origin is infront of the facet.
					const K::Triangle_3 triangle = triangulation_.triangle(next_cell, i);
					if (CGAL::orientation(
						triangle[0], triangle[1], triangle[2], ray_segment.start()) ==
						K::Orientation::NEGATIVE) {
						continue;
					}

					// Check if the segment intersects the facet.
					K::Point_3 intersection_point;
					if (!CGAL::assign(intersection_point,
						CGAL::intersection(ray_segment, triangle))) {
						continue;
					}

					// Make sure the next intersection is closer to target than previous.
					const double target_distance_squared =
						(intersection_point - ray_segment.end()).squared_length();
					if (!intersections->empty() &&
						intersections->back().target_distance_squared <
						target_distance_squared) {
						continue;
					}

					Intersection intersection;
					intersection.facet = Delaunay::Facet(next_cell, i);
					intersection.target_distance_squared = target_distance_squared;
					intersections->push_back(intersection);

					next_cell = next_cell->neighbor(i);
					next_cell_found = true;

					break;
				}
			}
		}
	}

private:
	// Find all finite facets of infinite cells.
	void FindHullFacets() {
		for (auto it = triangulation_.all_cells_begin();
			it != triangulation_.all_cells_end();
			++it) {
			if (triangulation_.is_infinite(it)) {
				for (int i = 0; i < 4; ++i) {
					if (!triangulation_.is_infinite(it, i)) {
						hull_facets_.emplace_back(it, i);
					}
				}
			}
		}
	}

	const Delaunay& triangulation_;
	std::vector<Delaunay::Facet> hull_facets_;
};

struct DelaunayCellData {
	DelaunayCellData() : DelaunayCellData(-1) {}
	explicit DelaunayCellData(const int index)
		: index(index),
		source_weight(0),
		sink_weight(0),
		edge_weights({ {0, 0, 0, 0} }) {}
	int index;
	float source_weight;
	float sink_weight;
	std::array<float, 4> edge_weights;
};

typedef std::unordered_map<const Delaunay::Cell_handle, DelaunayCellData> CellGraphData;

class MeshManager {
public:
	DelaunayMeshingInput input_data;
	DelaunayMeshingOptions options;
	Delaunay triangulation;
	DelaunayTriangulationRayCaster* ray_caster = nullptr;
	DelaunayMeshingEdgeWeightComputer* edge_weight_computer = nullptr;
	std::shared_ptr<PlyMesh> mesh;
	std::vector<float> surface_facet_areas; // 存储面片面积
	std::vector<Delaunay::Facet> surface_facets;
	std::vector<float> surface_facet_side_lengths;
	StackTimer timer;

	MeshManager() {
		input_data = DelaunayMeshingInput();
		options = DelaunayMeshingOptions();
		mesh = std::make_shared<PlyMesh>();
		options.num_threads = 6;
	}

	std::shared_ptr<PlyMesh> GetMeshPtr() const { return mesh; }
	std::shared_ptr<RenderData> GetRenderingData() const { return renderdata; }

	void CopyFromReconstruction(const Reconstruction& reconstruction);

	void CreateMesh();
	void UpdateMesh();
	void MergeMesh(std::shared_ptr<PlyMesh> meshblock);
	void CreateMeshSimulatly(const Reconstruction& reconstruction);
	std::shared_ptr<PlyMesh> FilterSurfaceFacets(
		const std::unordered_set<Delaunay::Vertex_handle>& surface_vertices,
		const std::vector<Delaunay::Facet>& surface_facets,
		const std::vector<float>& surface_facet_side_lengths);

	// 边界区域自适应过滤函数
	std::shared_ptr<PlyMesh> FilterSurfaceFacetsWithBoundaryAdaptation(
		const std::unordered_set<Delaunay::Vertex_handle>& surface_vertices,
		const std::vector<Delaunay::Facet>& surface_facets,
		const std::vector<float>& surface_facet_side_lengths);

	std::vector<double> QualityAssessment();

	void GenerateRenderMeshData();

	void WriteDelaunayTriangulationPly(const std::string& path,
		const Delaunay& triangulation);

private:
	std::shared_ptr<RenderData> renderdata;

	// Implementation of geometry visualized in Figure 9 in P. Labatut, J‐P. Pons,
	// and R. Keriven. "Robust and efficient surface reconstruction from range
	// data." Computer graphics forum, 2009.
	double ComputeCosFacetCellAngle(const Delaunay::Facet& facet) const {
		if (triangulation.is_infinite(facet.first)) {
			return 1.0;
		}

		const K::Triangle_3 triangle = triangulation.triangle(facet);

		const K::Vector_3 facet_normal =
			CGAL::cross_product(triangle[1] - triangle[0], triangle[2] - triangle[0]);
		const double facet_normal_length_squared = facet_normal.squared_length();
		if (facet_normal_length_squared == 0.0) {
			return 0.5;
		}

		const K::Vector_3 co_tangent = facet.first->circumcenter() - triangle[0];
		const float co_tangent_length_squared = co_tangent.squared_length();
		if (co_tangent_length_squared == 0.0) {
			return 0.5;
		}

		return (facet_normal * co_tangent) /
			std::sqrt(facet_normal_length_squared * co_tangent_length_squared);
	}
};


// 异常边长检测配置结构体
struct AdaptiveFilterConfig {
	enum class FilterMethod {
		FIXED_THRESHOLD,           // 固定阈值
		IQR_OUTLIER,              // 基于四分位距的异常检测
		ZSCORE_OUTLIER,           // 基于Z-Score的异常检测
		PERCENTILE_BASED,         // 基于百分位数
		DYNAMIC_CLUSTERING,       // 基于聚类的动态检测
		COMBINED_METHOD           // 组合方法
	};

	FilterMethod method = FilterMethod::DYNAMIC_CLUSTERING;

	// ============= 各种过滤方法的参数 =============

	// 固定阈值参数
	struct FixedThresholdParams {
		double threshold = 10.0;
	} fixed_threshold_params;

	// IQR方法参数
	struct IQRParams {
		double multiplier = 1.5;  // 标准为1.5，可调整为2.0或3.0
	} iqr_params;

	// Z-Score方法参数
	struct ZScoreParams {
		double threshold = 3.0;  // 标准为3.0
	} zscore_params;

	// 百分位数方法参数
	struct PercentileParams {
		double threshold = 95.0;  // 保留95%的数据
	} percentile_params;

	// DBSCAN 聚类参数
	struct DBSCANParams {
		double eps_factor = 0.08;               // eps = eps_factor * IQR
		double min_eps = 0.03;                  // 最小 eps 值
		double min_pts_ratio = 0.015;           // min_pts = data_size * min_pts_ratio
		int min_min_pts = 8;                    // 最小 min_pts
		int max_min_pts = 80;                   // 最大 min_pts
		double distance_threshold_factor = 2.0; // 距离主簇的距离阈值 = factor * std_dev
		double extreme_percentile = 99.0;       // 极端值百分位数阈值
		double main_cluster_std_factor = 2.0;   // 主簇标准差倍数（2σ原则）
	} dbscan_params;

	// 组合方法权重
	struct CombinedWeights {
		double iqr_weight = 0.4;
		double zscore_weight = 0.3;
		double percentile_weight = 0.3;
		double combination_threshold = 0.5;  // 组合得分阈值
	} combined_weights;

	// 安全参数：即使检测为异常，也保留一定比例的面片
	double min_retention_ratio = 0.85;  // 至少保留85%的面片

	// ============= 预设配置静态方法 =============

	static AdaptiveFilterConfig GetDefaultConfig() {
		return AdaptiveFilterConfig();  // 使用默认初始化值
	}

	static AdaptiveFilterConfig GetStrictDBSCANConfig() {
		AdaptiveFilterConfig config;
		config.method = FilterMethod::DYNAMIC_CLUSTERING;
		config.min_retention_ratio = 0.80;

		// 严格的DBSCAN参数
		config.dbscan_params.eps_factor = 0.06;
		config.dbscan_params.min_eps = 0.02;
		config.dbscan_params.min_pts_ratio = 0.02;
		config.dbscan_params.min_min_pts = 10;
		config.dbscan_params.max_min_pts = 100;
		config.dbscan_params.distance_threshold_factor = 1.5;
		config.dbscan_params.extreme_percentile = 96.0;
		config.dbscan_params.main_cluster_std_factor = 1.8;

		return config;
	}

	static AdaptiveFilterConfig GetLooseDBSCANConfig() {
		AdaptiveFilterConfig config;
		config.method = FilterMethod::DYNAMIC_CLUSTERING;
		config.min_retention_ratio = 0.95;

		// 宽松的DBSCAN参数
		config.dbscan_params.eps_factor = 0.15;
		config.dbscan_params.min_eps = 0.08;
		config.dbscan_params.min_pts_ratio = 0.008;
		config.dbscan_params.min_min_pts = 5;
		config.dbscan_params.max_min_pts = 50;
		config.dbscan_params.distance_threshold_factor = 3.5;
		config.dbscan_params.extreme_percentile = 99.5;
		config.dbscan_params.main_cluster_std_factor = 3.0;

		return config;
	}

	static AdaptiveFilterConfig GetCombinedConfig() {
		AdaptiveFilterConfig config;
		config.method = FilterMethod::COMBINED_METHOD;
		config.min_retention_ratio = 0.90;

		// 调整各种方法的参数
		config.fixed_threshold_params.threshold = 10.0;
		config.iqr_params.multiplier = 2.0;
		config.zscore_params.threshold = 3.0;
		config.percentile_params.threshold = 96.0;

		config.combined_weights.iqr_weight = 0.3;
		config.combined_weights.zscore_weight = 0.3;
		config.combined_weights.percentile_weight = 0.4;
		config.combined_weights.combination_threshold = 0.6;

		return config;
	}
};



// 异常边长检测类
class AdaptiveEdgeLengthFilter {
private:
	AdaptiveFilterConfig config_;

	// 一维 DBSCAN 实现
	class DBSCAN1D {
	private:
		double eps_;
		int min_pts_;

	public:
		DBSCAN1D(double eps, int min_pts) : eps_(eps), min_pts_(min_pts) {}

		std::vector<int> cluster(const std::vector<double>& data) {
			if (data.empty()) return {};

			size_t n = data.size();
			std::vector<int> labels(n, -1);  // -1 表示噪声点
			std::vector<bool> visited(n, false);

			// 创建索引数组用于排序
			std::vector<size_t> indices(n);
			std::iota(indices.begin(), indices.end(), 0);

			// 按数据值排序，但保留原始索引
			std::sort(indices.begin(), indices.end(),
				[&data](size_t a, size_t b) { return data[a] < data[b]; });

			int cluster_id = 0;

			for (size_t i = 0; i < n; ++i) {
				size_t idx = indices[i];
				if (visited[idx]) continue;

				visited[idx] = true;
				std::vector<size_t> neighbors = getNeighbors(data, indices, i, n);

				if (neighbors.size() < static_cast<size_t>(min_pts_)) {
					labels[idx] = -1;  // 噪声点
				}
				else {
					expandCluster(data, indices, idx, cluster_id, labels, visited, n);
					cluster_id++;
				}
			}

			return labels;
		}

	private:
		std::vector<size_t> getNeighbors(const std::vector<double>& data,
			const std::vector<size_t>& indices,
			size_t center_pos, size_t n) {
			std::vector<size_t> neighbors;
			size_t center_idx = indices[center_pos];
			double center_val = data[center_idx];

			// 向左搜索
			for (int i = static_cast<int>(center_pos) - 1; i >= 0; --i) {
				size_t idx = indices[i];
				if (center_val - data[idx] <= eps_) {
					neighbors.push_back(idx);
				}
				else {
					break;  // 由于已排序，可以提前退出
				}
			}

			// 添加中心点
			neighbors.push_back(center_idx);

			// 向右搜索
			for (size_t i = center_pos + 1; i < n; ++i) {
				size_t idx = indices[i];
				if (data[idx] - center_val <= eps_) {
					neighbors.push_back(idx);
				}
				else {
					break;  // 由于已排序，可以提前退出
				}
			}

			return neighbors;
		}

		void expandCluster(const std::vector<double>& data,
			const std::vector<size_t>& indices,
			size_t start_idx, int cluster_id,
			std::vector<int>& labels,
			std::vector<bool>& visited,
			size_t n) {

			std::queue<size_t> seeds;
			labels[start_idx] = cluster_id;

			// 找到 start_idx 在排序数组中的位置
			size_t start_pos = 0;
			for (size_t i = 0; i < n; ++i) {
				if (indices[i] == start_idx) {
					start_pos = i;
					break;
				}
			}

			std::vector<size_t> neighbors = getNeighbors(data, indices, start_pos, n);
			for (size_t neighbor : neighbors) {
				if (neighbor != start_idx) {
					seeds.push(neighbor);
				}
			}

			while (!seeds.empty()) {
				size_t current = seeds.front();
				seeds.pop();

				if (!visited[current]) {
					visited[current] = true;

					// 找到 current 在排序数组中的位置
					size_t current_pos = 0;
					for (size_t i = 0; i < n; ++i) {
						if (indices[i] == current) {
							current_pos = i;
							break;
						}
					}

					std::vector<size_t> current_neighbors = getNeighbors(data, indices, current_pos, n);

					if (current_neighbors.size() >= static_cast<size_t>(min_pts_)) {
						for (size_t new_neighbor : current_neighbors) {
							if (!visited[new_neighbor]) {
								seeds.push(new_neighbor);
							}
						}
					}
				}

				if (labels[current] == -1) {
					labels[current] = cluster_id;
				}
			}
		}
	};

	// 计算统计量
	struct Statistics {
		double mean = 0.0;
		double median = 0.0;
		double std_dev = 0.0;
		double q1 = 0.0;
		double q3 = 0.0;
		double iqr = 0.0;
		double min_val = 0.0;
		double max_val = 0.0;
		std::vector<double> percentiles;
	};

	Statistics computeStatistics(const std::vector<float>& edge_lengths) {
		Statistics stats;
		if (edge_lengths.empty()) return stats;

		std::vector<double> sorted_lengths(edge_lengths.begin(), edge_lengths.end());
		std::sort(sorted_lengths.begin(), sorted_lengths.end());

		size_t n = sorted_lengths.size();

		// 基本统计量
		stats.min_val = sorted_lengths.front();
		stats.max_val = sorted_lengths.back();

		// 中位数
		if (n % 2 == 0) {
			stats.median = (sorted_lengths[n / 2 - 1] + sorted_lengths[n / 2]) / 2.0;
		}
		else {
			stats.median = sorted_lengths[n / 2];
		}

		// 四分位数
		size_t q1_idx = n / 4;
		size_t q3_idx = 3 * n / 4;
		stats.q1 = sorted_lengths[q1_idx];
		stats.q3 = sorted_lengths[q3_idx];
		stats.iqr = stats.q3 - stats.q1;

		// 均值和标准差
		double sum = std::accumulate(sorted_lengths.begin(), sorted_lengths.end(), 0.0);
		stats.mean = sum / n;

		double sq_sum = 0.0;
		for (double val : sorted_lengths) {
			sq_sum += (val - stats.mean) * (val - stats.mean);
		}
		stats.std_dev = std::sqrt(sq_sum / n);

		// 百分位数
		stats.percentiles.resize(101);
		for (int p = 0; p <= 100; ++p) {
			size_t idx = static_cast<size_t>(p * (n - 1) / 100.0);
			stats.percentiles[p] = sorted_lengths[idx];
		}

		return stats;
	}

	std::vector<bool> detectByFixedThreshold(const std::vector<float>& edge_lengths, const Statistics& stats) {
		std::vector<bool> is_outlier(edge_lengths.size(), false);
		double threshold = config_.fixed_threshold_params.threshold;

		for (size_t i = 0; i < edge_lengths.size(); ++i) {
			is_outlier[i] = edge_lengths[i] > threshold;
		}

		LOG(INFO) << "Fixed threshold method: threshold=" << threshold;
		return applySafetyConstraint(is_outlier, edge_lengths);
	}

	std::vector<bool> detectByIQR(const std::vector<float>& edge_lengths, const Statistics& stats) {
		double upper_bound = stats.q3 + config_.iqr_params.multiplier * stats.iqr;
		std::vector<bool> is_outlier(edge_lengths.size(), false);

		for (size_t i = 0; i < edge_lengths.size(); ++i) {
			is_outlier[i] = edge_lengths[i] > upper_bound;
		}

		LOG(INFO) << "IQR method: Q3=" << stats.q3 << ", IQR=" << stats.iqr
			<< ", Multiplier=" << config_.iqr_params.multiplier
			<< ", Upper bound=" << upper_bound;

		return applySafetyConstraint(is_outlier, edge_lengths);
	}

	std::vector<bool> detectByZScore(const std::vector<float>& edge_lengths, const Statistics& stats) {
		double threshold = config_.zscore_params.threshold;
		std::vector<bool> is_outlier(edge_lengths.size(), false);

		for (size_t i = 0; i < edge_lengths.size(); ++i) {
			double z_score = std::abs(edge_lengths[i] - stats.mean) / stats.std_dev;
			is_outlier[i] = z_score > threshold;
		}

		LOG(INFO) << "Z-Score method: Mean=" << stats.mean << ", Std=" << stats.std_dev
			<< ", Threshold=" << threshold;

		return applySafetyConstraint(is_outlier, edge_lengths);
	}

	std::vector<bool> detectByPercentile(const std::vector<float>& edge_lengths, const Statistics& stats) {
		int percentile_idx = static_cast<int>(config_.percentile_params.threshold);
		if (percentile_idx < 0 || percentile_idx >= static_cast<int>(stats.percentiles.size())) {
			percentile_idx = 95; // 默认值
		}
		double threshold = stats.percentiles[percentile_idx];

		std::vector<bool> is_outlier(edge_lengths.size(), false);
		for (size_t i = 0; i < edge_lengths.size(); ++i) {
			is_outlier[i] = edge_lengths[i] > threshold;
		}

		LOG(INFO) << "Percentile method: " << config_.percentile_params.threshold
			<< "% threshold=" << threshold;

		return applySafetyConstraint(is_outlier, edge_lengths);
	}

	std::vector<bool> detectByClustering(const std::vector<float>& edge_lengths, const Statistics& stats) {
		if (edge_lengths.empty()) {
			return std::vector<bool>();
		}

		// 使用配置的 DBSCAN 参数
		const auto& dbscan_cfg = config_.dbscan_params;

		std::vector<double> double_lengths(edge_lengths.begin(), edge_lengths.end());

		// 动态计算 DBSCAN 参数
		double adaptive_eps = std::max(dbscan_cfg.eps_factor * stats.iqr, dbscan_cfg.min_eps);
		int adaptive_min_pts = std::max(static_cast<int>(edge_lengths.size() * dbscan_cfg.min_pts_ratio),
			dbscan_cfg.min_min_pts);
		adaptive_min_pts = std::min(adaptive_min_pts, dbscan_cfg.max_min_pts);

		DBSCAN1D dbscan(adaptive_eps, adaptive_min_pts);
		std::vector<int> cluster_labels = dbscan.cluster(double_lengths);

		// 分析聚类结果
		std::map<int, std::vector<size_t>> clusters;
		std::vector<size_t> noise_points;

		for (size_t i = 0; i < cluster_labels.size(); ++i) {
			if (cluster_labels[i] == -1) {
				noise_points.push_back(i);
			}
			else {
				clusters[cluster_labels[i]].push_back(i);
			}
		}

		// 找到主要簇
		int main_cluster_id = -1;
		size_t max_cluster_size = 0;
		double main_cluster_center = 0.0;

		for (const auto& [cluster_id, indices] : clusters) {
			if (indices.size() > max_cluster_size) {
				max_cluster_size = indices.size();
				main_cluster_id = cluster_id;

				double sum = 0.0;
				for (size_t idx : indices) {
					sum += edge_lengths[idx];
				}
				main_cluster_center = sum / indices.size();
			}
		}

		std::vector<bool> is_outlier(edge_lengths.size(), false);

		if (main_cluster_id != -1) {
			// 策略1: 基于与主簇的距离
			double distance_threshold = dbscan_cfg.distance_threshold_factor * stats.std_dev;

			for (size_t i = 0; i < edge_lengths.size(); ++i) {
				if (cluster_labels[i] != main_cluster_id) {
					double distance_to_main = std::abs(edge_lengths[i] - main_cluster_center);
					if (distance_to_main > distance_threshold || cluster_labels[i] == -1) {
						is_outlier[i] = true;
					}
				}
			}

			// 策略2: 极端值过滤
			if (!stats.percentiles.empty()) {
				int extreme_idx = static_cast<int>(dbscan_cfg.extreme_percentile);
				if (extreme_idx >= 0 && extreme_idx < static_cast<int>(stats.percentiles.size())) {
					double extreme_threshold = stats.percentiles[extreme_idx];
					for (size_t i = 0; i < edge_lengths.size(); ++i) {
						if (edge_lengths[i] > extreme_threshold) {
							is_outlier[i] = true;
						}
					}
				}
			}

			// 策略3: 基于主簇统计特征的过滤
			if (max_cluster_size > 0) {
				std::vector<double> main_cluster_values;
				main_cluster_values.reserve(max_cluster_size);
				for (size_t i = 0; i < edge_lengths.size(); ++i) {
					if (cluster_labels[i] == main_cluster_id) {
						main_cluster_values.push_back(edge_lengths[i]);
					}
				}

				double main_cluster_mean = main_cluster_center;
				double main_cluster_sq_sum = 0.0;
				for (double val : main_cluster_values) {
					main_cluster_sq_sum += (val - main_cluster_mean) * (val - main_cluster_mean);
				}
				double main_cluster_std = std::sqrt(main_cluster_sq_sum / main_cluster_values.size());

				double strict_threshold = main_cluster_mean + dbscan_cfg.main_cluster_std_factor * main_cluster_std;

				for (size_t i = 0; i < edge_lengths.size(); ++i) {
					if (edge_lengths[i] > strict_threshold) {
						is_outlier[i] = true;
					}
				}
			}

		}
		else {
			// 回退策略
			double threshold = stats.q3 + 1.0 * stats.iqr;
			for (size_t i = 0; i < edge_lengths.size(); ++i) {
				is_outlier[i] = edge_lengths[i] > threshold;
			}
		}

		size_t outlier_count = std::count(is_outlier.begin(), is_outlier.end(), true);
		size_t noise_count = noise_points.size();

		LOG(INFO) << "DBSCAN Clustering method:";
		LOG(INFO) << "  - eps_factor: " << dbscan_cfg.eps_factor << " -> eps: " << adaptive_eps;
		LOG(INFO) << "  - min_pts_ratio: " << dbscan_cfg.min_pts_ratio << " -> min_pts: " << adaptive_min_pts;
		LOG(INFO) << "  - distance_threshold_factor: " << dbscan_cfg.distance_threshold_factor;
		LOG(INFO) << "  - extreme_percentile: " << dbscan_cfg.extreme_percentile;
		LOG(INFO) << "  - main_cluster_std_factor: " << dbscan_cfg.main_cluster_std_factor;
		LOG(INFO) << "  - Found " << clusters.size() << " clusters";
		LOG(INFO) << "  - Main cluster size: " << max_cluster_size << "/" << edge_lengths.size();
		LOG(INFO) << "  - Noise points: " << noise_count;
		LOG(INFO) << "  - Final outliers: " << outlier_count;

		return applySafetyConstraint(is_outlier, edge_lengths);
	}

	std::vector<bool> detectByCombinedMethod(const std::vector<float>& edge_lengths, const Statistics& stats) {
		auto iqr_result = detectByIQR(edge_lengths, stats);
		auto zscore_result = detectByZScore(edge_lengths, stats);
		auto percentile_result = detectByPercentile(edge_lengths, stats);

		const auto& weights = config_.combined_weights;
		std::vector<bool> is_outlier(edge_lengths.size(), false);
		std::vector<double> outlier_scores(edge_lengths.size(), 0.0);

		for (size_t i = 0; i < edge_lengths.size(); ++i) {
			if (iqr_result[i]) outlier_scores[i] += weights.iqr_weight;
			if (zscore_result[i]) outlier_scores[i] += weights.zscore_weight;
			if (percentile_result[i]) outlier_scores[i] += weights.percentile_weight;

			is_outlier[i] = outlier_scores[i] > weights.combination_threshold;
		}

		size_t outlier_count = std::count(is_outlier.begin(), is_outlier.end(), true);
		LOG(INFO) << "Combined method: threshold=" << weights.combination_threshold
			<< ", detected " << outlier_count << " outliers";

		return applySafetyConstraint(is_outlier, edge_lengths);
	}

	// 修复的安全约束函数，现在接受边长参数
	std::vector<bool> applySafetyConstraint(const std::vector<bool>& is_outlier, const std::vector<float>& edge_lengths) {
		size_t outlier_count = std::count(is_outlier.begin(), is_outlier.end(), true);
		size_t max_allowed_outliers = static_cast<size_t>(
			is_outlier.size() * (1.0 - config_.min_retention_ratio));

		if (outlier_count <= max_allowed_outliers) {
			return is_outlier;
		}

		// 如果异常值太多，选择最异常的部分
		std::vector<std::pair<float, size_t>> indexed_lengths;
		indexed_lengths.reserve(outlier_count);

		for (size_t i = 0; i < is_outlier.size(); ++i) {
			if (is_outlier[i]) {
				indexed_lengths.emplace_back(edge_lengths[i], i);
			}
		}

		// 按边长降序排序，选择最长的边
		std::sort(indexed_lengths.begin(), indexed_lengths.end(),
			std::greater<std::pair<float, size_t>>());

		std::vector<bool> constrained_outlier(is_outlier.size(), false);
		for (size_t i = 0; i < max_allowed_outliers && i < indexed_lengths.size(); ++i) {
			constrained_outlier[indexed_lengths[i].second] = true;
		}

		LOG(WARNING) << "Too many outliers detected (" << outlier_count
			<< "), limiting to " << max_allowed_outliers;

		return constrained_outlier;
	}

public:
	AdaptiveEdgeLengthFilter(const AdaptiveFilterConfig& config) : config_(config) {}

	// 主要的过滤函数
	std::vector<bool> detectOutliers(const std::vector<float>& edge_lengths) {
		if (edge_lengths.empty()) {
			return std::vector<bool>();
		}

		Statistics stats = computeStatistics(edge_lengths);

		// 输出统计信息
		LOG(INFO) << "Edge length statistics:";
		LOG(INFO) << "  - Count: " << edge_lengths.size();
		LOG(INFO) << "  - Min: " << stats.min_val << ", Max: " << stats.max_val;
		LOG(INFO) << "  - Mean: " << stats.mean << ", Median: " << stats.median;
		LOG(INFO) << "  - Std: " << stats.std_dev;
		LOG(INFO) << "  - Q1: " << stats.q1 << ", Q3: " << stats.q3 << ", IQR: " << stats.iqr;

		switch (config_.method) {
		case AdaptiveFilterConfig::FilterMethod::FIXED_THRESHOLD:
			return detectByFixedThreshold(edge_lengths, stats);

		case AdaptiveFilterConfig::FilterMethod::IQR_OUTLIER:
			return detectByIQR(edge_lengths, stats);

		case AdaptiveFilterConfig::FilterMethod::ZSCORE_OUTLIER:
			return detectByZScore(edge_lengths, stats);

		case AdaptiveFilterConfig::FilterMethod::PERCENTILE_BASED:
			return detectByPercentile(edge_lengths, stats);

		case AdaptiveFilterConfig::FilterMethod::DYNAMIC_CLUSTERING:
			return detectByClustering(edge_lengths, stats);

		case AdaptiveFilterConfig::FilterMethod::COMBINED_METHOD:
			return detectByCombinedMethod(edge_lengths, stats);

		default:
			return detectByFixedThreshold(edge_lengths, stats);
		}
	}
};

