#include "../Base/endian.h"
#include "../Base/logging.h"
#include "../Base/misc.h"
#include "../Base/threading.h"
#include "DBSCAN.h"
#include "graph_cut.h"
#include "mesh_operate.h"
#include "meshing.h"
#include "Reconstruction.h"
#include <algorithm>
#include <CGAL/boost/graph/IO/PLY.h>
#include <CGAL/Surface_mesh/Surface_mesh.h>
#include <fstream>
#include <iomanip>
#include <limits>
#include <tbb/parallel_for_each.h>
#include <tbb/spin_mutex.h>
#include <unordered_map>
#include <vector>

namespace {
	std::vector<Eigen::Vector4f> generateColorsWithQuantiles(
		const std::vector<double>& weights,
		std::vector<double>& quantiles_out) {
		const Eigen::Vector4f LIGHT_BLUE(0.678f, 0.847f, 0.902f, 1.0f); //默认基础颜色
		const Eigen::Vector4f RED(0.70f, 0.0f, 0.13f, 1.0f); // Red for Best quality
		const Eigen::Vector4f WHITE(1.0f, 1.0f, 1.0f, 1.0f); // Green for Medium quality
		const Eigen::Vector4f BLUE(0.23f, 0.29f, 0.75f, 1.0f); // Blue for Worst quality

		std::vector<Eigen::Vector4f> colors;
		quantiles_out.clear();
		colors.reserve(weights.size());

		if (weights.empty()) return colors;

		//////////方法一：3σ过滤//////////
		//// 收集所有非 0 的面积值
		//std::vector<double> nonZeroAreas;
		//nonZeroAreas.reserve(weights.size());
		//for (double area : weights) {
		//	if (area != 0.0) {
		//		nonZeroAreas.push_back(area);
		//	}
		//}
		//// 如果所有值都为 0 或者 weights 为空，则所有都设为基础颜色
		//if (nonZeroAreas.empty()) {
		//	for (size_t i = 0; i < weights.size(); ++i) {
		//		colors.emplace_back(LIGHT_BLUE);
		//	}
		//	return colors;
		//}
		//// 计算非 0 值的均值
		//double sum = std::accumulate(nonZeroAreas.begin(), nonZeroAreas.end(), 0.0);
		//double mean = sum / nonZeroAreas.size();
		//// 计算非 0 值的标准差
		//double sq_sum = std::accumulate(nonZeroAreas.begin(), nonZeroAreas.end(), 0.0,
		//	[mean](double acc, double val) {
		//		double diff = val - mean;
		//		return acc + diff * diff;
		//	});
		//double std_dev = std::sqrt(sq_sum / nonZeroAreas.size());
		//// 定义 3σ 范围
		//double lower_bound = mean - 3.0 * std_dev;
		//double upper_bound = mean + 3.0 * std_dev;
		//// 找到符合要求的最小和最大值，用于归一化
		//double min_val = std::numeric_limits<double>::max();
		//double max_val = std::numeric_limits<double>::lowest();
		//for (double area : nonZeroAreas) {
		//	if (area >= lower_bound && area <= upper_bound) {
		//		if (area < min_val) min_val = area;
		//		if (area > max_val) max_val = area;
		//	}
		//}
		//// 处理边界情况，避免除以零
		//double range = max_val - min_val;
		//if (range == 0.0) {
		//	range = 1.0; // 所有值相同，归一化后设为 1
		//}

		// 记录非零并排序
		std::vector<double> sorted_weights = weights;
		std::sort(sorted_weights.begin(), sorted_weights.end());

		// 用 map 缓存每个值的分位百分比（quantile）0~1
		std::map<double, double> quantile_map;
		for (size_t i = 0; i < sorted_weights.size(); ++i) {
			double val = sorted_weights[i];
			double quantile = static_cast<double>(i) / (sorted_weights.size() - 1); // 范围 [0,1]
			quantile_map[val] = quantile; // 若重复值，后面的覆盖前面，无影响
		}

		// 映射每个 weight 到颜色
		for (double w : weights) {
			double q = quantile_map[w];
			quantiles_out.push_back(q);

			Eigen::Vector4f color;
			if (q < 0.5) {
				float t = static_cast<float>(q / 0.5); // 0~1
				color = (1 - t) * BLUE + t * WHITE;
			}
			else {
				float t = static_cast<float>((q - 0.5) / 0.5);
				color = (1 - t) * WHITE + t * RED;
			}

			color[3] = 1.0f;
			colors.emplace_back(color);
		}

		return colors;
	}

	// 获取四面体邻接的四面体
	std::vector<Cell_handle> get_adjacent_cells_safe(const Delaunay& dt, Cell_handle c) {
		std::vector<Cell_handle> neighbors;
		if (dt.is_infinite(c)) {
			for (int i = 0; i < 4; ++i) {
				Cell_handle neighbor_cell = c->neighbor(i);
				if (!dt.is_infinite(neighbor_cell)) {
					neighbors.push_back(neighbor_cell);
				}
			}
		}
		else {
			for (int i = 0; i < 4; ++i) {
				Cell_handle neighbor_cell = c->neighbor(i);
				neighbors.push_back(neighbor_cell);
			}
		}
		return neighbors;
	}

	void outputCellIDsToCSV(const Delaunay& dt, const std::string& filename) {
		std::ofstream outFile(filename);

		if (!outFile.is_open()) {
			std::cerr << "Error opening file: " << filename << std::endl;
			return;
		}

		// Write CSV header
		outFile << "vertex0,vertex1,vertex2,vertex3,is_infinite\n";

		// Iterate over all finite cells
		for (auto cell_it = dt.finite_cells_begin(); cell_it != dt.finite_cells_end(); ++cell_it) {
			bool is_infinite = dt.is_infinite(cell_it);

			// Get the four vertex IDs
			std::vector<size_t> vertices;
			for (int i = 0; i < 4; ++i) {
				vertices.push_back(cell_it->vertex(i)->info());
			}

			// Sort the vertex IDs in ascending order
			std::sort(vertices.begin(), vertices.end());

			// Output sorted vertex IDs and infinite flag
			outFile << vertices[0] << ","
				<< vertices[1] << ","
				<< vertices[2] << ","
				<< vertices[3] << ","
				<< (is_infinite ? "1" : "0") << "\n";
		}

		outFile.close();
	}

	void WritePlyMeshToFile(const std::shared_ptr<PlyMesh>& mesh, const std::string& filename) {
		if (!mesh) {
			return;
		}

		std::ofstream outFile(filename);
		if (!outFile.is_open()) {
			throw std::runtime_error("无法打开文件: " + filename);
		}

		// 写入PLY文件头
		outFile << "ply\n";
		outFile << "format ascii 1.0\n";
		outFile << "comment Generated by PlyMesh exporter\n";

		// 顶点属性
		outFile << "element vertex " << mesh->vertices.size() << "\n";
		outFile << "property float x\n";
		outFile << "property float y\n";
		outFile << "property float z\n";
		outFile << "property uint point3D_idx\n";

		// 面属性
		outFile << "element face " << mesh->faces.size() << "\n";
		outFile << "property list uchar uint vertex_indices\n";
		outFile << "property float img_track\n";

		outFile << "end_header\n";

		// 写入顶点数据
		for (const auto& vertex : mesh->vertices) {
			outFile << vertex.x << " " << vertex.y << " " << vertex.z;
			outFile << " " << vertex.point3D_idx << "\n";
		}

		// 写入面数据
		for (const auto& face : mesh->faces) {
			outFile << "3 " << face.vertex_idx1 << " " << face.vertex_idx2 << " " << face.vertex_idx3;
			outFile << " " << face.img_track << "\n";
		}

		outFile.close();
	}

	struct TrackPointTriplet {
		TrackPoint2D point_a, point_b, point_c;
	};

	std::vector<TrackPointTriplet> common_image_points(
		const std::vector<TrackPoint2D>& point2ds_a,
		const std::vector<TrackPoint2D>& point2ds_b,
		const std::vector<TrackPoint2D>& point2ds_c) {

		std::unordered_map<uint32_t, TrackPoint2D> map_a, map_b, map_c;

		for (const auto& point : point2ds_a) {
			map_a[point.image_id] = point;
		}
		for (const auto& point : point2ds_b) {
			map_b[point.image_id] = point;
		}
		for (const auto& point : point2ds_c) {
			map_c[point.image_id] = point;
		}

		std::vector<TrackPointTriplet> result;
		for (const auto& [id, point_a] : map_a) {
			auto it_b = map_b.find(id);
			auto it_c = map_c.find(id);

			if (it_b != map_b.end() && it_c != map_c.end()) {
				result.push_back({ point_a, it_b->second, it_c->second });
			}
		}
		return result;
	}
}

// DelaunayMeshingInput implementation
void DelaunayMeshingInput::ReadSparseReconstruction(const std::string& path) {
	Reconstruction reconstruction;
	reconstruction.Read(path);
	CopyFromSparseReconstruction(reconstruction);
}

void DelaunayMeshingInput::CopyFromSparseReconstruction(const Reconstruction& reconstruction) {
	images.clear(); points.clear(); cameras.clear();
	images.reserve(reconstruction.NumRegImages());
	points.reserve(reconstruction.NumPoints3D());

	cameras = reconstruction.Cameras();

	for (const auto& point3D : reconstruction.Points3D()) {
		DelaunayMeshingInput::Point input_point;
		input_point.position = point3D.second.XYZ().cast<float>();
		input_point.num_visible_images = point3D.second.Track().Length();
		input_point.pro_error = point3D.second.Error();
		for (const auto& element : point3D.second.Track().Elements()) {
			input_point.visible_images_ids.emplace_back(element.image_id);
			const auto& img = reconstruction.Image(element.image_id);
			const auto& p = img.Point2D(element.point2D_idx);
			input_point.point2ds.emplace_back(TrackPoint2D{ {p.xy[0], p.xy[1]}, element.image_id, element.point2D_idx });
		}
		points[point3D.first] = input_point;
	}

	for (const image_t& image_id : reconstruction.RegImageIds()) {
		const auto& image = reconstruction.Image(image_id);
		DelaunayMeshingInput::Image input_image;
		input_image.camera_id = image.CameraId();
		input_image.proj_matrix = image.CamFromWorld().ToMatrix().cast<float>();
		input_image.proj_center = image.ProjectionCenter().cast<float>();
		input_image.point_ids.reserve(image.NumPoints3D());
		for (const auto& point2D : image.Points2D()) {
			if (point2D.HasPoint3D()) {
				input_image.point_ids.push_back(point2D.point3D_id);
			}
		}
		images[image_id] = input_image;

		modified_images.emplace(image_id);
	}

	pointsTracker.init(points);
	imagesTracker.init(images);

	for (const auto& image_id : modified_images) {
		const auto& image = images[image_id];
		for (const auto& point_id : image.point_ids) {
			modified_points.emplace(point_id);
		}
	}

	// 为每个modified_images影像添加虚拟三维点
	//{
	//	point3D_t next_virtual_id = std::numeric_limits<point3D_t>::max();
	//	for (const auto& image_id : modified_images) {
	//		auto& image = images[image_id];
	//		// 收集所有已关联三维点的位置
	//		std::vector<Eigen::Vector3f> point_positions;
	//		for (const auto& pid : image.point_ids) {
	//			auto it = points.find(pid);
	//			if (it != points.end()) {
	//				point_positions.push_back(it->second.position);
	//			}
	//		}
	//		Eigen::Vector3f cam_center = image.proj_center;
	//		float min_dist = std::numeric_limits<float>::max();
	//		Eigen::Vector3f avg_dir = Eigen::Vector3f::Zero();
	//		int valid_count = 0;
	//		for (const auto& pos : point_positions) {
	//			Eigen::Vector3f dir = pos - cam_center;
	//			float dist = dir.norm();
	//			if (dist > 1e-6) {
	//				avg_dir += dir.normalized();
	//				if (dist < min_dist) min_dist = dist;
	//				++valid_count;
	//			}
	//		}
	//		if (valid_count > 0) {
	//			avg_dir /= static_cast<float>(valid_count);
	//		}
	//		else {
	//			avg_dir = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
	//			min_dist = 1.0f;
	//		}
	//		if (avg_dir.norm() < 1e-6) avg_dir = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
	//		avg_dir.normalize();
	//		float virtual_dist = min_dist * 0.8f;
	//		Eigen::Vector3f virtual_pos = cam_center + avg_dir * virtual_dist;
	//		// 生成虚拟点ID
	//		--next_virtual_id;
	//		DelaunayMeshingInput::Point fake_point;
	//		fake_point.position = virtual_pos;
	//		fake_point.num_visible_images = 1;
	//		fake_point.visible_images_ids.push_back(image_id);
	//		points[next_virtual_id] = fake_point;
	//		modified_points.emplace(next_virtual_id);
	//	}
	//}
}

void DelaunayMeshingInput::UpdateFromSparseReconstruction(const Reconstruction& reconstruction) {
	std::unordered_map<image_t, Image> new_images;
	std::unordered_map<point3D_t, Point> new_points;
	new_images.reserve(reconstruction.NumRegImages());
	new_points.reserve(reconstruction.NumPoints3D());

	cameras = reconstruction.Cameras();

	for (const auto& point3D : reconstruction.Points3D()) {
		DelaunayMeshingInput::Point input_point;
		input_point.position = point3D.second.XYZ().cast<float>();
		input_point.num_visible_images = point3D.second.Track().Length();
		new_points[point3D.first] = input_point;
	}

	for (const image_t& image_id : reconstruction.RegImageIds()) {
		const auto& image = reconstruction.Image(image_id);
		DelaunayMeshingInput::Image input_image;
		input_image.camera_id = image.CameraId();
		input_image.proj_matrix = image.CamFromWorld().ToMatrix().cast<float>();
		input_image.proj_center = image.ProjectionCenter().cast<float>();
		input_image.point_ids.reserve(image.NumPoints3D());
		for (const auto& point2D : image.Points2D()) {
			if (point2D.HasPoint3D()) {
				input_image.point_ids.push_back(point2D.point3D_id);
				new_points[point2D.point3D_id].visible_images_ids.push_back(image_id);
			}
		}
		new_images[image_id] = input_image;
	}

	pointsTracker.track_changes(new_points);
	imagesTracker.track_changes(new_images);

	modified_images.clear();
	modified_points.clear();

	for (const auto& point_id : pointsTracker.get_added_keys()) {
		modified_images.insert(new_points[point_id].visible_images_ids.begin(), new_points[point_id].visible_images_ids.end());
	}
	points = std::move(new_points);
	images = std::move(new_images);

	for (const auto& image_id : modified_images) {
		const auto& image = images[image_id];
		for (const auto& point_id : image.point_ids) {
			modified_points.emplace(point_id);
		}
	}

	// 为每个modified_images影像添加虚拟三维点
	// 1. 找到当前所有点的最大ID，作为虚拟点ID起始值
	point3D_t max_point_id = 0;
	for (const auto& kv : points) {
		if (kv.first > max_point_id) max_point_id = kv.first;
	}
	point3D_t next_virtual_id = max_point_id + 1;

	for (const auto& image_id : modified_images) {
		auto& image = images[image_id];
		// 收集所有已关联三维点的位置
		std::vector<Eigen::Vector3f> point_positions;
		for (const auto& pid : image.point_ids) {
			auto it = points.find(pid);
			if (it != points.end()) {
				point_positions.push_back(it->second.position);
			}
		}
		Eigen::Vector3f cam_center = image.proj_center;
		float min_dist = std::numeric_limits<float>::max();
		Eigen::Vector3f avg_dir = Eigen::Vector3f::Zero();
		int valid_count = 0;
		for (const auto& pos : point_positions) {
			Eigen::Vector3f dir = pos - cam_center;
			float dist = dir.norm();
			if (dist > 1e-6) {
				avg_dir += dir.normalized();
				if (dist < min_dist) min_dist = dist;
				++valid_count;
			}
		}
		if (valid_count > 0) {
			avg_dir /= static_cast<float>(valid_count);
		}
		else {
			avg_dir = Eigen::Vector3f(1.0f, 0.0f, 0.0f); // 默认方向
			min_dist = 1.0f; // 默认距离
		}
		if (avg_dir.norm() < 1e-6) avg_dir = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
		avg_dir.normalize();
		float virtual_dist = min_dist * 0.8f;
		Eigen::Vector3f virtual_pos = cam_center + avg_dir * virtual_dist;
		// 生成虚拟点ID
		++next_virtual_id;
		DelaunayMeshingInput::Point virtual_point;
		virtual_point.position = virtual_pos;
		virtual_point.num_visible_images = 1;
		virtual_point.visible_images_ids.push_back(image_id);
		points[next_virtual_id] = virtual_point;
		modified_points.emplace(next_virtual_id);
	}

	//LOG(INFO) << StringPrintf("add %d points, remove %d points, total points: %d",
	//    pointsTracker.get_added_keys().size(),
	//    pointsTracker.get_removed_keys().size(),
	//    points.size());
}

Delaunay DelaunayMeshingInput::CreateDelaunayTriangulation() const {
	std::vector<Delaunay::Point> delaunay_points(points.size());
	for (const auto& [key, point] : points) {
		delaunay_points.push_back(Delaunay::Point(
			point.position.x(),
			point.position.y(),
			point.position.z()));
	}
	return Delaunay(delaunay_points.begin(), delaunay_points.end());
}

void DelaunayMeshingInput::CreateSubSampledDelaunayTriangulation(
	const float max_proj_dist, const float max_depth_dist, Delaunay& triangulation) {

	if (max_proj_dist == 0) {
		triangulation = CreateDelaunayTriangulation();
		return;
	}

	triangulation.clear();
	const float max_squared_proj_dist = max_proj_dist * max_proj_dist;
	const float min_depth_ratio = 1.0f - max_depth_dist;
	const float max_depth_ratio = 1.0f + max_depth_dist;

	for (const auto& point_id : modified_points) {
		const auto it = points.find(point_id);
		if (it == points.end()) continue;
		const auto& point = it->second;
		const K::Point_3 point_position = EigenToCGAL(point.position);

		// Insert point into triangulation until there is one cell.
		if (triangulation.number_of_vertices() < 4) {
			Vertex_handle v = triangulation.insert(point_position);
			v->info() = point_id;
			continue;
		}

		const Cell_handle cell = triangulation.locate(point_position);

		// If the point is outside the current hull, then extend the hull.
		if (triangulation.is_infinite(cell)) {
			auto v = triangulation.insert(point_position);
			v->info() = point_id;
			continue;
		}

		// Project point and located cell vertices to all visible images and
		// determine reprojection error.

		bool insert_point = false;

		for (const image_t& image_id : point.visible_images_ids) {
			const auto img_it = images.find(image_id);
			if (img_it == images.end()) continue;
			const auto& image = img_it->second;
			const auto& camera = cameras.at(image.camera_id);

			for (int i = 0; i < 4; ++i) {
				const Eigen::Vector3f cell_point =
					CGALToEigen(cell->vertex(i)->point());

				const Eigen::Vector3f point_local =
					image.proj_matrix * point.position.homogeneous();
				const Eigen::Vector3f cell_point_local =
					image.proj_matrix * cell_point.homogeneous();

				// Ensure that both points are infront of camera.
				if (point_local.z() <= 0 || cell_point_local.z() <= 0) {
					insert_point = true;
					break;
				}

				// Check depth ratio between the two points.
				const float depth_ratio = point_local.z() / cell_point_local.z();
				if (depth_ratio < min_depth_ratio || depth_ratio > max_depth_ratio) {
					insert_point = true;
					break;
				}

				// Check reprojection error between the two points.
				const Eigen::Vector2f point_proj =
					camera.ImgFromCam(point_local.hnormalized().cast<double>())
					.cast<float>();
				const Eigen::Vector2f cell_point_proj =
					camera.ImgFromCam(cell_point_local.hnormalized().cast<double>())
					.cast<float>();
				const float squared_proj_dist =
					(point_proj - cell_point_proj).squaredNorm();
				if (squared_proj_dist > max_squared_proj_dist) {
					insert_point = true;
					break;
				}
			}

			if (insert_point) {
				break;
			}
		}

		if (insert_point) {
			auto v = triangulation.insert(point_position);
			v->info() = point_id;
		}
	}
}

void DelaunayMeshingInput::ClearCaches() {
	cameras.clear();
	images.clear();
	points.clear();
	modified_images.clear();
	modified_points.clear();
}

void MeshManager::CopyFromReconstruction(const Reconstruction& reconstruction)
{
	input_data.CopyFromSparseReconstruction(reconstruction);
}

void MeshManager::CreateMesh() {
	timer.start("创建Delaunay三角剖分");
	input_data.CreateSubSampledDelaunayTriangulation(options.max_proj_dist, options.max_depth_dist, triangulation);
	timer.stop();

	timer.start("初始化权重计算器");
	ray_caster = new DelaunayTriangulationRayCaster(triangulation);
	edge_weight_computer = new DelaunayMeshingEdgeWeightComputer(triangulation, options.visibility_sigma, options.distance_sigma_factor);
	timer.stop();

	timer.start("计算所有光线权重图");

	CellGraphData cell_graph_data;
	cell_graph_data.reserve(triangulation.number_of_cells());
	for (auto it = triangulation.all_cells_begin();
		it != triangulation.all_cells_end();
		++it) {
		cell_graph_data.emplace(it, DelaunayCellData(cell_graph_data.size()));
	}

	// Function that accumulates edge weights in the s-t graph for a single image.
	auto IntegreateImage = [&](const size_t image_id) -> CellGraphData {
		// Accumulated weights for the current image only.
		CellGraphData image_cell_graph_data;

		// Image that is integrated into s-t graph.
		const auto& image = input_data.images[image_id];
		const K::Point_3 image_position = EigenToCGAL(image.proj_center);

		// Intersections between viewing rays and Delaunay triangulation.
		std::vector<DelaunayTriangulationRayCaster::Intersection> intersections;

		// Iterate through all image observations and integrate them into the graph.
		for (const auto& point_id : image.point_ids) {
			const auto& point = input_data.points[point_id];

			// Likelihood of the point observation.
			const double alpha = edge_weight_computer->ComputeVisibilityProb(
				point.num_visible_images * point.num_visible_images);

			const K::Point_3 point_position = EigenToCGAL(point.position);
			const K::Ray_3 viewing_ray = K::Ray_3(image_position, point_position);
			const K::Vector_3 viewing_direction = point_position - image_position;
			const K::Vector_3 viewing_direction_normalized =
				viewing_direction / std::sqrt(viewing_direction.squared_length());
			const K::Vector_3 viewing_direction_epsilon =
				0.001 * edge_weight_computer->DistanceSigma() *
				viewing_direction_normalized;

			// Find intersected facets between image and point.
			ray_caster->CastRaySegment(
				K::Segment_3(image_position,
					point_position - viewing_direction_epsilon),
				&intersections);

			// Accumulate source weights for cell containing image.
			if (!intersections.empty()) {
				image_cell_graph_data[intersections.front().facet.first]
					.source_weight += alpha;
			}

			// Accumulate edge weights from image to point.
			for (const auto& intersection : intersections) {
				image_cell_graph_data[intersection.facet.first]
					.edge_weights[intersection.facet.second] +=
					alpha * edge_weight_computer->ComputeDistanceProb(
						intersection.target_distance_squared);
			}

			// Accumulate edge weights from point to extended point
			// and accumulate sink weight of the cell inside the surface.

			{
				// Find the first facet that is intersected by the extended ray behind
				// the observed point. Then accumulate the edge weight of that facet
				// and accumulate the sink weight of the cell behind that facet.

				const Delaunay::Cell_handle behind_point_cell =
					triangulation.locate(point_position + viewing_direction_epsilon);

				int behind_neighbor_idx = -1;
				double behind_distance_squared = 0.0;
				for (int neighbor_idx = 0; neighbor_idx < 4; ++neighbor_idx) {
					const K::Triangle_3 triangle =
						triangulation.triangle(behind_point_cell, neighbor_idx);

					K::Point_3 inter_point;
					if (CGAL::assign(inter_point,
						CGAL::intersection(viewing_ray, triangle))) {
						const double distance_squared =
							(inter_point - point_position).squared_length();
						if (distance_squared > behind_distance_squared) {
							behind_distance_squared = distance_squared;
							behind_neighbor_idx = neighbor_idx;
						}
					}
				}

				if (behind_neighbor_idx >= 0) {
					image_cell_graph_data[behind_point_cell]
						.edge_weights[behind_neighbor_idx] +=
						alpha *
						edge_weight_computer->ComputeDistanceProb(behind_distance_squared);

					const auto& inside_cell =
						behind_point_cell->neighbor(behind_neighbor_idx);
					image_cell_graph_data[inside_cell].sink_weight += alpha;
				}
			}
		}
		return image_cell_graph_data;
		};
	// Add first batch of images to the thread job queue.
	std::vector<image_t> image_vector(input_data.modified_images.begin(), input_data.modified_images.end());
	for (size_t i = 0; i < input_data.modified_images.size(); ++i) {
		const auto image_cell_graph_data = IntegreateImage(i);
		for (const auto& image_cell_data : image_cell_graph_data) {
			auto& cell_data = cell_graph_data.at(image_cell_data.first);
			cell_data.sink_weight += image_cell_data.second.sink_weight;
			cell_data.source_weight += image_cell_data.second.source_weight;
			for (size_t j = 0; j < cell_data.edge_weights.size(); ++j) {
				cell_data.edge_weights[j] += image_cell_data.second.edge_weights[j];
			}
		}
	}

	// Setup the min-cut (max-flow) graph optimization.

	// Each oriented facet in the Delaunay triangulation corresponds to a directed
	// edge and each cell corresponds to a node in the graph.
	MinSTGraphCut<size_t, float> graph_cut(cell_graph_data.size());

	// Iterate all cells in the triangulation.
	for (auto& cell_data : cell_graph_data) {
		graph_cut.AddNode(cell_data.second.index,
			cell_data.second.source_weight,
			cell_data.second.sink_weight);

		// Iterate all facets of the current cell to accumulate edge weight.
		for (int i = 0; i < 4; ++i) {
			// Compose the current facet.
			const Delaunay::Facet facet = std::make_pair(cell_data.first, i);

			// Extract the mirrored facet of the current cell (opposite orientation).
			const Delaunay::Facet mirror_facet = triangulation.mirror_facet(facet);
			const auto& mirror_cell_data = cell_graph_data.at(mirror_facet.first);

			// Avoid duplicate edges in graph.
			if (cell_data.second.index < mirror_cell_data.index) {
				continue;
			}

			// Implementation of geometry visualized in Figure 9 in P. Labatut, J‐P.
			// Pons, and R. Keriven. "Robust and efficient surface reconstruction from
			// range data." Computer graphics forum, 2009.
			const double edge_shape_weight =
				options.quality_regularization *
				(1.0 -
					std::min(ComputeCosFacetCellAngle(facet),
						ComputeCosFacetCellAngle(mirror_facet)));

			const float forward_edge_weight =
				cell_data.second.edge_weights[facet.second] + edge_shape_weight;
			const float backward_edge_weight =
				mirror_cell_data.edge_weights[mirror_facet.second] +
				edge_shape_weight;

			graph_cut.AddEdge(cell_data.second.index,
				mirror_cell_data.index,
				forward_edge_weight,
				backward_edge_weight);
		}
	}

	timer.stop();

	// Extract the surface facets as the oriented min-cut of the graph.
	timer.start("运行graph-cut optimization");
	graph_cut.Compute();
	timer.stop();

	timer.start("提取mesh表面");
	std::unordered_set<Delaunay::Vertex_handle> surface_vertices;
	std::vector<Delaunay::Facet> surface_facets;
	std::vector<float> surface_facet_side_lengths;

	for (auto it = triangulation.finite_facets_begin();
		it != triangulation.finite_facets_end();
		++it) {

		if (triangulation.is_infinite(it->first->neighbor(it->second))) {
			continue;
		}

		const auto& cell_data = cell_graph_data.at(it->first);
		const auto& mirror_cell_data =
			cell_graph_data.at(it->first->neighbor(it->second));

		// Obtain labeling after the graph-cut.
		const bool cell_is_source = graph_cut.IsConnectedToSource(cell_data.index);
		const bool mirror_cell_is_source =
			graph_cut.IsConnectedToSource(mirror_cell_data.index);

		// The surface is equal to the location of the cut, which is at the
		// transition between source and sink nodes.
		if (cell_is_source == mirror_cell_is_source) {
			continue;
		}

		// Remember all unique vertices of the surface mesh.
		for (int i = 0; i < 3; ++i) {
			const auto& vertex =
				it->first->vertex(triangulation.vertex_triple_index(it->second, i));
			surface_vertices.insert(vertex);
		}

		// Determine maximum side length of facet.
		const K::Triangle_3 triangle = triangulation.triangle(*it);
		const float max_squared_side_length =
			std::max({ (triangle[0] - triangle[1]).squared_length(),
					  (triangle[0] - triangle[2]).squared_length(),
					  (triangle[1] - triangle[2]).squared_length() });
		surface_facet_side_lengths.push_back(std::sqrt(max_squared_side_length));

		// Remember surface mesh facet and make sure it is oriented correctly.
		if (cell_is_source) {
			surface_facets.push_back(*it);
		}
		else {
			surface_facets.push_back(triangulation.mirror_facet(*it));
		}
	}

	std::unordered_map<const Delaunay::Vertex_handle, size_t>
		surface_vertex_indices;
	surface_vertex_indices.reserve(surface_vertices.size());
	mesh->vertices.reserve(surface_vertices.size());
	for (const auto& vertex : surface_vertices) {
		mesh->vertices.emplace_back(
			vertex->point().x(), vertex->point().y(), vertex->point().z(), vertex->info());
		surface_vertex_indices.emplace(vertex, surface_vertex_indices.size());
	}

	mesh->faces.reserve(surface_facets.size());

	for (size_t i = 0; i < surface_facets.size(); ++i) {
		const auto& facet = surface_facets[i];
		mesh->faces.emplace_back(
			surface_vertex_indices.at(facet.first->vertex(
				triangulation.vertex_triple_index(facet.second, 0))),
			surface_vertex_indices.at(facet.first->vertex(
				triangulation.vertex_triple_index(facet.second, 1))),
			surface_vertex_indices.at(facet.first->vertex(
				triangulation.vertex_triple_index(facet.second, 2))));
	}
	timer.stop();
	timer.printExecutionTimes();

	//mesh = FilterSurfaceFacetsWithBoundaryAdaptation(
	//    surface_vertices,
	//    surface_facets,
	//    surface_facet_side_lengths);

	//MeshOperator::FilterConfig config;
	//config.outlier_threshold_multiplier = 2.5;
	//config.max_iterations = 5;
	//MeshOperator::FilterMesh(mesh, config);
	//{
	//	CMesh cgalmesh;
	//	std::unordered_map<size_t, CMesh::Vertex_index> vertex_map;
	//	size_t index = 0;
	//	for (const auto& vh : mesh->vertices) {
	//		Point p = Point(vh.x, vh.y, vh.z);
	//		auto cgal_vh = cgalmesh.add_vertex(p);
	//		vertex_map[index] = cgal_vh;
	//		index++;
	//	}
	//	for (const auto& facet : mesh->faces) {
	//		cgalmesh.add_face(
	//			vertex_map[facet.vertex_idx1],
	//			vertex_map[facet.vertex_idx2],
	//			vertex_map[facet.vertex_idx3]
	//		);
	//	}
	//	CGAL::IO::write_PLY("D:\\PROJECT\\On_the_fly_SfME\\x64\\Release\\cgal_create.ply", cgalmesh);
	//}
	//WriteTextPlyMesh("D:/PROJECT/On_the_fly_SfME/x64/Release/create.ply", *mesh);
}

void MeshManager::UpdateMesh() {
	StackTimer timer;

	/*timer.start("总耗时");
	timer.start("读取重建数据");
	input_data.UpdateFromSparseReconstruction(reconstruction);
	timer.stop();*/
	timer.start("创建Delaunay三角剖分");
	input_data.CreateSubSampledDelaunayTriangulation(options.max_proj_dist, options.max_depth_dist, triangulation);
	timer.stop();

	//outputCellIDsToCSV(triangulation , "cellblock.csv");

	//LOG(INFO) << "Initializing ray tracer...";
	timer.start("初始化权重计算器");
	ray_caster = new DelaunayTriangulationRayCaster(triangulation);
	edge_weight_computer = new DelaunayMeshingEdgeWeightComputer(triangulation, options.visibility_sigma, options.distance_sigma_factor);
	timer.stop();

	//LOG(INFO) << "Initializing graph optimization...";
	timer.start("计算所有光线权重图");

	CellGraphData cell_graph_data;
	cell_graph_data.reserve(triangulation.number_of_cells());
	for (auto it = triangulation.all_cells_begin();
		it != triangulation.all_cells_end();
		++it) {
		cell_graph_data.emplace(it, DelaunayCellData(cell_graph_data.size()));
	}

	const int num_threads = GetEffectiveNumThreads(options.num_threads);
	ThreadPool thread_pool(num_threads);
	JobQueue<CellGraphData> result_queue(num_threads);

	// Function that accumulates edge weights in the s-t graph for a single image.
	// 遍历所有图像，对图像和其观察到的稀疏点之间的视线进行射线投射
	// 对每张图像进行光线投射和体素图（cell graph）构建
	// 对于一张图像，从相机位置出发，向其观测到的稀疏点投射光线，并在 Delaunay 三维网格中分析光线的穿透情况。由此构建出图中的 source_weight、sink_weight、edge_weight 数据，用于最终的图优化（min-cut）。

	//**** Lambda 表达式（匿名函数） 的赋值，用于在局部定义一个函数对象****
	//****一个可捕获外部变量的匿名函数（Lambda）赋值给变量 IntegreateImage，后续你可以像函数一样使用它（例如并发调用、传入线程池）****
	auto IntegreateImage = [&](const size_t image_id) {
		// 当前图像构建的局部 graph 数据结构
		CellGraphData image_cell_graph_data;

		// Image that is integrated into s-t graph.
		const auto& image = input_data.images[image_id];
		const K::Point_3 image_position = EigenToCGAL(image.proj_center);

		// Intersections between viewing rays and Delaunay triangulation.
		std::vector<DelaunayTriangulationRayCaster::Intersection> intersections;

		// Iterate through all image observations and integrate them into the graph.
		for (const auto& point_id : image.point_ids) {
			const auto& point = input_data.points[point_id];

			// Likelihood of the point observation.
			const double alpha = edge_weight_computer->ComputeVisibilityProb(
				point.num_visible_images * point.num_visible_images);

			const K::Point_3 point_position = EigenToCGAL(point.position);
			const K::Ray_3 viewing_ray = K::Ray_3(image_position, point_position);//构建光线：图像-->点
			const K::Vector_3 viewing_direction = point_position - image_position;
			const K::Vector_3 viewing_direction_normalized =
				viewing_direction / std::sqrt(viewing_direction.squared_length());
			const K::Vector_3 viewing_direction_epsilon =
				0.001 * edge_weight_computer->DistanceSigma() *
				viewing_direction_normalized;//防止数值误差导致交点计算失败

			// Find intersected facets between image and point.
			ray_caster->CastRaySegment(
				K::Segment_3(image_position,
					point_position - viewing_direction_epsilon),
				&intersections);// 射线与 Delaunay 网格交点

			// Accumulate source weights for cell containing image.
			if (!intersections.empty()) {
				image_cell_graph_data[intersections.front().facet.first]
					.source_weight += alpha;
			}

			// Accumulate edge weights from image to point.
			// 对交点 facet 的 边权重（edge_weight）累加
			for (const auto& intersection : intersections) {
				image_cell_graph_data[intersection.facet.first]
					.edge_weights[intersection.facet.second] +=
					alpha * edge_weight_computer->ComputeDistanceProb(
						intersection.target_distance_squared);// 累加可见性、距离概率权重
			}

			// Accumulate edge weights from point to extended point
			// and accumulate sink weight of the cell inside the surface.

			{
				// Find the first facet that is intersected by the extended ray behind
				// the observed point. Then accumulate the edge weight of that facet
				// and accumulate the sink weight of the cell behind that facet.
				//soft visibility constrains

				const Delaunay::Cell_handle behind_point_cell =
					triangulation.locate(point_position + viewing_direction_epsilon);//定位稀疏点后方的cell

				int behind_neighbor_idx = -1;
				double behind_distance_squared = 0.0;

				//遍历该 cell 的 4 个邻面，找哪个面是 ray 穿过的
				for (int neighbor_idx = 0; neighbor_idx < 4; ++neighbor_idx) {
					const K::Triangle_3 triangle =
						triangulation.triangle(behind_point_cell, neighbor_idx);//返回这个面对应的三角形

					//返回这个面对应的三角形
					K::Point_3 inter_point;
					if (CGAL::assign(inter_point,// CGAL 的通用类型转换工具，用于将交点赋值到 inter_point
						CGAL::intersection(viewing_ray, triangle))) {
						//找“最远”的交点，作为真正穿透的那一面
						const double distance_squared =
							(inter_point - point_position).squared_length();
						if (distance_squared > behind_distance_squared) {
							behind_distance_squared = distance_squared;
							behind_neighbor_idx = neighbor_idx;
						}
					}
				}

				/*
				* behind_point_cell 是当前 cell
				neighbor_idx 是穿过的那个面
				通过距离概率函数计算权重：距离越远，权重越小
				乘上 alpha，这是整体的光线权重因子
				@note 该 cell 向后方穿透的那一面，其“穿越边”的权重提升
				*/
				if (behind_neighbor_idx >= 0) {
					image_cell_graph_data[behind_point_cell]
						.edge_weights[behind_neighbor_idx] +=
						alpha *
						edge_weight_computer->ComputeDistanceProb(behind_distance_squared);

					const auto& inside_cell =
						behind_point_cell->neighbor(behind_neighbor_idx);
					image_cell_graph_data[inside_cell].sink_weight += alpha;
				}
			}
		}
		result_queue.Push(std::move(image_cell_graph_data));
		};

	// Add first batch of images to the thread job queue.
	std::vector<image_t> image_vector(input_data.modified_images.begin(), input_data.modified_images.end());
	size_t image_idx = 0;
	const size_t init_num_tasks =
		std::min(input_data.modified_images.size(), 2 * thread_pool.NumThreads());
	for (; image_idx < init_num_tasks; ++image_idx) {
		thread_pool.AddTask(IntegreateImage, image_vector[image_idx]);
	}

	// Pop the integrated images from the thread job queue and integrate their
	// accumulated weights into the global graph.
	for (size_t i = 0; i < input_data.modified_images.size(); ++i) {

		// Push the next image to the queue.
		if (image_idx < input_data.modified_images.size()) {
			thread_pool.AddTask(IntegreateImage, image_idx);
			image_idx += 1;
		}

		// Pop the next results from the queue.
		auto result = result_queue.Pop();

		// Accumulate the weights of the image into the global graph.
		const auto& image_cell_graph_data = result.Data();
		for (const auto& image_cell_data : image_cell_graph_data) {
			auto& cell_data = cell_graph_data.at(image_cell_data.first);
			cell_data.sink_weight += image_cell_data.second.sink_weight;
			cell_data.source_weight += image_cell_data.second.source_weight;
			for (size_t j = 0; j < cell_data.edge_weights.size(); ++j) {
				cell_data.edge_weights[j] += image_cell_data.second.edge_weights[j];
			}
		}
	}

	// Setup the min-cut (max-flow) graph optimization.

	// Each oriented facet in the Delaunay triangulation corresponds to a directed
	// edge and each cell corresponds to a node in the graph.
	MinSTGraphCut<size_t, float> graph_cut(cell_graph_data.size());

	// Iterate all cells in the triangulation.
	for (auto& cell_data : cell_graph_data) {
		graph_cut.AddNode(cell_data.second.index,
			cell_data.second.source_weight,
			cell_data.second.sink_weight);

		// Iterate all facets of the current cell to accumulate edge weight.
		for (int i = 0; i < 4; ++i) {
			// Compose the current facet.
			const Delaunay::Facet facet = std::make_pair(cell_data.first, i);

			// Extract the mirrored facet of the current cell (opposite orientation).
			const Delaunay::Facet mirror_facet = triangulation.mirror_facet(facet);
			const auto& mirror_cell_data = cell_graph_data.at(mirror_facet.first);

			// Avoid duplicate edges in graph.
			if (cell_data.second.index < mirror_cell_data.index) {
				continue;
			}

			// Implementation of geometry visualized in Figure 9 in P. Labatut, J‐P.
			// Pons, and R. Keriven. "Robust and efficient surface reconstruction from
			// range data." Computer graphics forum, 2009.
			const double edge_shape_weight =
				options.quality_regularization *
				(1.0 -
					std::min(ComputeCosFacetCellAngle(facet),
						ComputeCosFacetCellAngle(mirror_facet)));

			const float forward_edge_weight =
				cell_data.second.edge_weights[facet.second] + edge_shape_weight;
			const float backward_edge_weight =
				mirror_cell_data.edge_weights[mirror_facet.second] +
				edge_shape_weight;

			graph_cut.AddEdge(cell_data.second.index,
				mirror_cell_data.index,
				forward_edge_weight,
				backward_edge_weight);
		}
	}

	timer.stop();

	// Extract the surface facets as the oriented min-cut of the graph.
	timer.start("运行graph-cut optimization");
	graph_cut.Compute();
	timer.stop();

	timer.start("提取mesh表面");
	std::unordered_set<Delaunay::Vertex_handle> surface_vertices;
	std::vector<Delaunay::Facet> surface_facets;
	std::vector<float> surface_facet_side_lengths;

	for (auto it = triangulation.finite_facets_begin();
		it != triangulation.finite_facets_end();
		++it) {
		const auto& cell_data = cell_graph_data.at(it->first);
		const auto& mirror_cell_data =
			cell_graph_data.at(it->first->neighbor(it->second));

		// Obtain labeling after the graph-cut.
		const bool cell_is_source = graph_cut.IsConnectedToSource(cell_data.index);
		const bool mirror_cell_is_source =
			graph_cut.IsConnectedToSource(mirror_cell_data.index);

		// The surface is equal to the location of the cut, which is at the
		// transition between source and sink nodes.
		if (cell_is_source == mirror_cell_is_source) {
			continue;
		}

		// Remember all unique vertices of the surface mesh.
		for (int i = 0; i < 3; ++i) {
			const auto& vertex =
				it->first->vertex(triangulation.vertex_triple_index(it->second, i));
			surface_vertices.insert(vertex);
		}

		// Determine maximum side length of facet.
		const K::Triangle_3 triangle = triangulation.triangle(*it);
		const float max_squared_side_length =
			std::max({ (triangle[0] - triangle[1]).squared_length(),
					  (triangle[0] - triangle[2]).squared_length(),
					  (triangle[1] - triangle[2]).squared_length() });
		surface_facet_side_lengths.push_back(std::sqrt(max_squared_side_length));

		// Remember surface mesh facet and make sure it is oriented correctly.
		if (cell_is_source) {
			surface_facets.push_back(*it);
		}
		else {
			surface_facets.push_back(triangulation.mirror_facet(*it));
		}
	}

	//{
	//    CMesh cgalmesh;
	//    std::unordered_map<Delaunay::Vertex_handle, CMesh::Vertex_index> vertex_map;
	//    
	//    for (const auto& vh : surface_vertices) {
	//        Point p = Point(vh->point().x(), vh->point().y(), vh->point().z());
	//        auto cgal_vh = cgalmesh.add_vertex(p);
	//        vertex_map[vh] = cgal_vh;
	//    }
	//
	//    for (const auto& facet : surface_facets) {
	//        cgalmesh.add_face(
	//            vertex_map[facet.first->vertex(
	//                triangulation.vertex_triple_index(facet.second, 0))],
	//            vertex_map[facet.first->vertex(
	//                triangulation.vertex_triple_index(facet.second, 1))],
	//            vertex_map[facet.first->vertex(
	//                triangulation.vertex_triple_index(facet.second, 2))]
	//        );
	//    }
	//
	//    CGAL::IO::write_PLY("D:\\PROJECT\\On_the_fly_SfME\\x64\\Release\\cgal_update.ply", cgalmesh);
	//}

	std::unordered_map<const Delaunay::Vertex_handle, size_t>
		surface_vertex_indices;
	surface_vertex_indices.reserve(surface_vertices.size());
	std::shared_ptr<PlyMesh> meshblock = std::make_shared<PlyMesh>();
	meshblock->vertices.reserve(surface_vertices.size());
	for (const auto& vertex : surface_vertices) {
		meshblock->vertices.emplace_back(
			vertex->point().x(), vertex->point().y(), vertex->point().z(), vertex->info());
		surface_vertex_indices.emplace(vertex, surface_vertex_indices.size());
	}

	meshblock->faces.reserve(surface_facets.size());

	for (size_t i = 0; i < surface_facets.size(); ++i) {
		const auto& facet = surface_facets[i];
		meshblock->faces.emplace_back(
			surface_vertex_indices.at(facet.first->vertex(
				triangulation.vertex_triple_index(facet.second, 0))),
			surface_vertex_indices.at(facet.first->vertex(
				triangulation.vertex_triple_index(facet.second, 1))),
			surface_vertex_indices.at(facet.first->vertex(
				triangulation.vertex_triple_index(facet.second, 2))));
	}

	//std::shared_ptr<PlyMesh> newsurfacemesh = FilterSurfaceFacetsWithBoundaryAdaptation(
	//    surface_vertices,
	//    surface_facets,
	//    surface_facet_side_lengths);

	MeshOperator::FilterConfig config;
	config.outlier_threshold_multiplier = 2.5;
	config.max_iterations = 3;
	MeshOperator::FilterMesh(meshblock, config);

	{
		CMesh cgalmesh;
		std::unordered_map<size_t, CMesh::Vertex_index> vertex_map;
		size_t index = 0;
		for (const auto& vh : mesh->vertices) {
			Point p = Point(vh.x, vh.y, vh.z);
			auto cgal_vh = cgalmesh.add_vertex(p);
			vertex_map[index] = cgal_vh;
			index++;
		}

		for (const auto& facet : mesh->faces) {
			cgalmesh.add_face(
				vertex_map[facet.vertex_idx1],
				vertex_map[facet.vertex_idx2],
				vertex_map[facet.vertex_idx3]
			);
		}
		CGAL::IO::write_PLY("D:\\PROJECT\\On_the_fly_SfME\\x64\\Release\\cgal_update.ply", cgalmesh);
	}

	MeshOperator::MergeMesh(mesh, meshblock, input_data.modified_points);

	timer.stop();
	timer.stop();
	timer.printExecutionTimes();
	// ExportLargestFacets(10);
	WriteTextPlyMesh("D:/PROJECT/On_the_fly_SfME/x64/Release/mesh.ply", *mesh);
	WriteTextPlyMesh("D:/PROJECT/On_the_fly_SfME/x64/Release/meshblock.ply", *meshblock);
}

/*
* @brief 合并面片
*/
void MeshManager::MergeMesh(std::shared_ptr<PlyMesh> meshblock) {
	// 创建一个从 point3D_idx 到 mesh 中新顶点索引的映射
	std::unordered_map<point3D_t, size_t> point3d_to_vertex_idx;

	// 首先处理 mesh 中已有的顶点，建立映射
	for (size_t i = 0; i < mesh->vertices.size(); ++i) {
		const auto& vertex = mesh->vertices[i];
		if (vertex.point3D_idx != -1) {
			point3d_to_vertex_idx[vertex.point3D_idx] = i;
		}
	}

	// 记录 meshblock 中顶点索引到 mesh 中新顶点索引的映射
	std::vector<size_t> vertex_index_map(meshblock->vertices.size());

	// 处理 meshblock 中的顶点
	for (size_t i = 0; i < meshblock->vertices.size(); ++i) {
		const auto& vertex = meshblock->vertices[i];

		// 检查是否已经存在相同 point3D_idx 的顶点
		if (vertex.point3D_idx != -1) {
			auto it = point3d_to_vertex_idx.find(vertex.point3D_idx);
			if (it != point3d_to_vertex_idx.end()) {
				// 已经存在，直接使用现有顶点
				vertex_index_map[i] = it->second;
				continue;
			}
		}

		// 不存在相同 point3D_idx 的顶点，添加新顶点
		size_t new_index = mesh->vertices.size();
		mesh->vertices.push_back(vertex);
		vertex_index_map[i] = new_index;

		// 如果是有效 point3D_idx，添加到映射中
		if (vertex.point3D_idx != -1) {
			point3d_to_vertex_idx[vertex.point3D_idx] = new_index;
		}
	}

	// 清理原始mesh中需要更新的面
	auto face_it = mesh->faces.begin();
	while (face_it != mesh->faces.end()) {
		const auto& face = *face_it;
		bool should_remove = false;

		auto& v1 = mesh->vertices[face.vertex_idx1];
		auto& v2 = mesh->vertices[face.vertex_idx2];
		auto& v3 = mesh->vertices[face.vertex_idx3];

		// 检查mesh表面是否位于更新区域
		if (v1.point3D_idx != -1 && input_data.modified_points.count(v1.point3D_idx)) {
			should_remove = true;
		}
		else if (v2.point3D_idx != -1 && input_data.modified_points.count(v2.point3D_idx)) {
			should_remove = true;
		}
		else if (v3.point3D_idx != -1 && input_data.modified_points.count(v3.point3D_idx)) {
			should_remove = true;
		}

		if (should_remove) {
			face_it = mesh->faces.erase(face_it);
		}
		else {
			++face_it;
		}
	}

	// 处理 meshblock 中的面片，转换顶点索引
	for (const auto& face : meshblock->faces) {
		PlyMeshFace new_face(
			vertex_index_map[face.vertex_idx1],
			vertex_index_map[face.vertex_idx2],
			vertex_index_map[face.vertex_idx3]
		);
		mesh->faces.push_back(new_face);
	}

	// TODO:填充空洞


}

std::shared_ptr<PlyMesh> MeshManager::FilterSurfaceFacets(
	const std::unordered_set<Delaunay::Vertex_handle>& surface_vertices,
	const std::vector<Delaunay::Facet>& surface_facets,
	const std::vector<float>& surface_facet_side_lengths) {

	// 创建新的网格
	std::shared_ptr<PlyMesh> filtered_mesh = std::make_shared<PlyMesh>();
	filtered_mesh->vertices.reserve(surface_vertices.size());

	// 创建顶点索引映射
	std::unordered_map<const Delaunay::Vertex_handle, size_t> surface_vertex_indices;
	surface_vertex_indices.reserve(surface_vertices.size());

	// 复制顶点数据并创建索引映射
	for (const auto& vertex : surface_vertices) {
		filtered_mesh->vertices.emplace_back(
			vertex->point().x(), vertex->point().y(), vertex->point().z(), vertex->info());
		surface_vertex_indices.emplace(vertex, surface_vertex_indices.size());
	}

	filtered_mesh->faces.reserve(surface_facets.size());

	// 配置自适应过滤器 - 修改为使用新的结构体参数访问方式
	AdaptiveFilterConfig filter_config;
	filter_config.method = AdaptiveFilterConfig::FilterMethod::DYNAMIC_CLUSTERING;

	////auto filter_config = AdaptiveFilterConfig::GetStrictDBSCANConfig();

   ////// 可以根据需要微调个别参数
   ////filter_config.min_retention_ratio = 0.92;
   ////filter_config.dbscan_params.eps_factor = 0.06;  // 更严格一些

	// 通过结构体成员访问参数
	filter_config.fixed_threshold_params.threshold = 10.0;
	filter_config.iqr_params.multiplier = 2.0;
	filter_config.zscore_params.threshold = 3.0;
	filter_config.percentile_params.threshold = 96.0;
	filter_config.min_retention_ratio = 0.92;

	// DBSCAN 参数设置
	filter_config.dbscan_params.eps_factor = 0.08;
	filter_config.dbscan_params.min_eps = 0.03;
	filter_config.dbscan_params.min_pts_ratio = 0.015;
	filter_config.dbscan_params.min_min_pts = 8;
	filter_config.dbscan_params.max_min_pts = 80;
	filter_config.dbscan_params.distance_threshold_factor = 2.0;
	filter_config.dbscan_params.extreme_percentile = 99.0;
	filter_config.dbscan_params.main_cluster_std_factor = 2.0;


	AdaptiveEdgeLengthFilter edge_filter(filter_config);
	std::vector<bool> is_edge_outlier = edge_filter.detectOutliers(surface_facet_side_lengths);

	// 应用过滤条件
	size_t filtered_count = 0;
	size_t edge_filtered_count = 0;

	for (size_t i = 0; i < surface_facets.size(); ++i) {
		bool should_filter = false;

		// 1. 自适应边长过滤
		if (is_edge_outlier[i]) {
			should_filter = true;
			edge_filtered_count++;
		}

		if (should_filter) {
			filtered_count++;
			continue;
		}

		// 获取顶点索引
		size_t v0 = surface_vertex_indices.at(surface_facets[i].first->vertex(triangulation.vertex_triple_index(surface_facets[i].second, 0)));
		size_t v1 = surface_vertex_indices.at(surface_facets[i].first->vertex(triangulation.vertex_triple_index(surface_facets[i].second, 1)));
		size_t v2 = surface_vertex_indices.at(surface_facets[i].first->vertex(triangulation.vertex_triple_index(surface_facets[i].second, 2)));

		// 保留面片
		filtered_mesh->faces.emplace_back(v0, v1, v2);
	}

	LOG(INFO) << "Face filtering complete: " << filtered_count << " faces filtered out of "
		<< surface_facets.size() << " total faces";

	return filtered_mesh;
}

// 边界区域自适应过滤函数
std::shared_ptr<PlyMesh> MeshManager::FilterSurfaceFacetsWithBoundaryAdaptation(
	const std::unordered_set<Delaunay::Vertex_handle>& surface_vertices,
	const std::vector<Delaunay::Facet>& surface_facets,
	const std::vector<float>& surface_facet_side_lengths) {

	// 创建新的网格
	std::shared_ptr<PlyMesh> filtered_mesh = std::make_shared<PlyMesh>();
	filtered_mesh->vertices.reserve(surface_vertices.size());

	// 创建顶点索引映射
	std::unordered_map<const Delaunay::Vertex_handle, size_t> surface_vertex_indices;
	surface_vertex_indices.reserve(surface_vertices.size());

	// 复制顶点数据并创建索引映射
	for (const auto& vertex : surface_vertices) {
		filtered_mesh->vertices.emplace_back(
			vertex->point().x(), vertex->point().y(), vertex->point().z(), vertex->info());
		surface_vertex_indices.emplace(vertex, surface_vertex_indices.size());
	}

	// 1. 计算整个模型的 Bounding Box
	Eigen::Vector3f bbox_min(std::numeric_limits<float>::max(),
		std::numeric_limits<float>::max(),
		std::numeric_limits<float>::max());
	Eigen::Vector3f bbox_max(std::numeric_limits<float>::lowest(),
		std::numeric_limits<float>::lowest(),
		std::numeric_limits<float>::lowest());

	for (const auto& vertex : filtered_mesh->vertices) {
		bbox_min.x() = std::min(bbox_min.x(), vertex.x);
		bbox_min.y() = std::min(bbox_min.y(), vertex.y);
		bbox_min.z() = std::min(bbox_min.z(), vertex.z);
		bbox_max.x() = std::max(bbox_max.x(), vertex.x);
		bbox_max.y() = std::max(bbox_max.y(), vertex.y);
		bbox_max.z() = std::max(bbox_max.z(), vertex.z);
	}

	// 2. 计算内部区域边界（90%区域）
	Eigen::Vector3f bbox_size = bbox_max - bbox_min;
	float shrink_ratio = 0.90f;  // 内部区域占90%
	Eigen::Vector3f shrink_offset = bbox_size * (1.0f - shrink_ratio) * 0.5f;
	Eigen::Vector3f inner_bbox_min = bbox_min + shrink_offset;
	Eigen::Vector3f inner_bbox_max = bbox_max - shrink_offset;

	// 3. 区域判断函数
	auto get_region_type = [&](const Eigen::Vector3f& point) -> int {
		if (point.x() >= inner_bbox_min.x() && point.x() <= inner_bbox_max.x() &&
			point.y() >= inner_bbox_min.y() && point.y() <= inner_bbox_max.y() &&
			point.z() >= inner_bbox_min.z() && point.z() <= inner_bbox_max.z()) {
			return 0; // 内部区域（90%）
		}
		return 1; // 边界区域（10%）
		};

	// 4. 计算面片中心和区域类型
	std::vector<Eigen::Vector3f> facet_centers(surface_facets.size());
	std::vector<int> facet_regions(surface_facets.size());

	for (size_t i = 0; i < surface_facets.size(); ++i) {
		const auto& facet = surface_facets[i];
		size_t v0 = surface_vertex_indices.at(facet.first->vertex(triangulation.vertex_triple_index(facet.second, 0)));
		size_t v1 = surface_vertex_indices.at(facet.first->vertex(triangulation.vertex_triple_index(facet.second, 1)));
		size_t v2 = surface_vertex_indices.at(facet.first->vertex(triangulation.vertex_triple_index(facet.second, 2)));

		const auto& p0 = filtered_mesh->vertices[v0];
		const auto& p1 = filtered_mesh->vertices[v1];
		const auto& p2 = filtered_mesh->vertices[v2];

		// 计算面片中心
		Eigen::Vector3f center((p0.x + p1.x + p2.x) / 3.0f,
			(p0.y + p1.y + p2.y) / 3.0f,
			(p0.z + p1.z + p2.z) / 3.0f);
		facet_centers[i] = center;

		// 判断区域类型
		facet_regions[i] = get_region_type(center);
	}

	// 5. 分别收集内部和边界区域的边长数据
	std::vector<float> inner_edge_lengths, boundary_edge_lengths;

	for (size_t i = 0; i < surface_facets.size(); ++i) {
		if (facet_regions[i] == 0) { // 内部区域
			inner_edge_lengths.push_back(surface_facet_side_lengths[i]);
		}
		else { // 边界区域
			boundary_edge_lengths.push_back(surface_facet_side_lengths[i]);
		}
	}

	// 6. 配置不同区域的过滤参数
	struct RegionFilterConfig {
		// 自适应过滤参数
		AdaptiveFilterConfig adaptive_config;
	};

	// 内部区域配置（宽松）
	RegionFilterConfig inner_config;
	inner_config.adaptive_config.method = AdaptiveFilterConfig::FilterMethod::PERCENTILE_BASED;
	inner_config.adaptive_config.percentile_params.threshold = 99.9;
	inner_config.adaptive_config.min_retention_ratio = 0.99;

	// 边界区域配置（严格）
	RegionFilterConfig boundary_config;
	boundary_config.adaptive_config.method = AdaptiveFilterConfig::FilterMethod::COMBINED_METHOD;
	boundary_config.adaptive_config.fixed_threshold_params.threshold = 8.0;
	boundary_config.adaptive_config.iqr_params.multiplier = 1.5;
	boundary_config.adaptive_config.zscore_params.threshold = 2.5;
	boundary_config.adaptive_config.percentile_params.threshold = 92.0;
	boundary_config.adaptive_config.min_retention_ratio = 0.85;

	//// 内部区域配置（宽松）
	//RegionFilterConfig inner_config;
	//inner_config.adaptive_config = AdaptiveFilterConfig::GetLooseDBSCANConfig();
	//inner_config.adaptive_config.method = AdaptiveFilterConfig::FilterMethod::PERCENTILE_BASED;
	//inner_config.adaptive_config.percentile_params.threshold = 99.9;
	//inner_config.adaptive_config.min_retention_ratio = 0.99;

	//// 边界区域配置（严格）
	//RegionFilterConfig boundary_config;
	//boundary_config.adaptive_config = AdaptiveFilterConfig::GetStrictDBSCANConfig();
	//boundary_config.adaptive_config.method = AdaptiveFilterConfig::FilterMethod::COMBINED_METHOD;

	// 7. 分别为两个区域创建过滤器
	AdaptiveEdgeLengthFilter inner_edge_filter(inner_config.adaptive_config);
	AdaptiveEdgeLengthFilter boundary_edge_filter(boundary_config.adaptive_config);

	std::vector<bool> inner_edge_outliers, boundary_edge_outliers;
	if (!inner_edge_lengths.empty()) {
		inner_edge_outliers = inner_edge_filter.detectOutliers(inner_edge_lengths);
	}
	if (!boundary_edge_lengths.empty()) {
		boundary_edge_outliers = boundary_edge_filter.detectOutliers(boundary_edge_lengths);
	}

	// 8. 统计所有边出现的次数（用于边界面片判断）
	std::map<std::pair<size_t, size_t>, int> edge_count;
	for (size_t i = 0; i < surface_facets.size(); ++i) {
		const auto& facet = surface_facets[i];
		for (int j = 0; j < 3; ++j) {
			size_t v1 = triangulation.vertex_triple_index(facet.second, j);
			size_t v2 = triangulation.vertex_triple_index(facet.second, (j + 1) % 3);
			if (v1 > v2) std::swap(v1, v2);
			edge_count[{v1, v2}]++;
		}
	}

	auto is_boundary_facet = [&](size_t facet_idx) -> bool {
		const auto& facet = surface_facets[facet_idx];
		for (int j = 0; j < 3; ++j) {
			size_t v1 = triangulation.vertex_triple_index(facet.second, j);
			size_t v2 = triangulation.vertex_triple_index(facet.second, (j + 1) % 3);
			if (v1 > v2) std::swap(v1, v2);
			if (edge_count[{v1, v2}] == 1) return true;
		}
		return false;
		};

	// 9. 应用区域自适应过滤
	filtered_mesh->faces.reserve(surface_facets.size());

	size_t total_filtered = 0;
	size_t inner_filtered = 0;
	size_t boundary_filtered = 0;
	size_t inner_edge_idx = 0;
	size_t boundary_edge_idx = 0;

	LOG(INFO) << "Bounding Box: Min(" << bbox_min.transpose() << "), Max(" << bbox_max.transpose() << ")";
	LOG(INFO) << "Inner Region: Min(" << inner_bbox_min.transpose() << "), Max(" << inner_bbox_max.transpose() << ")";

	for (size_t i = 0; i < surface_facets.size(); ++i) {
		bool should_filter = false;
		int region = facet_regions[i];

		if (region == 0) { // 内部区域 - 宽松条件
			// 仅进行边长过滤
			if (!inner_edge_lengths.empty() && inner_edge_idx < inner_edge_outliers.size()) {
				if (inner_edge_outliers[inner_edge_idx]) {
					should_filter = true;
				}
				inner_edge_idx++;
			}

			if (should_filter) {
				inner_filtered++;
				total_filtered++;
				continue;
			}

		}
		else { // 边界区域 - 严格条件
			// 边长过滤
			if (!boundary_edge_lengths.empty() && boundary_edge_idx < boundary_edge_outliers.size()) {
				if (boundary_edge_outliers[boundary_edge_idx]) {
					should_filter = true;
				}
				boundary_edge_idx++;
			}

			// 边界面片过滤（仅在边界区域应用）
			if (!should_filter && is_boundary_facet(i)) {
				should_filter = true;
			}

			if (should_filter) {
				boundary_filtered++;
				total_filtered++;
				continue;
			}
		}

		// 保留面片
		const auto& facet = surface_facets[i];
		size_t v0 = surface_vertex_indices.at(facet.first->vertex(triangulation.vertex_triple_index(facet.second, 0)));
		size_t v1 = surface_vertex_indices.at(facet.first->vertex(triangulation.vertex_triple_index(facet.second, 1)));
		size_t v2 = surface_vertex_indices.at(facet.first->vertex(triangulation.vertex_triple_index(facet.second, 2)));

		filtered_mesh->faces.emplace_back(v0, v1, v2);
	}

	LOG(INFO) << "Boundary-adaptive filtering complete:";
	LOG(INFO) << "  - Total faces: " << surface_facets.size();
	LOG(INFO) << "  - Inner region filtered: " << inner_filtered;
	LOG(INFO) << "  - Boundary region filtered: " << boundary_filtered;
	LOG(INFO) << "  - Total filtered: " << total_filtered;
	LOG(INFO) << "  - Retention rate: " << std::fixed << std::setprecision(2)
		<< (100.0 * (surface_facets.size() - total_filtered) / surface_facets.size()) << "%";

	return filtered_mesh;
}

std::vector<double> MeshManager::QualityAssessment()
{
	using TriangleElement = std::tuple<image_t, point2D_t, point2D_t, point2D_t>;
	std::vector<double> weights;

	// 第一步：计算每个面片的质量指标 (GSD, img_track, reprojection_error)
	for (auto& f : mesh->faces) {
		PlyMeshVertex vertex1 = mesh->vertices[f.vertex_idx1];
		PlyMeshVertex vertex2 = mesh->vertices[f.vertex_idx2];
		PlyMeshVertex vertex3 = mesh->vertices[f.vertex_idx3];

		auto& track1 = input_data.points[vertex1.point3D_idx].point2ds;
		auto& track2 = input_data.points[vertex2.point3D_idx].point2ds;
		auto& track3 = input_data.points[vertex3.point3D_idx].point2ds;
		f.img_track = static_cast<double>(track1.size() + track2.size() + track3.size()) / 3.0;

		std::vector<TrackPointTriplet> result = common_image_points(track1, track2, track3);

		// 计算 3D 面积
		double v1x = vertex2.x - vertex1.x, v1y = vertex2.y - vertex1.y, v1z = vertex2.z - vertex1.z;
		double v2x = vertex3.x - vertex1.x, v2y = vertex3.y - vertex1.y, v2z = vertex3.z - vertex1.z;
		double crossX = v1y * v2z - v1z * v2y;
		double crossY = v1z * v2x - v1x * v2z;
		double crossZ = v1x * v2y - v1y * v2x;
		double area3d = 0.5 * std::sqrt(crossX * crossX + crossY * crossY + crossZ * crossZ);
		f.area_weight = area3d;

		// 计算 2D 最大投影面积
		std::vector<double> areas2d;
		double max_area2d = std::numeric_limits<double>::lowest(); // 修正初始值
		for (const auto& triplet : result) {
			const auto& p1 = triplet.point_a.position;
			const auto& p2 = triplet.point_b.position;
			const auto& p3 = triplet.point_c.position;

			double area2d = 0.5 * std::abs(p1.x() * (p2.y() - p3.y()) +
				p2.x() * (p3.y() - p1.y()) + p3.x() * (p1.y() - p2.y()));
			areas2d.push_back(area2d);
			if (area2d > max_area2d) max_area2d = area2d; // 更新最大值
		}

		// 计算 GSD
		if (max_area2d > 0.0 && !std::isinf(max_area2d)) { // 更健壮的检查
			f.gsd = std::sqrt(area3d / max_area2d);
		}
		else {
			f.gsd = std::numeric_limits<double>::max(); // 或者一个默认大值
		}

		// 计算平均重投影误差
		const auto& error1 = input_data.points[vertex1.point3D_idx].pro_error;
		const auto& error2 = input_data.points[vertex2.point3D_idx].pro_error;
		const auto& error3 = input_data.points[vertex3.point3D_idx].pro_error;
		f.reprojection_error = (error1 + error2 + error3) / 3.0;
	}

	// 第二步：基于质量指标计算每个面片的总质量权重 Qtotal
	// 收集所有三角面片的指标值
	std::vector<double> all_gsd, all_img_track, all_warping_score;
	for (const auto& f : mesh->faces) {
		all_gsd.push_back(f.gsd);
		all_img_track.push_back(static_cast<double>(f.img_track));
		all_warping_score.push_back(f.reprojection_error);
	}

	// 计算各指标的 5%-95% 分位数值（过滤极端值）
	double gsd_min = 0.0, gsd_max = 0.0;
	double img_track_min = 0.0, img_track_max = 0.0;
	double warping_score_min = 0.0, warping_score_max = 0.0;

	if (!all_gsd.empty()) {
		std::sort(all_gsd.begin(), all_gsd.end());
		size_t min_idx = static_cast<size_t>(all_gsd.size() * 0.05);
		size_t max_idx = std::min(static_cast<size_t>(all_gsd.size() * 0.95), all_gsd.size() - 1); // 防止越界
		gsd_min = all_gsd[min_idx];
		gsd_max = all_gsd[max_idx];
	}
	if (!all_img_track.empty()) {
		std::sort(all_img_track.begin(), all_img_track.end());
		size_t min_idx = static_cast<size_t>(all_img_track.size() * 0.05);
		size_t max_idx = std::min(static_cast<size_t>(all_img_track.size() * 0.95), all_img_track.size() - 1);
		img_track_min = all_img_track[min_idx];
		img_track_max = all_img_track[max_idx];
	}
	if (!all_warping_score.empty()) {
		std::sort(all_warping_score.begin(), all_warping_score.end());
		size_t min_idx = static_cast<size_t>(all_warping_score.size() * 0.05);
		size_t max_idx = std::min(static_cast<size_t>(all_warping_score.size() * 0.95), all_warping_score.size() - 1);
		warping_score_min = all_warping_score[min_idx];
		warping_score_max = all_warping_score[max_idx];
	}

	// 设置权重
	double w1 = 0.1;  // GSD权重
	double w2 = 0.8;  // img_track权重
	double w3 = 0.1;  // warping_score权重

	// 对每个三角面片归一化并计算总质量指标
	for (auto& f : mesh->faces) {
		double range_gsd = (gsd_max - gsd_min);
		double range_img = (img_track_max - img_track_min);
		double range_warp = (warping_score_max - warping_score_min);
		if (range_gsd <= 0.0) range_gsd = 1.0;
		if (range_img <= 0.0) range_img = 1.0;
		if (range_warp <= 0.0) range_warp = 1.0;

		double normalized_gsd = (gsd_max - f.gsd) / range_gsd;
		double normalized_img_track = (f.img_track - img_track_min) / range_img;
		double normalized_warping_score = (warping_score_max - f.reprojection_error) / range_warp;

		normalized_gsd = std::max(0.0, std::min(1.0, normalized_gsd));
		normalized_img_track = std::max(0.0, std::min(1.0, normalized_img_track));
		normalized_warping_score = std::max(0.0, std::min(1.0, normalized_warping_score));

		f.Qtotal = w1 * normalized_gsd +
			w2 * normalized_img_track +
			w3 * normalized_warping_score;
		weights.push_back(f.Qtotal);
	}

	return weights;
}

// 渐进式连续构建Mesh，权重部分累积，提取部分重新构建
void MeshManager::CreateMeshSimulatly(const Reconstruction& reconstruction) {
	StackTimer timer;

	timer.start("总耗时");
	input_data.ClearCaches();
	timer.start("读取重建数据");
	input_data.CopyFromSparseReconstruction(reconstruction);
	timer.stop();
	timer.start("创建Delaunay三角剖分");
	input_data.CreateSubSampledDelaunayTriangulation(options.max_proj_dist, options.max_depth_dist, triangulation);
	timer.stop();

	//LOG(INFO) << "Initializing ray tracer...";
	timer.start("初始化权重计算器");
	ray_caster = new DelaunayTriangulationRayCaster(triangulation);
	edge_weight_computer = new DelaunayMeshingEdgeWeightComputer(triangulation, options.visibility_sigma, options.distance_sigma_factor);
	timer.stop();

	//LOG(INFO) << "Initializing graph optimization...";
	timer.start("计算所有光线权重图");

	CellGraphData cell_graph_data;
	cell_graph_data.reserve(triangulation.number_of_cells());
	for (auto it = triangulation.all_cells_begin();
		it != triangulation.all_cells_end();
		++it) {
		cell_graph_data.emplace(it, DelaunayCellData(cell_graph_data.size()));
	}

	const int num_threads = GetEffectiveNumThreads(options.num_threads);
	ThreadPool thread_pool(num_threads);
	JobQueue<CellGraphData> result_queue(num_threads);

	// Function that accumulates edge weights in the s-t graph for a single image.
	auto IntegreateImage = [&](const size_t image_id) {
		// Accumulated weights for the current image only.
		CellGraphData image_cell_graph_data;

		// Image that is integrated into s-t graph.
		const auto& image = input_data.images[image_id];
		const K::Point_3 image_position = EigenToCGAL(image.proj_center);

		// Intersections between viewing rays and Delaunay triangulation.
		std::vector<DelaunayTriangulationRayCaster::Intersection> intersections;

		// Iterate through all image observations and integrate them into the graph.
		for (const auto& point_id : image.point_ids) {
			const auto& point = input_data.points[point_id];

			// Likelihood of the point observation.
			const double alpha = edge_weight_computer->ComputeVisibilityProb(
				point.num_visible_images * point.num_visible_images);

			const K::Point_3 point_position = EigenToCGAL(point.position);
			const K::Ray_3 viewing_ray = K::Ray_3(image_position, point_position);
			const K::Vector_3 viewing_direction = point_position - image_position;
			const K::Vector_3 viewing_direction_normalized =
				viewing_direction / std::sqrt(viewing_direction.squared_length());
			const K::Vector_3 viewing_direction_epsilon =
				0.001 * edge_weight_computer->DistanceSigma() *
				viewing_direction_normalized;

			// Find intersected facets between image and point.
			ray_caster->CastRaySegment(
				K::Segment_3(image_position,
					point_position - viewing_direction_epsilon),
				&intersections);

			// Accumulate source weights for cell containing image.
			if (!intersections.empty()) {
				image_cell_graph_data[intersections.front().facet.first]
					.source_weight += alpha;
			}

			// Accumulate edge weights from image to point.
			for (const auto& intersection : intersections) {
				image_cell_graph_data[intersection.facet.first]
					.edge_weights[intersection.facet.second] +=
					alpha * edge_weight_computer->ComputeDistanceProb(
						intersection.target_distance_squared);
			}

			// Accumulate edge weights from point to extended point
			// and accumulate sink weight of the cell inside the surface.

			{
				// Find the first facet that is intersected by the extended ray behind
				// the observed point. Then accumulate the edge weight of that facet
				// and accumulate the sink weight of the cell behind that facet.

				const Delaunay::Cell_handle behind_point_cell =
					triangulation.locate(point_position + viewing_direction_epsilon);

				int behind_neighbor_idx = -1;
				double behind_distance_squared = 0.0;
				for (int neighbor_idx = 0; neighbor_idx < 4; ++neighbor_idx) {
					const K::Triangle_3 triangle =
						triangulation.triangle(behind_point_cell, neighbor_idx);

					K::Point_3 inter_point;
					if (CGAL::assign(inter_point,
						CGAL::intersection(viewing_ray, triangle))) {
						const double distance_squared =
							(inter_point - point_position).squared_length();
						if (distance_squared > behind_distance_squared) {
							behind_distance_squared = distance_squared;
							behind_neighbor_idx = neighbor_idx;
						}
					}
				}

				if (behind_neighbor_idx >= 0) {
					image_cell_graph_data[behind_point_cell]
						.edge_weights[behind_neighbor_idx] +=
						alpha *
						edge_weight_computer->ComputeDistanceProb(behind_distance_squared);

					const auto& inside_cell =
						behind_point_cell->neighbor(behind_neighbor_idx);
					image_cell_graph_data[inside_cell].sink_weight += alpha;
				}
			}
		}
		result_queue.Push(std::move(image_cell_graph_data));
		};

	// Add first batch of images to the thread job queue.
	std::vector<image_t> image_vector(input_data.modified_images.begin(), input_data.modified_images.end());
	size_t image_idx = 0;
	const size_t init_num_tasks =
		std::min(input_data.images.size(), thread_pool.NumThreads());
	for (; image_idx < init_num_tasks; ++image_idx) {
		thread_pool.AddTask(IntegreateImage, image_vector[image_idx]);
	}

	// Pop the integrated images from the thread job queue and integrate their
	// accumulated weights into the global graph.
	for (size_t i = 0; i < input_data.modified_images.size(); ++i) {

		// Push the next image to the queue.
		if (image_idx < input_data.modified_images.size()) {
			thread_pool.AddTask(IntegreateImage, image_idx);
			image_idx += 1;
		}

		// Pop the next results from the queue.
		auto result = result_queue.Pop();

		// Accumulate the weights of the image into the global graph.
		const auto& image_cell_graph_data = result.Data();
		for (const auto& image_cell_data : image_cell_graph_data) {
			auto& cell_data = cell_graph_data.at(image_cell_data.first);
			cell_data.sink_weight += image_cell_data.second.sink_weight;
			cell_data.source_weight += image_cell_data.second.source_weight;
			for (size_t j = 0; j < cell_data.edge_weights.size(); ++j) {
				cell_data.edge_weights[j] += image_cell_data.second.edge_weights[j];
			}
		}
	}

	// Setup the min-cut (max-flow) graph optimization.

	// Each oriented facet in the Delaunay triangulation corresponds to a directed
	// edge and each cell corresponds to a node in the graph.
	MinSTGraphCut<size_t, float> graph_cut(cell_graph_data.size());

	// Iterate all cells in the triangulation.
	for (auto& cell_data : cell_graph_data) {
		graph_cut.AddNode(cell_data.second.index,
			cell_data.second.source_weight,
			cell_data.second.sink_weight);

		// Iterate all facets of the current cell to accumulate edge weight.
		for (int i = 0; i < 4; ++i) {
			// Compose the current facet.
			const Delaunay::Facet facet = std::make_pair(cell_data.first, i);

			// Extract the mirrored facet of the current cell (opposite orientation).
			const Delaunay::Facet mirror_facet = triangulation.mirror_facet(facet);
			const auto& mirror_cell_data = cell_graph_data.at(mirror_facet.first);

			// Avoid duplicate edges in graph.
			if (cell_data.second.index < mirror_cell_data.index) {
				continue;
			}

			// Implementation of geometry visualized in Figure 9 in P. Labatut, J‐P.
			// Pons, and R. Keriven. "Robust and efficient surface reconstruction from
			// range data." Computer graphics forum, 2009.
			const double edge_shape_weight =
				options.quality_regularization *
				(1.0 -
					std::min(ComputeCosFacetCellAngle(facet),
						ComputeCosFacetCellAngle(mirror_facet)));

			const float forward_edge_weight =
				cell_data.second.edge_weights[facet.second] + edge_shape_weight;
			const float backward_edge_weight =
				mirror_cell_data.edge_weights[mirror_facet.second] +
				edge_shape_weight;

			graph_cut.AddEdge(cell_data.second.index,
				mirror_cell_data.index,
				forward_edge_weight,
				backward_edge_weight);
		}
	}

	timer.stop();

	// Extract the surface facets as the oriented min-cut of the graph.
	timer.start("运行graph-cut optimization");
	graph_cut.Compute();
	timer.stop();

	timer.start("提取mesh表面");
	std::unordered_set<Delaunay::Vertex_handle> surface_vertices;
	std::vector<Delaunay::Facet> surface_facets;
	std::vector<float> surface_facet_side_lengths;

	for (auto it = triangulation.finite_facets_begin();
		it != triangulation.finite_facets_end();
		++it) {
		const auto& cell_data = cell_graph_data.at(it->first);
		const auto& mirror_cell_data =
			cell_graph_data.at(it->first->neighbor(it->second));

		// Obtain labeling after the graph-cut.
		const bool cell_is_source = graph_cut.IsConnectedToSource(cell_data.index);
		const bool mirror_cell_is_source =
			graph_cut.IsConnectedToSource(mirror_cell_data.index);

		// The surface is equal to the location of the cut, which is at the
		// transition between source and sink nodes.
		if (cell_is_source == mirror_cell_is_source) {
			continue;
		}

		// Remember all unique vertices of the surface mesh.
		for (int i = 0; i < 3; ++i) {
			const auto& vertex =
				it->first->vertex(triangulation.vertex_triple_index(it->second, i));
			surface_vertices.insert(vertex);
		}

		// Determine maximum side length of facet.
		const K::Triangle_3 triangle = triangulation.triangle(*it);
		const float max_squared_side_length =
			std::max({ (triangle[0] - triangle[1]).squared_length(),
					  (triangle[0] - triangle[2]).squared_length(),
					  (triangle[1] - triangle[2]).squared_length() });
		surface_facet_side_lengths.push_back(std::sqrt(max_squared_side_length));

		// Remember surface mesh facet and make sure it is oriented correctly.
		if (cell_is_source) {
			surface_facets.push_back(*it);
		}
		else {
			surface_facets.push_back(triangulation.mirror_facet(*it));
		}
	}


	//LOG(INFO) << "Creating surface mesh model...";
	std::unordered_map<const Delaunay::Vertex_handle, size_t>
		surface_vertex_indices;
	surface_vertex_indices.reserve(surface_vertices.size());
	mesh->clear();
	mesh->vertices.reserve(surface_vertices.size());
	for (const auto& vertex : surface_vertices) {
		mesh->vertices.emplace_back(
			vertex->point().x(), vertex->point().y(), vertex->point().z(), vertex->info());
		surface_vertex_indices.emplace(vertex, surface_vertex_indices.size());
	}

	const float max_facet_side_length =
		options.max_side_length_factor *
		Percentile(std::vector<float>(surface_facet_side_lengths),
			options.max_side_length_percentile);

	mesh->faces.reserve(surface_facets.size());

	for (size_t i = 0; i < surface_facets.size(); ++i) {
		// Note that skipping some of the facets here means that there will be
		// some unused vertices in the final mesh.
		if (surface_facet_side_lengths[i] > max_facet_side_length) {
			continue;
		}

		const auto& facet = surface_facets[i];
		mesh->faces.emplace_back(
			surface_vertex_indices.at(facet.first->vertex(
				triangulation.vertex_triple_index(facet.second, 0))),
			surface_vertex_indices.at(facet.first->vertex(
				triangulation.vertex_triple_index(facet.second, 1))),
			surface_vertex_indices.at(facet.first->vertex(
				triangulation.vertex_triple_index(facet.second, 2))));
	}
	timer.stop();
	timer.stop();
	timer.printExecutionTimes();
}

//生成渲染数据
void MeshManager::GenerateRenderMeshData() {
	if (renderdata) renderdata.reset();
	std::shared_ptr<RenderData> rendata = std::make_shared<RenderData>();
	using TriangleElement = std::tuple<image_t, point2D_t, point2D_t, point2D_t>;

	//// 渲染基础Mesh网格
	std::set<std::pair<size_t, size_t>> drawnEdges;
	// 生成统一浅蓝色的网格
	Eigen::Vector4f light_blue_color(0.678f, 0.847f, 0.902f, 1.0f); // 浅蓝色 RGBA
	for (const auto& f : mesh->faces) {
		const auto& v0 = mesh->vertices[f.vertex_idx1];
		const auto& v1 = mesh->vertices[f.vertex_idx2];
		const auto& v2 = mesh->vertices[f.vertex_idx3];
		// 渲染面片
		rendata->mesh_data.emplace_back(TrianglePainterData{
			v0.x, v0.y, v0.z, light_blue_color(0), light_blue_color(1), light_blue_color(2), light_blue_color(3),
			v1.x, v1.y, v1.z, light_blue_color(0), light_blue_color(1), light_blue_color(2), light_blue_color(3),
			v2.x, v2.y, v2.z, light_blue_color(0), light_blue_color(1), light_blue_color(2), light_blue_color(3)
			});
		// 渲染边
		std::vector<std::pair<size_t, size_t>> edges = {
				{f.vertex_idx1,f.vertex_idx2},
				{f.vertex_idx2,f.vertex_idx3},
				{f.vertex_idx1,f.vertex_idx3},
		};
		for (auto& edge : edges) {
			if (edge.first > edge.second) {
				std::swap(edge.first, edge.second);
			}
			if (drawnEdges.find(edge) == drawnEdges.end()) {
				drawnEdges.insert(edge);
				const auto& edgeFirst = mesh->vertices[edge.first];
				const auto& edgeSecond = mesh->vertices[edge.second];
				rendata->mesh_lines.emplace_back(LinePainterData{
					edgeFirst.x,edgeFirst.y,edgeFirst.z,0.0 , 0.7 , 0.0 , 1.0,
					edgeSecond.x,edgeSecond.y,edgeSecond.z,0.0 , 0.7 , 0.0 , 1.0
					});
			}
		}
	}

	// 质量评估计算，生成渲染数据
	std::vector<double> quantiles;
	auto weights = QualityAssessment();
	auto colors = generateColorsWithQuantiles(weights, quantiles); // 填充 quantiles
	for (size_t idx = 0; idx < mesh->faces.size(); ++idx) {
		const auto& f = mesh->faces[idx];
		const auto& v0 = mesh->vertices[f.vertex_idx1];
		const auto& v1 = mesh->vertices[f.vertex_idx2];
		const auto& v2 = mesh->vertices[f.vertex_idx3];
		rendata->quality_data.emplace_back(TrianglePainterData{
			v0.x, v0.y, v0.z, colors[idx](0), colors[idx](1), colors[idx](2), colors[idx](3),
			v1.x, v1.y, v1.z, colors[idx](0), colors[idx](1), colors[idx](2), colors[idx](3),
			v2.x, v2.y, v2.z, colors[idx](0), colors[idx](1), colors[idx](2), colors[idx](3)
			});
	}

	// 目前显示已飞行的航线
	std::vector<std::pair<image_t, const DelaunayMeshingInput::Image*>> images_order;
	images_order.reserve(input_data.images.size());
	for (auto& [key, img] : input_data.images) {
		images_order.emplace_back(key, &img);
	}
	std::sort(images_order.begin(), images_order.end(),
		[](const auto& a, const auto& b) {
			return a.first < b.first;
		});
	const Eigen::Vector4f Red{ 1.0f,0.0f,1.0f,1.0f };
	const Eigen::Vector4f Orange{ 1.0f, 0.647f, 0.0f,1.0f };
	const Eigen::Vector4f Green{ 0.0f, 1.0f, 0.0f,1.0f };
	Eigen::Vector3f previous_cam_center;
	bool initialized = false;
	for (auto& [id, img] : images_order) {
		const Eigen::Vector3f cam_center = img->proj_center;
		Eigen::Matrix3f R = img->proj_matrix.block<3, 3>(0, 0);
		Eigen::Vector3f cam_dir = R.transpose() * Eigen::Vector3f(0, 0, 1);
		cam_dir.normalize();
		float line_length = 1.0f;
		Eigen::Vector3f line_end = cam_center + line_length * cam_dir;
		rendata->guide_points.emplace_back(PointPainterData{
		cam_center.x(),cam_center.y(),cam_center.z(),Red.x(),Red.y(),Red.z(),Red.w()
			});
		rendata->guide_directions.emplace_back(LinePainterData{
			cam_center.x(),cam_center.y(),cam_center.z(),Green.x(),Green.y(),Green.z(),Green.w(),
			line_end.x(),line_end.y(),line_end.z(),Green.x(),Green.y(),Green.z(),Green.w()
			});

		if (initialized) {
			rendata->guide_lines.emplace_back(LinePainterData{
			previous_cam_center.x(),previous_cam_center.y(),previous_cam_center.z(),Orange.x(),Orange.y(),Orange.z(),Orange.w(),
			cam_center.x(),cam_center.y(),cam_center.z(),Orange.x(),Orange.y(),Orange.z(),Orange.w()
				});
		}
		else
		{
			initialized = true;
		}
		previous_cam_center = cam_center;
	}

	renderdata = rendata;
}

void MeshManager::WriteDelaunayTriangulationPly(const std::string& path,
	const Delaunay& triangulation) {
	std::fstream file(path, std::ios::out);

	file << "ply" << std::endl;
	file << "format ascii 1.0" << std::endl;
	file << "element vertex " << triangulation.number_of_vertices() << std::endl;
	file << "property float x" << std::endl;
	file << "property float y" << std::endl;
	file << "property float z" << std::endl;
	file << "element edge " << triangulation.number_of_finite_edges()
		<< std::endl;
	file << "property int vertex1" << std::endl;
	file << "property int vertex2" << std::endl;
	file << "element face " << triangulation.number_of_finite_facets()
		<< std::endl;
	file << "property list uchar int vertex_index" << std::endl;
	file << "end_header" << std::endl;

	std::unordered_map<const Delaunay::Vertex_handle, size_t> vertex_indices;
	vertex_indices.reserve(triangulation.number_of_vertices());
	for (auto it = triangulation.finite_vertices_begin();
		it != triangulation.finite_vertices_end();
		++it) {
		vertex_indices.emplace(it, vertex_indices.size());
		file << it->point().x() << " " << it->point().y() << " " << it->point().z()
			<< std::endl;
	}

	for (auto it = triangulation.finite_edges_begin();
		it != triangulation.finite_edges_end();
		++it) {
		file << vertex_indices.at(it->first->vertex(it->second)) << " "
			<< vertex_indices.at(it->first->vertex(it->third)) << std::endl;
	}

	for (auto it = triangulation.finite_facets_begin();
		it != triangulation.finite_facets_end();
		++it) {
		file << "3 "
			<< vertex_indices.at(it->first->vertex(
				triangulation.vertex_triple_index(it->second, 0)))
			<< " "
			<< vertex_indices.at(it->first->vertex(
				triangulation.vertex_triple_index(it->second, 1)))
			<< " "
			<< vertex_indices.at(it->first->vertex(
				triangulation.vertex_triple_index(it->second, 2)))
			<< std::endl;
	}
}


