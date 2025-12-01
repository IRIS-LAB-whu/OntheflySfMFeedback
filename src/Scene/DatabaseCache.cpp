#include "../Base/string.h"
#include "../Base/timer.h"
#include "../Feature/Common.h"
#include "DatabaseCache.h"
#include <unordered_set>

bool DatabaseCache::initialized = false;

void DatabaseCache::Create(
	const Database& database,
	const size_t min_num_matches,
	const bool ignore_watermarks,
	const std::set<size_t>& p_image_ids) {

	//////////////////////////////////////////////////////////////////////////////
	// Load cameras
	//////////////////////////////////////////////////////////////////////////////

	Timer timer;
	timer.Start();

	{
		std::vector<class Camera> cameras = database.ReadAllCameras();
		cameras_.reserve(cameras.size());
		for (auto& camera : cameras) {
			const camera_t camera_id = camera.CameraId();
			cameras_.emplace(camera_id, std::move(camera));
		}
	}

	//////////////////////////////////////////////////////////////////////////////
	// Load matches
	//////////////////////////////////////////////////////////////////////////////

	timer.Restart();

	std::vector<image_pair_t> image_pair_ids;
	std::vector<TwoViewGeometry> two_view_geometries;
	database.ReadTwoViewGeometries(&image_pair_ids, &two_view_geometries);

	auto UseInlierMatchesCheck = [min_num_matches, ignore_watermarks](
		const TwoViewGeometry& two_view_geometry) {
			return static_cast<size_t>(two_view_geometry.inlier_matches.size()) >=
				min_num_matches &&
				(!ignore_watermarks ||
					two_view_geometry.config != TwoViewGeometry::WATERMARK);
		};

	//////////////////////////////////////////////////////////////////////////////
	// Load images
	//////////////////////////////////////////////////////////////////////////////

	timer.Restart();

	std::unordered_set<image_t> image_ids;

	{
		std::vector<class Image> images = database.ReadAllImages();
		const size_t num_images = images.size();

		// Determines for which images data should be loaded.
		if (p_image_ids.empty()) {
			for (const auto& image : images) {
				image_ids.insert(image.ImageId());
			}
		}
		else {
			for (const auto& image : images) {
				if (p_image_ids.count(image.ImageId()) > 0) {
					image_ids.insert(image.ImageId());
				}
			}
		}

		// Collect all images that are connected in the correspondence graph.
		std::unordered_set<image_t> connected_image_ids;
		connected_image_ids.reserve(image_ids.size());
		for (size_t i = 0; i < image_pair_ids.size(); ++i) {
			if (UseInlierMatchesCheck(two_view_geometries[i])) {
				image_t image_id1;
				image_t image_id2;
				Database::PairIdToImagePair(image_pair_ids[i], &image_id1, &image_id2);
				if (image_ids.count(image_id1) > 0 && image_ids.count(image_id2) > 0) {
					connected_image_ids.insert(image_id1);
					connected_image_ids.insert(image_id2);
				}
			}
		}

		// Load images with correspondences and discard images without
		// correspondences, as those images are useless for SfM.
		images_.reserve(connected_image_ids.size());
		for (auto& image : images) {
			const image_t image_id = image.ImageId();
			if (image_ids.count(image_id) > 0 &&
				connected_image_ids.count(image_id) > 0) {
				image.SetPoints2D(
					FeatureKeypointsToPointsVector(database.ReadKeypoints(image_id)));
				images_.emplace(image_id, std::move(image));
			}
		}
	}

	//////////////////////////////////////////////////////////////////////////////
	// Build correspondence graph
	//////////////////////////////////////////////////////////////////////////////

	timer.Restart();


	correspondence_graph_ = std::make_shared<class CorrespondenceGraph>();

	for (const auto& image : images_) {
		correspondence_graph_->AddImage(image.first,
			image.second.NumPoints2D());
	}

	size_t num_ignored_image_pairs = 0;
	for (size_t i = 0; i < image_pair_ids.size(); ++i) {
		if (UseInlierMatchesCheck(two_view_geometries[i])) {
			image_t image_id1;
			image_t image_id2;
			Database::PairIdToImagePair(image_pair_ids[i], &image_id1, &image_id2);
			if (image_ids.count(image_id1) > 0 && image_ids.count(image_id2) > 0) {
				correspondence_graph_->AddCorrespondences(
					image_id1, image_id2, two_view_geometries[i].inlier_matches);
			}
			else {
				num_ignored_image_pairs += 1;
			}
		}
		else {
			num_ignored_image_pairs += 1;
		}
	}

	correspondence_graph_->Finalize();

	// Set number of observations and correspondences per image.
	for (auto& image : images_) {
		image.second.SetNumObservations(
			correspondence_graph_->NumObservationsForImage(image.first));
		image.second.SetNumCorrespondences(
			correspondence_graph_->NumCorrespondencesForImage(image.first));
	}
	initialized = true;
}

const class Image* DatabaseCache::FindImageWithName(
	const std::string& name) const {
	for (const auto& image : images_) {
		if (image.second.Name() == name) {
			return &image.second;
		}
	}
	return nullptr;
}

void DatabaseCache::AddImage(const Database& database,
	const size_t min_num_matches,
	const bool ignore_watermarks,
	const std::set<image_t>& p_image_ids) {
	//////////////////////////////////////////////////////////////////////////////
	// Add cameras
	//////////////////////////////////////////////////////////////////////////////

	std::vector<class Camera> cameras = database.ReadAllCameras();
	for (auto& camera : cameras) {
		const camera_t camera_id = camera.CameraId();
		if (cameras_.find(camera_id) == cameras_.end()) {
			cameras_.emplace(camera_id, std::move(camera));
		}
	}

	//////////////////////////////////////////////////////////////////////////////
	// Add matches
	//////////////////////////////////////////////////////////////////////////////

	std::vector<image_pair_t> image_pair_ids;
	std::vector<TwoViewGeometry> two_view_geometries;
	database.ReadTwoViewGeometries(&image_pair_ids, &two_view_geometries);

	auto UseInlierMatchesCheck = [min_num_matches, ignore_watermarks](
		const TwoViewGeometry& two_view_geometry) {
			return static_cast<size_t>(two_view_geometry.inlier_matches.size()) >=
				min_num_matches &&
				(!ignore_watermarks ||
					two_view_geometry.config != TwoViewGeometry::WATERMARK);
		};

	//////////////////////////////////////////////////////////////////////////////
	// Add images
	//////////////////////////////////////////////////////////////////////////////
	{
		for (const auto& imageID : p_image_ids) {
			class Image addImage = database.ReadImage(imageID);

			// Collect all images that are connected in the correspondence graph.
			std::unordered_set<image_t> connected_image_ids;
			for (size_t i = 0; i < image_pair_ids.size(); ++i) {
				image_t image_id1;
				image_t image_id2;
				Database::PairIdToImagePair(image_pair_ids[i], &image_id1, &image_id2);
				if (image_id1 != imageID && image_id2 != imageID) continue;
				if (UseInlierMatchesCheck(two_view_geometries[i])) {
					connected_image_ids.insert(image_id1);
					connected_image_ids.insert(image_id2);
				}
			}

			// Load images with correspondences and discard images without
			// correspondences, as those images are useless for SfM.
			if (connected_image_ids.count(imageID) > 0) {
				addImage.SetPoints2D(
					FeatureKeypointsToPointsVector(database.ReadKeypoints(imageID)));
				images_.emplace(imageID, std::move(addImage));
			}
		}
	}

	//////////////////////////////////////////////////////////////////////////////
	// Build correspondence graph
	//////////////////////////////////////////////////////////////////////////////
	correspondence_graph_.reset();
	correspondence_graph_ = std::make_shared<class CorrespondenceGraph>();

	for (const auto& image : images_) {
		correspondence_graph_->AddImage(image.first,
			image.second.NumPoints2D());
	}

	size_t num_ignored_image_pairs = 0;
	for (size_t i = 0; i < image_pair_ids.size(); ++i) {
		if (UseInlierMatchesCheck(two_view_geometries[i])) {
			image_t image_id1;
			image_t image_id2;
			Database::PairIdToImagePair(image_pair_ids[i], &image_id1, &image_id2);
			correspondence_graph_->AddCorrespondences(
				image_id1, image_id2, two_view_geometries[i].inlier_matches);
		}
		else {
			num_ignored_image_pairs += 1;
		}
	}
	correspondence_graph_->Finalize();

	// Set number of observations and correspondences per image.
	for (auto& image : images_) {
		image.second.SetNumObservations(
			correspondence_graph_->NumObservationsForImage(image.first));
		image.second.SetNumCorrespondences(
			correspondence_graph_->NumCorrespondencesForImage(image.first));
	}
}