#include "../Base/timer.h"
#include "EstimateTask.h"
#include "WorkData.h"

void EstimateTask::run()
{
	Timer geometricVerificationTimer;
	geometricVerificationTimer.Start();

	EstimateTwoViewGeometry();

	const size_t geometricVerificationTime = geometricVerificationTimer.ElapsedMicroSeconds() / 1000;
	m_data->geometricVerificationTimes.at(imageID2) += geometricVerificationTime;

	bool existInlierMatches = m_data->DatabaseExistsInlierMatches(imageID1, imageID2);
	const size_t numInlierMatches = (existInlierMatches ? m_data->DatabaseReadTwoViewGeometry(imageID1, imageID2).inlier_matches.size() : 0);

	//LOG(INFO) << "[Estimate] " << imageID1 << " - " << imageID2 << ": " << numInlierMatches;

	if (m_data->numMatchingPairs.at(imageID2) > 0)
	{
		m_data->numMatchingPairs.at(imageID2)--;
	}
	if (m_data->numMatchingPairs.at(imageID2) == 0)
	{
		std::string imagePath = m_data->getImagePath(imageID2);
		m_data->setImageStatus(imagePath, CImageStatusFlag::CTwoViewGeometryEstimated);
		LOG(INFO) << "[Estimate] Image " << imageID2 << " estimated finished!";

		if (m_data->estimatedImagesNum.load() == 0) {
			m_data->AddEstimatedImage(imageID1);
		}
		m_data->AddEstimatedImage(imageID2);
		emit estimateAllFinished(imageID1, imageID2);
	}
}

void EstimateTask::EstimateTwoViewGeometry()
{
	const FeatureKeypoints keypoints1 = m_data->DatabaseReadKeypoints(imageID1);
	const FeatureKeypoints keypoints2 = m_data->DatabaseReadKeypoints(imageID2);
	if (keypoints1.empty() || keypoints2.empty() || !m_data->DatabaseExistsMatches(imageID1, imageID2))
	{
		emit estimateTaskFailed();
		return;
	}
	const std::vector<Eigen::Vector2d> pointsVector1 = FeatureKeypointsToPointsVector(keypoints1);
	const std::vector<Eigen::Vector2d> pointsVector2 = FeatureKeypointsToPointsVector(keypoints2);
	const FeatureMatches matches = m_data->DatabaseReadMatches(imageID1, imageID2);
	if (matches.empty())
	{
		emit estimateTaskFailed();
		return;
	}
	const camera_t cameraid1 = m_data->DatabaseReadImage(imageID1).CameraId();
	const Camera camera1 = m_data->DatabaseReadCamera(cameraid1);
	const camera_t cameraid2 = m_data->DatabaseReadImage(imageID2).CameraId();
	const Camera camera2 = m_data->DatabaseReadCamera(cameraid2);

	// 如果有相机的焦距为0, 会导致双视几何估计的过程存在死循环
	if (abs(camera1.FocalLength()) < 1e-5 || abs(camera2.FocalLength()) < 1e-5)
	{
		emit estimateTaskFailed();
		return;
	}

	TwoViewGeometryOptions options;
	const TwoViewGeometry twoViewGeometry = EstimateTwoViewGeometry_Internal(camera1, pointsVector1, camera2, pointsVector2, matches, options);
	if (!twoViewGeometry.inlier_matches.empty())
	{
		m_data->DatabaseWriteTwoViewGeometry(imageID1, imageID2, twoViewGeometry);
	}
}
