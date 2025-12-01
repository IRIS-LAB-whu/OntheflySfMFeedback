#include "../Base/timer.h"
#include "../Feature/Bitmap.h"
#include "ExtractTask.h"
#include "WorkData.h"


// TODO: 异常处理
void ExtractTask::run()
{
	std::string imagePathstr = imagePath.toStdString();
	m_data->setImageStatus(imagePathstr, CImageStatusFlag::CExtracting);
	Timer ExtractionTimer;
	ExtractionTimer.Start();
	CSIFTExtractionOptions options;
	CSIFTGPUExtractor* SIFTGPUExtractor = new CSIFTGPUExtractor(options);
	FeatureKeypoints keypoints;
	FeatureDescriptors descriptors;
	LOG(INFO) << "[Extraction] Running sift extract: " << imagePathstr;
	Bitmap bitmap = m_data->ReadBitmap(imageid);
	if (SIFTGPUExtractor->Extract(bitmap, keypoints, descriptors))
	{
		const size_t imageID = m_data->getImageID(imagePathstr);
		const size_t cameraID = m_data->DatabaseReadImage(imageID).CameraId();
		ScaleKeypoints(bitmap, m_data->DatabaseReadCamera(cameraID), &keypoints);
		m_data->DatabaseWriteKeypoints(imageID, keypoints);
		m_data->DatabaseWriteDescriptors(imageID, descriptors);
		m_data->setImageStatus(imagePathstr, CImageStatusFlag::CExtracted);

		const size_t ExtractionTime = ExtractionTimer.ElapsedMicroSeconds() / 1000;
		m_data->SIFTExtractionTimes.at(imageID) += ExtractionTime;

		LOG(INFO) << "[Extraction] Image " << imagePathstr << " extracted successed!";

		delete SIFTGPUExtractor;
		m_data->ReleaseBitmap(imageid);

		emit localExtractFinished(imagePath);
	}
}
