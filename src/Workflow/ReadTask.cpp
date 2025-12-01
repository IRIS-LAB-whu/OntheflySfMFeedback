#include "../Base/timer.h"
#include "ReadTask.h"
#include "WorkData.h"
#include <string>

using namespace std;

void ReadTask::run()
{
	Camera camera;
	std::string cameraModel;
	Bitmap bitmap;
	Image image;
	ImageReader* imageReader = new ImageReader();

	std::string imagePathstr = imagePath.toStdString();

	Timer imageReaderTimer;
	imageReaderTimer.Start();

	if (imageReader->Read(imagePathstr, camera, cameraModel, image, bitmap) == ImageReader::Status::SUCCESS)
	{
		camera_t cameraID = m_data->DatabaseWriteCamera(camera);
		camera.SetCameraId(cameraID);
		image.SetCameraId(cameraID);
		image_t imageID = m_data->DatabaseWriteImage(image);

		m_data->addImageMap(imageID, imagePathstr);

		const size_t readTime = imageReaderTimer.ElapsedMicroSeconds() / 1000;
		m_data->SIFTExtractionTimes[imageID] = readTime;
		m_data->globalFeatureExtractionTimes[imageID] = 0;
		m_data->globalFeatureRetrievalTimes[imageID] = 0;
		m_data->SIFTMatchingTimes[imageID] = 0;
		m_data->geometricVerificationTimes[imageID] = 0;

		m_data->WriteBitmap(bitmap, imageID);

		m_data->setImageStatus(imagePathstr, CImageStatusFlag::CRead);
		m_data->readImagesNum.store(m_data->DatabaseReadImageNum());
		emit readTaskDone(imageID, imagePath);
	}
	delete imageReader;
}