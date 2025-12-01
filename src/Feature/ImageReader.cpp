#include "../Base/config.h"
#include "ImageReader.h"
#include <string>


ImageReader::ImageReader() :defaultFocalLengthFactor(1.2), maxImageSize(4800)
{

}
ImageReader::Status ImageReader::Read(const std::string& imagePath, Camera& camera, std::string& cameraModel, Image& image, Bitmap& bitmap)
{
	const std::string imagePath_ = StringReplace(imagePath, "\\", "/");
	const size_t pos = imagePath_.find_last_of('/');
	const std::string imageName = imagePath_.substr(pos + 1);
	image.SetName(imageName);

	if (!bitmap.Read(imagePath, false))
	{
		return Status::BITMAP_ERROR;
	}
	bitmap.ExifCameraModel(&cameraModel);
	double focalLength = 0.0;
	if (bitmap.ExifFocalLength(&focalLength) && abs(focalLength) > 1e-5)
	{
		camera.SetPriorFocalLength(true);
	}
	else
	{
		focalLength = defaultFocalLengthFactor * std::max(bitmap.Width(), bitmap.Height());
		camera.SetPriorFocalLength(false);
	}

	auto& config = Config::getInstance();
	std::string definecameraModel = config.sfm.cameraModel;
	if (config.sfm.useFocalLength) {
		focalLength = config.sfm.useFactor * std::max(bitmap.Width(), bitmap.Height());
		camera.SetPriorFocalLength(false);
	}
	if (definecameraModel == "SIMPLE_PINHOLE") {
		camera.InitializeWithId(0, focalLength, bitmap.Width(), bitmap.Height());
	}
	else if (definecameraModel == "PINHOLE") {
		camera.InitializeWithId(1, focalLength, bitmap.Width(), bitmap.Height());
	}
	else if (definecameraModel == "SIMPLE_RADIAL") {
		camera.InitializeWithId(2, focalLength, bitmap.Width(), bitmap.Height());
	}
	else if (definecameraModel == "RADIAL") {
		camera.InitializeWithId(3, focalLength, bitmap.Width(), bitmap.Height());
	}
	else {
		// default is SIMPLE_RADIAL
		camera.InitializeWithId(2, focalLength, bitmap.Width(), bitmap.Height());
	}

	if (!camera.VerifyParams())
	{
		return Status::CAMERA_PARAM_ERROR;
	}

	Eigen::Vector3d& translationPrior = image.CamFromWorldPrior().translation;
	if (!bitmap.ExifLatitude(&translationPrior.x()) || !bitmap.ExifLongitude(&translationPrior.y()) || !bitmap.ExifAltitude(&translationPrior.z()))
	{
		translationPrior.setConstant(std::numeric_limits<double>::quiet_NaN());
	}

	if (bitmap.Width() > maxImageSize || bitmap.Height() > maxImageSize)
	{
		const double scale = static_cast<double>(maxImageSize) / std::max(bitmap.Width(), bitmap.Height());
		const int newWidth = static_cast<int>(bitmap.Width() * scale);
		const int newHeight = static_cast<int>(bitmap.Height() * scale);
		bitmap.Rescale(newWidth, newHeight);
	}

	return Status::SUCCESS;
}



