#pragma once
#include "../Estimator/IncrementalMapper.h"
#include "BaseTask.h"

enum class ReconstructType { AddModel, AddImage };

class ReconstructTask : public BaseTask
{
	Q_OBJECT

public:
	explicit ReconstructTask(
		WorkData* data,
		quint32 p_imageID,
		quint32 p_modelID,
		ReconstructType p_type,
		QObject* parent = nullptr)
		: BaseTask(data, parent)
		, imageID(p_imageID)
		, modelID(p_modelID)
		, type(p_type) {}
	void run() override;
	bool Reconstruct(bool init = false);
	bool InitializeEmptyModel(const IncrementalMapper::Options& options, Reconstruction& model, IncrementalMapper& mapper);
	void LocalBundleAdjustment(const IncrementalMapper::Options& options, size_t imageID, IncrementalMapper& mapper);
	void GlobalBundleAdjustment(const IncrementalMapper::Options& options, IncrementalMapper& mapper);
	size_t TriangulateImage(const IncrementalMapper::Options& options, size_t imageID, IncrementalMapper& mapper);
	size_t FilterPoints(const IncrementalMapper::Options& options, IncrementalMapper& mapper);
	size_t FilterImages(const IncrementalMapper::Options& options, IncrementalMapper& mapper);
	void ExtractColors(const IncrementalMapper::Options& options, size_t imageID, Reconstruction& model);
	void CopyEssentialReconstruction();
signals:
	void reconstructTaskFinished(quint32 p_imageID);
	void reconstructionInitial(quint32 p_modelID, bool success);
	void registedInitialImagePair(quint32 imageID1, quint32 imageID2);
	void registedSuccess(quint32 p_imageID);

private:
	quint32 imageID;
	quint32 modelID;
	ReconstructType type;
};