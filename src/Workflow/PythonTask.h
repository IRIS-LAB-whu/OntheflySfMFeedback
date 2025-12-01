#pragma once
#include "BaseTask.h"

enum class PythonTaskMode { GlobalFeatureExtract };

class PythonTask : public BaseTask
{
	Q_OBJECT

public:
	explicit PythonTask(WorkData* data,
		QString p_imagepath,
		PythonTaskMode p_mode,
		QObject* parent = nullptr)
		: BaseTask(data, parent)
		, imagePathQstr(p_imagepath)
		, mode(p_mode) {}

	void run() override;

signals:
	void globalFeatureExtractFinished(QString imagePath);

private:
	QString imagePathQstr;
	PythonTaskMode mode;
};