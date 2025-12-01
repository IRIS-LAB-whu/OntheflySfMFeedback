#pragma once
#include "BaseTask.h"

class ExtractTask : public BaseTask
{
	Q_OBJECT

public:
	explicit ExtractTask(WorkData* data,
		QString p_imagePath,
		quint32 p_imageid,
		QObject* parent = nullptr)
		: BaseTask(data, parent)
		, imagePath(p_imagePath)
		, imageid(p_imageid) {}

	void run() override;
signals:
	void localExtractFinished(QString p_imagePath);

private:
	QString imagePath;
	quint32 imageid;
};