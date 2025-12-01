#pragma once
#include "BaseTask.h"

class RetrieveTask : public BaseTask
{
	Q_OBJECT

public:
	explicit RetrieveTask(WorkData* data,
		QString p_imagePath,
		QObject* parent = nullptr)
		: BaseTask(data, parent)
		, imagePath(p_imagePath) {}

	void run() override;
signals:
	void retrieveTaskFinished(quint32 p_imageID1, quint32 p_imageID2);

private:
	QString imagePath;
};