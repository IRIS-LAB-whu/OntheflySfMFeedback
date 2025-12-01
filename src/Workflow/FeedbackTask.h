#pragma once
#include "BaseTask.h"



class FeedbackTask : public BaseTask
{
	Q_OBJECT

public:
	explicit FeedbackTask(
		WorkData* data,
		quint32 p_model,
		QObject* parent = nullptr)
		: BaseTask(data, parent),
		modelID(p_model) {}
	void run() override;
signals:

public slots:

private:
	quint32 modelID;
};