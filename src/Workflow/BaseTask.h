#pragma once

#include <QFuture>
#include <QObject>
#include <QRunnable>
#include <QtConcurrent/QtConcurrent>

class WorkData;

class BaseTask : public QObject, public QRunnable
{
	Q_OBJECT

public:
	explicit BaseTask(WorkData* data, QObject* parent = nullptr)
		: QObject(parent), m_data(data) {}

	virtual void run() = 0;  // 纯虚函数，具体任务需要实现

signals:
	void taskFinished();

protected:
	WorkData* m_data;  // 共享的数据
};
