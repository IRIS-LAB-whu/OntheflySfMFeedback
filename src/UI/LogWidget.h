#pragma once

#include <atomic>
#include <condition_variable>
#include <glog/logging.h>
#include <memory>
#include <mutex>
#include <QCheckBox>
#include <QGridLayout>
#include <QMutex>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QString>
#include <QTimer>
#include <queue>
#include <QWidget>
#include <string>
#include <thread>

class LogWidget : public QWidget
{
	Q_OBJECT

public:
	explicit LogWidget(QWidget* parent = nullptr);
	~LogWidget();

	void SetMaxLines(int maxLines);
	void Append(const std::string& message, google::LogSeverity severity);

	void SaveLogToFile(const std::string& filePath);

	// 可选：设置日志文件路径
	void SetLogFilePath(const std::string& filePath);

	class CustomLogSink : public google::LogSink
	{
	public:
		explicit CustomLogSink(LogWidget* widget);
		void send(google::LogSeverity severity, const char* full_filename,
			const char* base_filename, int line,
			const struct ::tm* tm_time,
			const char* message, size_t message_len) override;
	private:
		LogWidget* m_widget;
	};

private slots:
	void Clear();
	void Flush();

private:
	void InitializeUI();
	void InitializeLogging();
	QString GetLogPrefix(google::LogSeverity severity) const;

	// 文件写线程相关
	void StartLogFileThread();
	void StopLogFileThread();

	QPlainTextEdit* m_textEdit;
	QPushButton* m_clearButton;
	QCheckBox* m_autoScrollCheckBox;
	QGridLayout* m_layout;
	QTimer* m_flushTimer;

	QMutex m_bufferMutex;
	std::queue<std::pair<std::string, google::LogSeverity>> m_logBuffer; // UI显示用

	int m_maxLines;

	std::unique_ptr<CustomLogSink> m_logSink;

	// 文件写线程相关成员
	std::thread m_fileThread;
	std::queue<std::pair<std::string, google::LogSeverity>> m_fileLogQueue;
	QMutex m_fileQueueMutex;
	std::atomic<bool> m_fileThreadRunning;
	std::string m_logFileFolder;
};