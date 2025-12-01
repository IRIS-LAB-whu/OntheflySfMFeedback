#pragma once
#include <atomic>
#include <chrono>
#include <iostream>
#include <QObject>
#include <QString>
#include <string>
#include <thread>
#include <windows.h>

class Receiver : public QObject
{
	Q_OBJECT
public:
	explicit Receiver(QObject* parent = nullptr);
	~Receiver();

public slots:
	void Start();
	void Stop();

signals:
	void newDataReceived(const QString& value);

private:
	std::string ReadRegistryValue();
	void ClearRegistryValue();
	std::thread loopThread;
	std::atomic<bool> running = false;
};