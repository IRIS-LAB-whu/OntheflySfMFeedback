#include "Receiver.h"

Receiver::Receiver(QObject* parent)
	: QObject(parent)
{
}

Receiver::~Receiver()
{
	Stop();
}

void Receiver::Start()
{
	if (running) return;
	running = true;
	loopThread = std::thread([this]() {
		ClearRegistryValue();
		while (running) {
			std::string ss_value = ReadRegistryValue();
			QString value = QString::fromStdString(ss_value);
			if (!ss_value.empty()) {
				emit newDataReceived(value);
				ClearRegistryValue();
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
		}
		});
}

void Receiver::Stop()
{
	running = false;
	if (loopThread.joinable())
		loopThread.join();
}

std::string Receiver::ReadRegistryValue()
{
	// 示例代码，实际需根据你的注册表路径和键名调整
	HKEY hKey;
	std::string result;
	if (RegOpenKeyExA(HKEY_CURRENT_USER, "Software\\RTPS\\CamFiTransmit", 0, KEY_READ, &hKey) == ERROR_SUCCESS) {
		char buffer[512] = { 0 };
		DWORD bufferSize = sizeof(buffer);
		if (RegQueryValueExA(hKey, "NewImage", nullptr, nullptr, (LPBYTE)buffer, &bufferSize) == ERROR_SUCCESS) {
			result = buffer;
		}
		RegCloseKey(hKey);
	}
	return result;
}

void Receiver::ClearRegistryValue()
{
	HKEY hKey;
	if (RegOpenKeyExA(HKEY_CURRENT_USER, "Software\\RTPS\\CamFiTransmit", 0, KEY_SET_VALUE, &hKey) == ERROR_SUCCESS) {
		const char* empty = "";
		RegSetValueExA(hKey, "NewImage", 0, REG_SZ, (const BYTE*)empty, 1);
		RegCloseKey(hKey);
	}
}