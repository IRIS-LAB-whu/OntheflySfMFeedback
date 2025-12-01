#pragma once
#include <algorithm>
#include <chrono>
#include <cmath>

class BufferPIDController {
private:
	// PID 参数
	double Kp = 8.0;      // 比例系数：快速响应误差
	double Ki = 0.15;       // 积分系数：消除稳态误差
	double Kd = 80.0;      // 微分系数：抑制震荡

	// 目标值和约束
	const double TARGET_BUFFER = 70.0;
	const double MIN_INPUT_INTERVAL = 800.0;   // 最小输入间隔 (ms)
	const double MAX_INPUT_INTERVAL = 5000.0;  // 最大输入间隔 (ms)

	// PID 状态变量
	double integral = 0.0;
	double prevError = 0.0;
	double inputInterval = 1800.0;  // 初始输入间隔，略慢于平均处理速度

	// 时间跟踪
	long long lastCallTime = 0;     // 上次调用时间戳（毫秒）
	double processSpeed = 1750.0;   // 当前处理速度（毫秒），初始值为中间值
	bool isFirstCall = true;

	// 积分抗饱和
	const double MAX_INTEGRAL = 5000.0;
	const double MIN_INTEGRAL = -5000.0;

	// 处理速度平滑（可选：使用移动平均）
	const double SPEED_SMOOTH_FACTOR = 0.8;  // 平滑系数，0-1之间，越大越敏感

	/**
	 * 获取当前时间戳（毫秒）
	 */
	long long getCurrentTimeMs() {
		return std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::steady_clock::now().time_since_epoch()
		).count();
	}

public:
	BufferPIDController() {}

	// 自定义 PID 参数
	BufferPIDController(double kp, double ki, double kd)
		: Kp(kp), Ki(ki), Kd(kd) {}

	/**
	 * 在每次处理完一张照片后调用
	 * @param totalNum 总数据量
	 * @param processedNum 已处理数据量
	 */
	void processed(int totalNum, int processedNum) {
		// 1. 获取当前时间并计算处理速度
		long long currentTime = getCurrentTimeMs();

		if (!isFirstCall && lastCallTime > 0) {
			// 计算实际处理速度（两次调用的时间间隔）
			double measuredSpeed = static_cast<double>(currentTime - lastCallTime);

			// 使用指数移动平均平滑处理速度（减少噪声影响）
			processSpeed = SPEED_SMOOTH_FACTOR * measuredSpeed +
				(1.0 - SPEED_SMOOTH_FACTOR) * processSpeed;

			// 限制 processSpeed 在合理范围内（防止异常值）
			processSpeed = std::clamp(processSpeed, 1000.0, 3000.0);
		}

		lastCallTime = currentTime;

		// 2. 计算当前缓冲区大小（待处理数据量）
		int currentBuffer = totalNum - processedNum;

		// 3. 计算误差（正值表示缓冲区不足，需要加快输入）
		double error = TARGET_BUFFER - currentBuffer;

		// 4. 首次调用特殊处理
		if (isFirstCall) {
			prevError = error;
			isFirstCall = false;

			// 根据初始缓冲区设置初始输入间隔
			if (currentBuffer < 40) {
				inputInterval = 1000.0;  // 缓冲区过少，快速输入
			}
			else if (currentBuffer > 120) {
				inputInterval = 3000.0;  // 缓冲区过多，减慢输入
			}
			return;
		}

		// 5. 时间间隔（使用自动更新的 processSpeed）
		double dt = processSpeed / 1000.0;  // 转换为秒

		// 6. PID 计算
		// P: 比例项 - 对当前误差的即时响应
		double P = Kp * error;

		// I: 积分项 - 消除长期累积误差（带抗饱和）
		integral += error * dt;
		integral = std::clamp(integral, MIN_INTEGRAL, MAX_INTEGRAL);
		double I = Ki * integral;

		// D: 微分项 - 预测误差变化趋势，抑制震荡
		double derivative = (error - prevError) / dt;
		double D = Kd * derivative;

		// 7. PID 输出（注意符号：正输出表示需要减小间隔）
		double pidOutput = P + I + D;

		// 8. 更新输入间隔
		// 误差为正（缓冲区不足）-> pidOutput 为正 -> 减小间隔（加快输入）
		// 误差为负（缓冲区过多）-> pidOutput 为负 -> 增大间隔（减慢输入）
		inputInterval = inputInterval - pidOutput;

		// 9. 限制输入间隔范围
		inputInterval = std::clamp(inputInterval, MIN_INPUT_INTERVAL, MAX_INPUT_INTERVAL);

		// 10. 保存当前误差供下次使用
		prevError = error;
	}

	/**
	 * 获取当前建议的输入间隔时间（毫秒）
	 * @return 下一张照片应该在多久后输入
	 */
	double getInputInterval() const {
		return inputInterval;
	}

	/**
	 * 获取当前测量的处理速度（毫秒）
	 * @return 当前平均处理速度
	 */
	double getProcessSpeed() const {
		return processSpeed;
	}

	/**
	 * 获取当前缓冲区状态信息（用于调试和监控）
	 */
	struct Status {
		double inputInterval;
		double processSpeed;
		double integral;
		double prevError;
	};

	Status getStatus() const {
		return { inputInterval, processSpeed, integral, prevError };
	}

	/**
	 * 重置 PID 控制器状态
	 */
	void reset() {
		integral = 0.0;
		prevError = 0.0;
		inputInterval = 1800.0;
		lastCallTime = 0;
		processSpeed = 1750.0;
		isFirstCall = true;
	}

	/**
	 * 动态调整 PID 参数（用于运行时调优）
	 */
	void setPIDParams(double kp, double ki, double kd) {
		Kp = kp;
		Ki = ki;
		Kd = kd;
	}

	/**
	 * 设置目标缓冲区大小（如果需要动态调整目标）
	 */
	void setTargetBuffer(double target) {
		const_cast<double&>(TARGET_BUFFER) = target;
	}
};