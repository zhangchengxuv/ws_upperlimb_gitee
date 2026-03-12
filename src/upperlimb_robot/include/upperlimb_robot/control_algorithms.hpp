#pragma once

#include <array>
#include <cstddef>

// 电机状态结构体
// ===================== 电机状态缓存结构体 =====================
struct MotorStateCache
{
    std::array<double, 20> position{}; // 关节位置（可换算成角度）
    std::array<double, 20> velocity{}; // 关节速度
    std::array<double, 20> torque{};   // 关节扭矩
    std::array<bool, 20> enabled{};    // 轴是否使能
};

// ===== 控制模式函数声明 =====
void runIdle(const MotorStateCache &state);

void runPassiveTraining(const MotorStateCache &state);

void runZeroForceMode(const MotorStateCache &state);

void runBilateralMode(const MotorStateCache &state);

void runSlidingModeControl(const MotorStateCache &state);

void runLeftPresetPosition(const MotorStateCache &state);