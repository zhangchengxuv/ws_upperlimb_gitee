#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace upperlimb_robot
{

static constexpr std::size_t AXIS_NUM = 20;

// ===================== 控制模式定义 =====================
enum ControlMode : int
{
  MODE_IDLE = 0,
  MODE_LEFT_PRESET_POSITION = 1,
  MODE_RIGHT_PRESET_POSITION = 2,
  MODE_ACTIVE = 3,
};

// ===================== 电机状态缓存 =====================
// 对外统一使用真实物理量：
// position : deg
// velocity : deg/s
// torque   : Nm
struct MotorStateCache
{
  std::array<double, AXIS_NUM> position{};
  std::array<double, AXIS_NUM> velocity{};
  std::array<double, AXIS_NUM> torque{};
  std::array<bool, AXIS_NUM> enable{};
};

// ===================== 控制输出缓存 =====================
// 对外统一使用真实物理量：
// target_position    : deg
// target_velocity    : deg/s
// target_torque      : Nm
// target_acc/dec     : deg/s^2
// target_smooth_time : ms
struct ControlOutputCache
{
  std::array<double, AXIS_NUM> target_position{};
  std::array<double, AXIS_NUM> target_velocity{};
  std::array<double, AXIS_NUM> target_torque{};

  std::array<double, AXIS_NUM> target_acc{};
  std::array<double, AXIS_NUM> target_dec{};
  std::array<short, AXIS_NUM> target_smooth_time{};

  std::array<bool, AXIS_NUM> use_position{};
  std::array<bool, AXIS_NUM> use_velocity{};
  std::array<bool, AXIS_NUM> use_torque{};

  bool valid{false};
};

// ===================== 单位换算接口 =====================
// 这些接口用于在“关节物理量”和“控制器原始量”之间换算。
// 其中：
// - 关节角单位：deg
// - 关节角速度单位：deg/s
// - 关节角加速度单位：deg/s^2
// - 力矩单位：Nm
// - 控制器位置单位：pulse
// - 控制器速度单位：pulse/ms
// - 控制器加速度单位：pulse/ms^2

double jointDegToPulse(int axis, double joint_deg);
double pulseToJointDeg(int axis, double pulse);

double jointSpeedDegPerSecToPulsePerMs(int axis, double joint_speed_deg_per_sec);
double pulsePerMsToJointDegPerSec(int axis, double pulse_per_ms);

double jointAccelDegPerSec2ToPulsePerMs2(int axis, double joint_acc_deg_per_sec2);
double pulsePerMs2ToJointDegPerSec2(int axis, double pulse_per_ms2);

double torqueRawToNm(int axis, double raw);
double torqueNmToRaw(int axis, double torque_nm);

// ===================== 控制算法接口 =====================
void runLeftPresetPosition(const MotorStateCache &state, ControlOutputCache &output);
void runRightPresetPosition(const MotorStateCache &state, ControlOutputCache &output);

void runIdle(const MotorStateCache &state, ControlOutputCache &output);
void runActiveMode(const MotorStateCache &state, ControlOutputCache &output);

} // namespace upperlimb_robot