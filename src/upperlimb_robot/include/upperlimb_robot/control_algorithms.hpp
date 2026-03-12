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
  MODE_ACTIVE = 2,              // 主动模式（暂时只写框架）
  MODE_LEFT_PRESET_POSITION = 7 // 左侧回预设位置
};

// ===================== 电机状态缓存 =====================
struct MotorStateCache
{
  std::array<double, AXIS_NUM> position{};
  std::array<double, AXIS_NUM> velocity{};
  std::array<double, AXIS_NUM> torque{};
  std::array<bool, AXIS_NUM> enable{};
};

// ===================== 控制输出缓存 =====================
struct ControlOutputCache
{
  // 位置 / 速度 / 力矩目标
  std::array<double, AXIS_NUM> target_position{};
  std::array<double, AXIS_NUM> target_velocity{};
  std::array<double, AXIS_NUM> target_torque{};

  // 位置模式额外参数（给 GTN_SetTrapPrm 用）
  std::array<double, AXIS_NUM> target_acc{};
  std::array<double, AXIS_NUM> target_dec{};
  std::array<short, AXIS_NUM> target_smooth_time{};

  // 是否使用对应控制量
  std::array<bool, AXIS_NUM> use_position{};
  std::array<bool, AXIS_NUM> use_velocity{};
  std::array<bool, AXIS_NUM> use_torque{};

  bool valid{false};
};

// ===================== 算法接口 =====================
void runIdle(const MotorStateCache &state, ControlOutputCache &output);

// 主动模式：先只写框架，不写具体实现
void runActiveMode(const MotorStateCache &state, ControlOutputCache &output);

// 左侧回预设位置
void runLeftPresetPosition(const MotorStateCache &state, ControlOutputCache &output);

} // namespace upperlimb_robot