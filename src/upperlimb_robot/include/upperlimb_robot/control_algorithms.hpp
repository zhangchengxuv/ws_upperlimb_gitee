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
  MODE_ACTIVE = 2,
  MODE_LEFT_PRESET_POSITION = 7
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

void runIdle(const MotorStateCache & state, ControlOutputCache & output);
void runActiveMode(const MotorStateCache & state, ControlOutputCache & output);
void runLeftPresetPosition(const MotorStateCache & state, ControlOutputCache & output);

}  // namespace upperlimb_robot