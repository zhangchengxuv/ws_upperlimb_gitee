#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace upperlimb_robot
{

static constexpr std::size_t AXIS_NUM = 20;
static constexpr std::size_t FORCE_SENSOR_NUM = 4;

// ===================== 控制模式定义 =====================
enum ControlMode : int
{
  MODE_IDLE = 0,
  MODE_LEFT_PRESET_POSITION = 1,
  MODE_RIGHT_PRESET_POSITION = 2,
  MODE_ACTIVE = 3,
  MODE_ZERO_FORCE = 4,
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

// ===================== 单个六维力传感器 =====================
struct ForceSensorData
{
  double fx{0.0};
  double fy{0.0};
  double fz{0.0};
  double mx{0.0};
  double my{0.0};
  double mz{0.0};
};

// ===================== 传感器索引定义 =====================
enum ForceSensorIndex : std::size_t
{
  SENSOR_RIGHT_HAND = 0,
  SENSOR_RIGHT_ARM = 1,
  SENSOR_LEFT_HAND = 2,
  SENSOR_LEFT_ARM = 3,
};

// ===================== 传感器状态缓存 =====================
struct SensorStateCache
{
  std::array<ForceSensorData, FORCE_SENSOR_NUM> sensor{};
  std::array<ForceSensorData, FORCE_SENSOR_NUM> bias{};
  bool valid{false};
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

// ===================== 单位换算接口 =====================
double jointDegToPulse(int axis, double joint_deg);
double pulseToJointDeg(int axis, double pulse);

double jointSpeedDegPerSecToPulsePerMs(int axis, double joint_speed_deg_per_sec);
double pulsePerMsToJointDegPerSec(int axis, double pulse_per_ms);

double jointAccelDegPerSec2ToPulsePerMs2(int axis, double joint_acc_deg_per_sec2);
double pulsePerMs2ToJointDegPerSec2(int axis, double pulse_per_ms2);

double torqueRawToNm(int axis, double raw);
double torqueNmToRaw(int axis, double torque_nm);

// ===================== 控制算法接口 =====================
//
// 这 3 个算法不需要传感器信息
//
void runLeftPresetPosition(
    const MotorStateCache &state,
    ControlOutputCache &output);

void runRightPresetPosition(
    const MotorStateCache &state,
    ControlOutputCache &output);

void runIdle(
    const MotorStateCache &state,
    ControlOutputCache &output);

// 这 2 个算法需要传感器信息
//
void runZeroForceMode(
    const MotorStateCache &state,
    const SensorStateCache &sensor_state,
    ControlOutputCache &output);

void runActiveMode(
    const MotorStateCache &state,
    const SensorStateCache &sensor_state,
    ControlOutputCache &output);

} // namespace upperlimb_robot