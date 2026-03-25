#include "upperlimb_robot/control_algorithms.hpp"

#include <cmath>

namespace upperlimb_robot
{

namespace
{

// ===================== 编码器参数 =====================
//
// 每度对应的脉冲数，单位：pulse/deg
//
constexpr double PerAngle00 = 524288.0 / 360.0;
constexpr double PerAngle01 = 262144.0 / 360.0;
constexpr double PerAngle02 = 2048.0 / 360.0;

// ===================== 零位偏置 =====================
//
// 左臂零位偏置角，单位：deg
//
constexpr double SZO10 = 249.46;
constexpr double SZO20 = 12172.4808;
constexpr double SZO30 = -12.1935;
constexpr double SZO40 = -1354.3904;
constexpr double SZO50 = 23313.27;
constexpr double SZO60 = -728.71;
constexpr double SZO70 = -119.21;
constexpr double SZO80 = -23.2018;
constexpr double SZO90 = -23.2018;
constexpr double SZO100 = -833.12;

// 右臂零位偏置角，单位：deg
constexpr double SZRO10 = -1168.8423;
constexpr double SZRO20 = -100.2187;
constexpr double SZRO30 = 1009.6985;
constexpr double SZRO40 = -157.2542;
constexpr double SZRO50 = 76.6393;
constexpr double SZRO60 = 1231.8668;
constexpr double SZRO70 = 6597.68;
constexpr double SZRO80 = -447.8906;
constexpr double SZRO90 = 0.0;
constexpr double SZRO100 = -1200.0;

// ===================== 单轴参数表 =====================
struct JointParam
{
  double per_angle;    // pulse / deg
  double zero_offset;  // deg
  double ratio;        // 电机角 / 关节角
  double torque_scale; // Nm / raw
};

constexpr JointParam joint_map[21] = {
    {},

    // 右臂 1~10
    {PerAngle01, SZRO10, 121.0, 61.0 / 1000.0},
    {PerAngle00, SZRO20, 121.0, 61.0 / 1000.0},
    {PerAngle01, SZRO30, 121.0, 43.68 / 1000.0},
    {PerAngle01, SZRO40, 101.0, 49.0 / 1000.0},
    {PerAngle01, SZRO50, 121.0, 43.68 / 1000.0},
    {PerAngle00, SZRO60, 101.0, 36.4 / 1000.0},
    {PerAngle01, SZRO70, 100.0, 18.0 / 1000.0},
    {PerAngle02, SZRO80, 2048.0, 0.0},
    {PerAngle01, SZRO90, 100.0, 3.9 / 1000.0},
    {PerAngle00, SZRO100, 100.0, 3.9 / 1000.0},

    // 左臂 11~20
    {PerAngle00, SZO10, 121.0, 61.0 / 1000.0},
    {PerAngle01, SZO20, 121.0, 61.0 / 1000.0},
    {PerAngle01, SZO30, 121.0, 43.68 / 1000.0},
    {PerAngle01, SZO40, 101.0, 49.0 / 1000.0},
    {PerAngle01, SZO50, 121.0, 43.68 / 1000.0},
    {PerAngle00, SZO60, 101.0, 36.4 / 1000.0},
    {PerAngle01, SZO70, 100.0, 18.0 / 1000.0},
    {PerAngle02, SZO80, 2048.0, 0.0},
    {PerAngle01, SZO90, 100.0, 3.9 / 1000.0},
    {PerAngle01, SZO100, 100.0, 3.9 / 1000.0},
};

inline bool isLeftAxis(int axis)
{
  return axis >= 11 && axis <= 20;
}

inline const JointParam *getJointParam(int axis)
{
  if (axis < 1 || axis > 20)
  {
    return nullptr;
  }
  return &joint_map[axis];
}

// ===================== 幅值换算辅助函数 =====================
//
// 预设位姿模式中，Trap 参数只关心速度/加速度大小，不关心方向。
// 因为方向由当前位置和目标位置自动决定。
//
inline double pulsePerMsToJointSpeedDegPerSecMagnitude(int axis, double pulse_per_ms)
{
  const auto *p = getJointParam(axis);
  if (p == nullptr)
  {
    return 0.0;
  }

  return std::abs(pulse_per_ms) / (p->per_angle * p->ratio) * 1000.0;
}

inline double pulsePerMs2ToJointAccelDegPerSec2Magnitude(int axis, double pulse_per_ms2)
{
  const auto *p = getJointParam(axis);
  if (p == nullptr)
  {
    return 0.0;
  }

  return std::abs(pulse_per_ms2) / (p->per_angle * p->ratio) * 1000000.0;
}

} // namespace

// ===================== 位置换算 =====================
double jointDegToPulse(int axis, double joint_deg)
{
  const auto *p = getJointParam(axis);
  if (p == nullptr)
  {
    return 0.0;
  }

  if (isLeftAxis(axis))
  {
    return (p->zero_offset + p->ratio * joint_deg) * p->per_angle;
  }

  return (p->zero_offset - p->ratio * joint_deg) * p->per_angle;
}

double pulseToJointDeg(int axis, double pulse)
{
  const auto *p = getJointParam(axis);
  if (p == nullptr)
  {
    return 0.0;
  }

  if (isLeftAxis(axis))
  {
    return (pulse / p->per_angle - p->zero_offset) / p->ratio;
  }

  return (p->zero_offset - pulse / p->per_angle) / p->ratio;
}

// ===================== 速度换算 =====================
double jointSpeedDegPerSecToPulsePerMs(int axis, double joint_speed_deg_per_sec)
{
  const auto *p = getJointParam(axis);
  if (p == nullptr)
  {
    return 0.0;
  }

  return std::abs(joint_speed_deg_per_sec) * p->per_angle * p->ratio / 1000.0;
}

double pulsePerMsToJointDegPerSec(int axis, double pulse_per_ms)
{
  const auto *p = getJointParam(axis);
  if (p == nullptr)
  {
    return 0.0;
  }

  if (isLeftAxis(axis))
  {
    return (pulse_per_ms / p->per_angle) / p->ratio * 1000.0;
  }

  return -(pulse_per_ms / p->per_angle) / p->ratio * 1000.0;
}

// ===================== 加速度换算 =====================
double jointAccelDegPerSec2ToPulsePerMs2(int axis, double joint_acc_deg_per_sec2)
{
  const auto *p = getJointParam(axis);
  if (p == nullptr)
  {
    return 0.0;
  }

  return std::abs(joint_acc_deg_per_sec2) * p->per_angle * p->ratio / 1000000.0;
}

double pulsePerMs2ToJointDegPerSec2(int axis, double pulse_per_ms2)
{
  const auto *p = getJointParam(axis);
  if (p == nullptr)
  {
    return 0.0;
  }

  if (isLeftAxis(axis))
  {
    return (pulse_per_ms2 / p->per_angle) / p->ratio * 1000000.0;
  }

  return -(pulse_per_ms2 / p->per_angle) / p->ratio * 1000000.0;
}

// ===================== 扭矩换算 =====================
double torqueRawToNm(int axis, double raw)
{
  const auto *p = getJointParam(axis);
  if (p == nullptr)
  {
    return 0.0;
  }

  return raw * p->torque_scale;
}

double torqueNmToRaw(int axis, double torque_nm)
{
  const auto *p = getJointParam(axis);
  if (p == nullptr || p->torque_scale == 0.0)
  {
    return 0.0;
  }

  return torque_nm / p->torque_scale;
}

// ===================== 空闲模式 =====================
void runIdle(const MotorStateCache &state, ControlOutputCache &output)
{
  (void)state;
  output = ControlOutputCache{};
  output.valid = false;
}

// ===================== 主动模式骨架 =====================
void runActiveMode(
    const MotorStateCache &state,
    const SensorStateCache &sensor_state,
    ControlOutputCache &output)
{
  (void)state;
  (void)sensor_state;

  output = ControlOutputCache{};
  output.valid = false;

  // 后续在这里补：
  // 1. 传感器去偏
  // 2. 导纳
  // 3. 轨迹修正
  // 4. 位置或扭矩输出
}

// ===================== 零力模式骨架 =====================
void runZeroForceMode(
    const MotorStateCache &state,
    const SensorStateCache &sensor_state,
    ControlOutputCache &output)
{
  (void)state;
  (void)sensor_state;

  output = ControlOutputCache{};
  output.valid = false;
  // 后续在这里补：
  // 1. 取去偏后的六维力
  // 2. 死区和滤波
  // 3. 力 -> 关节目标或扭矩目标
  // 4. 设置 output.use_torque / use_position
}

// ===================== 左臂预设位姿 =====================
void runLeftPresetPosition(const MotorStateCache &state, ControlOutputCache &output)
{
  (void)state;

  output = ControlOutputCache{};
  output.valid = true;

  output.use_position[10] = true;
  output.target_position[10] = 0.0;
  output.target_velocity[10] = pulsePerMsToJointSpeedDegPerSecMagnitude(11, 400.0);
  output.target_acc[10] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(11, 0.5);
  output.target_dec[10] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(11, 0.25);
  output.target_smooth_time[10] = 25;

  output.use_position[11] = true;
  output.target_position[11] = 0.0;
  output.target_velocity[11] = pulsePerMsToJointSpeedDegPerSecMagnitude(12, 500.0);
  output.target_acc[11] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(12, 0.5);
  output.target_dec[11] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(12, 0.25);
  output.target_smooth_time[11] = 25;

  output.use_position[12] = true;
  output.target_position[12] = 0.0;
  output.target_velocity[12] = pulsePerMsToJointSpeedDegPerSecMagnitude(13, 500.0);
  output.target_acc[12] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(13, 0.5);
  output.target_dec[12] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(13, 0.25);
  output.target_smooth_time[12] = 25;

  output.use_position[13] = true;
  output.target_position[13] = 0.0;
  output.target_velocity[13] = pulsePerMsToJointSpeedDegPerSecMagnitude(14, 1500.0);
  output.target_acc[13] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(14, 5.0);
  output.target_dec[13] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(14, 5.0);
  output.target_smooth_time[13] = 25;

  output.use_position[14] = true;
  output.target_position[14] = 0.0;
  output.target_velocity[14] = pulsePerMsToJointSpeedDegPerSecMagnitude(15, 1500.0);
  output.target_acc[14] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(15, 5.0);
  output.target_dec[14] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(15, 5.0);
  output.target_smooth_time[14] = 25;

  output.use_position[15] = true;
  output.target_position[15] = 0.0;
  output.target_velocity[15] = pulsePerMsToJointSpeedDegPerSecMagnitude(16, 1500.0);
  output.target_acc[15] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(16, 5.0);
  output.target_dec[15] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(16, 5.0);
  output.target_smooth_time[15] = 25;

  output.use_position[16] = true;
  output.target_position[16] = 45.0;
  output.target_velocity[16] = pulsePerMsToJointSpeedDegPerSecMagnitude(17, 1500.0);
  output.target_acc[16] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(17, 5.0);
  output.target_dec[16] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(17, 0.25);
  output.target_smooth_time[16] = 25;

  output.use_position[18] = true;
  output.target_position[18] = 0.0;
  output.target_velocity[18] = pulsePerMsToJointSpeedDegPerSecMagnitude(19, 1500.0);
  output.target_acc[18] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(19, 5.0);
  output.target_dec[18] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(19, 0.25);
  output.target_smooth_time[18] = 25;

  output.use_position[19] = true;
  output.target_position[19] = 0.0;
  output.target_velocity[19] = pulsePerMsToJointSpeedDegPerSecMagnitude(20, 1500.0);
  output.target_acc[19] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(20, 5.0);
  output.target_dec[19] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(20, 0.25);
  output.target_smooth_time[19] = 25;
}

// ===================== 右臂预设位姿 =====================
void runRightPresetPosition(const MotorStateCache &state, ControlOutputCache &output)
{
  (void)state;

  output = ControlOutputCache{};
  output.valid = true;

  output.use_position[0] = true;
  output.target_position[0] = 0.0;
  output.target_velocity[0] = pulsePerMsToJointSpeedDegPerSecMagnitude(1, 200.0);
  output.target_acc[0] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(1, 0.5);
  output.target_dec[0] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(1, 0.25);
  output.target_smooth_time[0] = 25;

  output.use_position[1] = true;
  output.target_position[1] = 0.0;
  output.target_velocity[1] = pulsePerMsToJointSpeedDegPerSecMagnitude(2, 500.0);
  output.target_acc[1] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(2, 0.5);
  output.target_dec[1] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(2, 0.25);
  output.target_smooth_time[1] = 25;

  output.use_position[2] = true;
  output.target_position[2] = 0.0;
  output.target_velocity[2] = pulsePerMsToJointSpeedDegPerSecMagnitude(3, 500.0);
  output.target_acc[2] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(3, 0.5);
  output.target_dec[2] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(3, 0.25);
  output.target_smooth_time[2] = 25;

  output.use_position[3] = true;
  output.target_position[3] = 0.0;
  output.target_velocity[3] = pulsePerMsToJointSpeedDegPerSecMagnitude(4, 800.0);
  output.target_acc[3] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(4, 0.5);
  output.target_dec[3] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(4, 0.25);
  output.target_smooth_time[3] = 25;

  output.use_position[4] = true;
  output.target_position[4] = 0.0;
  output.target_velocity[4] = pulsePerMsToJointSpeedDegPerSecMagnitude(5, 600.0);
  output.target_acc[4] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(5, 5.0);
  output.target_dec[4] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(5, 0.25);
  output.target_smooth_time[4] = 25;

  output.use_position[5] = true;
  output.target_position[5] = 0.0;
  output.target_velocity[5] = pulsePerMsToJointSpeedDegPerSecMagnitude(6, 600.0);
  output.target_acc[5] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(6, 5.0);
  output.target_dec[5] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(6, 0.25);
  output.target_smooth_time[5] = 25;

  output.use_position[6] = true;
  output.target_position[6] = 45.0;
  output.target_velocity[6] = pulsePerMsToJointSpeedDegPerSecMagnitude(7, 3600.0);
  output.target_acc[6] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(7, 5.0);
  output.target_dec[6] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(7, 0.25);
  output.target_smooth_time[6] = 25;

  output.use_position[8] = true;
  output.target_position[8] = 0.0;
  output.target_velocity[8] = pulsePerMsToJointSpeedDegPerSecMagnitude(9, 9000.0);
  output.target_acc[8] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(9, 5.0);
  output.target_dec[8] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(9, 0.25);
  output.target_smooth_time[8] = 25;

  output.use_position[9] = true;
  output.target_position[9] = 0.0;
  output.target_velocity[9] = pulsePerMsToJointSpeedDegPerSecMagnitude(10, 3000.0);
  output.target_acc[9] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(10, 5.0);
  output.target_dec[9] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(10, 0.25);
  output.target_smooth_time[9] = 25;
}

} // namespace upperlimb_robot