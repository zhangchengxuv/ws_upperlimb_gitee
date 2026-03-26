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
// 不同轴使用的编码器分辨率不同，所以这里准备了 3 套常量。
// 后面在 joint_map 里按轴选择对应的 per_angle。
//
constexpr double PerAngle00 = 524288.0 / 360.0;
constexpr double PerAngle01 = 262144.0 / 360.0;
constexpr double PerAngle02 = 2048.0 / 360.0;

// ===================== 零位偏置 =====================
//
// zero_offset 的单位是 deg。
// 这些值用于“电机编码器零位”和“关节物理零位”之间的对应换算。
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
//
// 这里定义每个物理轴（axis=1~20）的换算参数。
//
// 字段说明：
// per_angle    : 每度对应多少脉冲，单位 pulse/deg
// zero_offset  : 零位偏置，单位 deg
// ratio        : 电机角 / 关节角 的减速比
// torque_scale : 扭矩换算比例，单位 Nm/raw
//
// 注意：
// 1. 这里的 axis 是“物理轴号 1~20”
// 2. 不是控制层数组索引 0~19
//
struct JointParam
{
  double per_angle;    // pulse / deg
  double zero_offset;  // deg
  double ratio;        // 电机角 / 关节角
  double torque_scale; // Nm / raw
};

constexpr JointParam joint_map[21] = {
    {},

    // ===================== 右臂 1~10 =====================
    //
    // 物理轴号 1~10 对应右臂
    //
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

    // ===================== 左臂 11~20 =====================
    //
    // 物理轴号 11~20 对应左臂
    //
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

// ===================== 判断是否为左臂物理轴 =====================
//
// 输入：
// axis -> 真实物理轴号 1~20
//
// 返回：
// true  -> 左臂物理轴 11~20
// false -> 非左臂物理轴
//
inline bool isLeftAxis(int axis)
{
  return axis >= 11 && axis <= 20;
}

// ===================== 获取单轴参数 =====================
//
// 输入：
// axis -> 真实物理轴号 1~20
//
// 返回：
// 对应轴的参数指针；非法轴号返回 nullptr
//
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
// 这两个函数只取“速度/加速度的大小”，不保留正负方向。
// 主要用于预设位姿模式下配置 Trap 参数。
//
// 说明：
// Trap 运动参数中的 vel / acc / dec 一般只需要幅值，
// 方向由“当前位置 -> 目标位置”的相对关系自动决定。
//

// pulse/ms -> deg/s（只保留幅值）
inline double pulsePerMsToJointSpeedDegPerSecMagnitude(int axis, double pulse_per_ms)
{
  const auto *p = getJointParam(axis);
  if (p == nullptr)
  {
    return 0.0;
  }

  return std::abs(pulse_per_ms) / (p->per_angle * p->ratio) * 1000.0;
}

// pulse/ms^2 -> deg/s^2（只保留幅值）
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
//
// jointDegToPulse / pulseToJointDeg
//
// 作用：
// 在“关节物理角度 deg”和“驱动编码器脉冲 pulse”之间做双向换算。
//
// 注意：
// 1. 输入的 axis 是物理轴号 1~20，不是数组索引 0~19
// 2. 左右臂方向定义不同，因此公式符号不同
//

// 关节角 deg -> 编码器脉冲 pulse
double jointDegToPulse(int axis, double joint_deg)
{
  const auto *p = getJointParam(axis);
  if (p == nullptr)
  {
    return 0.0;
  }

  // 左臂方向定义
  if (isLeftAxis(axis))
  {
    return (p->zero_offset + p->ratio * joint_deg) * p->per_angle;
  }

  // 右臂方向定义
  return (p->zero_offset - p->ratio * joint_deg) * p->per_angle;
}

// 编码器脉冲 pulse -> 关节角 deg
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
//
// jointSpeedDegPerSecToPulsePerMs / pulsePerMsToJointDegPerSec
//
// 作用：
// 在“关节速度 deg/s”和“驱动速度 pulse/ms”之间做双向换算。
//
// 注意：
// 1. 对外统一使用真实物理量 deg/s
// 2. 底层驱动速度单位是 pulse/ms
// 3. 左右臂在反馈方向上符号定义不同
//

// 关节速度 deg/s -> pulse/ms
double jointSpeedDegPerSecToPulsePerMs(int axis, double joint_speed_deg_per_sec)
{
  const auto *p = getJointParam(axis);
  if (p == nullptr)
  {
    return 0.0;
  }

  // 发送给驱动的速度参数通常只需要幅值
  return std::abs(joint_speed_deg_per_sec) * p->per_angle * p->ratio / 1000.0;
}

// pulse/ms -> 关节速度 deg/s
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
//
// jointAccelDegPerSec2ToPulsePerMs2 / pulsePerMs2ToJointDegPerSec2
//
// 作用：
// 在“关节加速度 deg/s^2”和“驱动加速度 pulse/ms^2”之间做双向换算。
//
// 注意：
// 1. 对外统一使用真实物理量 deg/s^2
// 2. 发送给驱动的 acc/dec 一般只需要幅值
//

// 关节加速度 deg/s^2 -> pulse/ms^2
double jointAccelDegPerSec2ToPulsePerMs2(int axis, double joint_acc_deg_per_sec2)
{
  const auto *p = getJointParam(axis);
  if (p == nullptr)
  {
    return 0.0;
  }

  return std::abs(joint_acc_deg_per_sec2) * p->per_angle * p->ratio / 1000000.0;
}

// pulse/ms^2 -> 关节加速度 deg/s^2
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
//
// torqueRawToNm / torqueNmToRaw
//
// 作用：
// 在“驱动内部扭矩单位 raw”和“物理扭矩单位 Nm”之间做双向换算。
//
// 使用约定：
// 1. 读驱动反馈时：raw -> Nm
// 2. 算法层 output.target_torque[] 一律使用 Nm
// 3. 真正下发给驱动前：Nm -> raw
//
// 例如：
// output.target_torque[6] = 2.0;   // 这里的 2.0 是 Nm
//

// raw -> Nm
double torqueRawToNm(int axis, double raw)
{
  const auto *p = getJointParam(axis);
  if (p == nullptr)
  {
    return 0.0;
  }

  return raw * p->torque_scale;
}

// Nm -> raw
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
//
// 当前实现：
// 1. 不使用 state
// 2. 清空 output
// 3. 标记 output 无效
//
// 执行层看到 output.valid == false 后，
// 会决定本轮不执行控制输出。
//
void runIdle(const MotorStateCache &state, ControlOutputCache &output)
{
  (void)state;
  output = ControlOutputCache{};
  output.valid = false;
}

// ===================== 主动模式骨架 =====================
//
// 当前只保留接口，不实现具体主动控制算法。
//
// 后续你可以在这里继续补：
// 1. 传感器去偏
// 2. 导纳/阻抗
// 3. 轨迹修正
// 4. 位置模式或扭矩模式输出
//
// 注意：
// 1. output.target_position 单位是 deg
// 2. output.target_velocity 单位是 deg/s
// 3. output.target_torque 单位是 Nm
//
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
//
// 当前实现说明：
// 1. 这里只是一个测试骨架，不是完整零力控制
// 2. 当前只给控制层索引 16 的轴发送 2.0 Nm 扭矩目标
// 3. 其他轴不输出
//
// 注意：
// output.target_torque[16] = 2.0;
// 这里的 2.0 单位是 Nm，不是 raw。
//
// 后续你可以在这里继续补：
// 1. 取去偏后的六维力
// 2. 死区和滤波
// 3. 力 -> 关节目标或扭矩目标
// 4. 设置 output.use_torque / use_position
//
void runZeroForceMode(
    const MotorStateCache &state,
    const SensorStateCache &sensor_state,
    ControlOutputCache &output)
{
  (void)state;
  (void)sensor_state;

  output = ControlOutputCache{};
  output.valid = true;

  // 控制层数组索引 16
  output.use_torque[16] = true;

  // 单位：Nm
  output.target_torque[16] = 2.0;

  // 后续在这里补：
  // 1. 取去偏后的六维力
  // 2. 死区和滤波
  // 3. 力 -> 关节目标或扭矩目标
  // 4. 设置 output.use_torque / use_position
}

// ===================== 左臂预设位姿 =====================
//
// 当前实现：
// 1. 只设置左臂相关控制层索引
// 2. 左臂控制层索引范围为 10~19
// 3. 目标位置单位是 deg
// 4. 目标速度单位是 deg/s
// 5. 目标加减速度单位是 deg/s^2
//
// 注意：
// 这里调用 pulsePerMsToJointSpeedDegPerSecMagnitude() 等辅助函数，
// 是为了把旧代码里按 pulse/ms、pulse/ms^2 写的经验参数，
// 转换成算法层统一使用的物理量单位。
//
void runLeftPresetPosition(const MotorStateCache &state, ControlOutputCache &output)
{
  (void)state;

  output = ControlOutputCache{};
  output.valid = true;

  // ===================== 左臂 物理轴11 -> 控制层索引10 =====================
  output.use_position[10] = true;
  output.target_position[10] = 0.0;
  output.target_velocity[10] = pulsePerMsToJointSpeedDegPerSecMagnitude(11, 400.0);
  output.target_acc[10] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(11, 0.5);
  output.target_dec[10] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(11, 0.25);
  output.target_smooth_time[10] = 25;

  // ===================== 左臂 物理轴12 -> 控制层索引11 =====================
  output.use_position[11] = true;
  output.target_position[11] = 0.0;
  output.target_velocity[11] = pulsePerMsToJointSpeedDegPerSecMagnitude(12, 500.0);
  output.target_acc[11] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(12, 0.5);
  output.target_dec[11] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(12, 0.25);
  output.target_smooth_time[11] = 25;

  // ===================== 左臂 物理轴13 -> 控制层索引12 =====================
  output.use_position[12] = true;
  output.target_position[12] = 0.0;
  output.target_velocity[12] = pulsePerMsToJointSpeedDegPerSecMagnitude(13, 500.0);
  output.target_acc[12] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(13, 0.5);
  output.target_dec[12] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(13, 0.25);
  output.target_smooth_time[12] = 25;

  // ===================== 左臂 物理轴14 -> 控制层索引13 =====================
  output.use_position[13] = true;
  output.target_position[13] = 0.0;
  output.target_velocity[13] = pulsePerMsToJointSpeedDegPerSecMagnitude(14, 1500.0);
  output.target_acc[13] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(14, 5.0);
  output.target_dec[13] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(14, 5.0);
  output.target_smooth_time[13] = 25;

  // ===================== 左臂 物理轴15 -> 控制层索引14 =====================
  output.use_position[14] = true;
  output.target_position[14] = 0.0;
  output.target_velocity[14] = pulsePerMsToJointSpeedDegPerSecMagnitude(15, 1500.0);
  output.target_acc[14] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(15, 5.0);
  output.target_dec[14] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(15, 5.0);
  output.target_smooth_time[14] = 25;

  // ===================== 左臂 物理轴16 -> 控制层索引15 =====================
  output.use_position[15] = true;
  output.target_position[15] = 0.0;
  output.target_velocity[15] = pulsePerMsToJointSpeedDegPerSecMagnitude(16, 1500.0);
  output.target_acc[15] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(16, 5.0);
  output.target_dec[15] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(16, 5.0);
  output.target_smooth_time[15] = 25;

  // ===================== 左臂 物理轴17 -> 控制层索引16 =====================
  output.use_position[16] = true;
  output.target_position[16] = 45.0;
  output.target_velocity[16] = pulsePerMsToJointSpeedDegPerSecMagnitude(17, 1500.0);
  output.target_acc[16] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(17, 5.0);
  output.target_dec[16] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(17, 0.25);
  output.target_smooth_time[16] = 25;

  // 控制层索引17 当前未设置，保持默认值

  // ===================== 左臂 物理轴19 -> 控制层索引18 =====================
  output.use_position[18] = true;
  output.target_position[18] = 0.0;
  output.target_velocity[18] = pulsePerMsToJointSpeedDegPerSecMagnitude(19, 1500.0);
  output.target_acc[18] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(19, 5.0);
  output.target_dec[18] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(19, 0.25);
  output.target_smooth_time[18] = 25;

  // ===================== 左臂 物理轴20 -> 控制层索引19 =====================
  output.use_position[19] = true;
  output.target_position[19] = 0.0;
  output.target_velocity[19] = pulsePerMsToJointSpeedDegPerSecMagnitude(20, 1500.0);
  output.target_acc[19] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(20, 5.0);
  output.target_dec[19] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(20, 0.25);
  output.target_smooth_time[19] = 25;
}

// ===================== 右臂预设位姿 =====================
//
// 当前实现：
// 1. 只设置右臂相关控制层索引
// 2. 右臂控制层索引范围为 0~9
// 3. 目标位置单位是 deg
// 4. 目标速度单位是 deg/s
// 5. 目标加减速度单位是 deg/s^2
//
void runRightPresetPosition(const MotorStateCache &state, ControlOutputCache &output)
{
  (void)state;

  output = ControlOutputCache{};
  output.valid = true;

  // ===================== 右臂 物理轴1 -> 控制层索引0 =====================
  output.use_position[0] = true;
  output.target_position[0] = 0.0;
  output.target_velocity[0] = pulsePerMsToJointSpeedDegPerSecMagnitude(1, 200.0);
  output.target_acc[0] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(1, 0.5);
  output.target_dec[0] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(1, 0.25);
  output.target_smooth_time[0] = 25;

  // ===================== 右臂 物理轴2 -> 控制层索引1 =====================
  output.use_position[1] = true;
  output.target_position[1] = 0.0;
  output.target_velocity[1] = pulsePerMsToJointSpeedDegPerSecMagnitude(2, 500.0);
  output.target_acc[1] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(2, 0.5);
  output.target_dec[1] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(2, 0.25);
  output.target_smooth_time[1] = 25;

  // ===================== 右臂 物理轴3 -> 控制层索引2 =====================
  output.use_position[2] = true;
  output.target_position[2] = 0.0;
  output.target_velocity[2] = pulsePerMsToJointSpeedDegPerSecMagnitude(3, 500.0);
  output.target_acc[2] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(3, 0.5);
  output.target_dec[2] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(3, 0.25);
  output.target_smooth_time[2] = 25;

  // ===================== 右臂 物理轴4 -> 控制层索引3 =====================
  output.use_position[3] = true;
  output.target_position[3] = 0.0;
  output.target_velocity[3] = pulsePerMsToJointSpeedDegPerSecMagnitude(4, 800.0);
  output.target_acc[3] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(4, 0.5);
  output.target_dec[3] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(4, 0.25);
  output.target_smooth_time[3] = 25;

  // ===================== 右臂 物理轴5 -> 控制层索引4 =====================
  output.use_position[4] = true;
  output.target_position[4] = 0.0;
  output.target_velocity[4] = pulsePerMsToJointSpeedDegPerSecMagnitude(5, 600.0);
  output.target_acc[4] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(5, 5.0);
  output.target_dec[4] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(5, 0.25);
  output.target_smooth_time[4] = 25;

  // ===================== 右臂 物理轴6 -> 控制层索引5 =====================
  output.use_position[5] = true;
  output.target_position[5] = 0.0;
  output.target_velocity[5] = pulsePerMsToJointSpeedDegPerSecMagnitude(6, 600.0);
  output.target_acc[5] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(6, 5.0);
  output.target_dec[5] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(6, 0.25);
  output.target_smooth_time[5] = 25;

  // ===================== 右臂 物理轴7 -> 控制层索引6 =====================
  output.use_position[6] = true;
  output.target_position[6] = 45.0;
  output.target_velocity[6] = pulsePerMsToJointSpeedDegPerSecMagnitude(7, 3600.0);
  output.target_acc[6] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(7, 5.0);
  output.target_dec[6] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(7, 0.25);
  output.target_smooth_time[6] = 25;

  // 控制层索引7 当前未设置，保持默认值

  // ===================== 右臂 物理轴9 -> 控制层索引8 =====================
  output.use_position[8] = true;
  output.target_position[8] = 0.0;
  output.target_velocity[8] = pulsePerMsToJointSpeedDegPerSecMagnitude(9, 9000.0);
  output.target_acc[8] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(9, 5.0);
  output.target_dec[8] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(9, 0.25);
  output.target_smooth_time[8] = 25;

  // ===================== 右臂 物理轴10 -> 控制层索引9 =====================
  output.use_position[9] = true;
  output.target_position[9] = 0.0;
  output.target_velocity[9] = pulsePerMsToJointSpeedDegPerSecMagnitude(10, 3000.0);
  output.target_acc[9] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(10, 5.0);
  output.target_dec[9] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(10, 0.25);
  output.target_smooth_time[9] = 25;
}

} // namespace upperlimb_robot