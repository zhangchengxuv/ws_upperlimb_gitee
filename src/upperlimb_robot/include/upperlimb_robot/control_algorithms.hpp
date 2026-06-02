#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace upperlimb_robot
{

  // ===================== 系统常量定义 =====================

  // 总轴数
  // 控制层内部数组索引统一使用 0~19
  static constexpr std::size_t AXIS_NUM = 20;

  // 六维力传感器数量
  // 传感器数组索引统一使用 0~3
  static constexpr std::size_t FORCE_SENSOR_NUM = 4;

  // ===================== 控制模式定义 =====================
  //
  // 说明：
  // 1. current_mode_ 表示“请求模式”
  // 2. applied_mode_ 表示“已经完成进入动作的模式”
  // 3. 真正的模式进入/退出动作建议在 onModeEnter()/onModeExit() 中执行
  //
  enum ControlMode : int
  {
    // 空闲模式
    // 默认不输出控制命令
    MODE_IDLE = 0,

    // 左臂预设位置模式
    MODE_LEFT_PRESET_POSITION = 1,

    // 右臂预设位置模式
    MODE_RIGHT_PRESET_POSITION = 2,

    // 左臂主动训练模式
    MODE_ACTIVE = 3,

    // 左臂零力模式
    MODE_ZERO_FORCE = 4,

    // 左臂正弦被动模式（PID跟踪）
    MODE_PASSIVE = 5,

    // 左臂正弦被动模式（PD+前馈跟踪）
    MODE_PASSIVE_SINE_PD_FF = 6,

    // 左臂正弦被动模式（扭矩输出）
    MODE_PASSIVE_TORQUE = 7,

    // 右臂零力模式
    MODE_RIGHT_ZERO_FORCE = 8,

    // 双臂零力模式
    MODE_BILATERAL_ZERO_FORCE = 9,

    // 双臂镜像模式
    MODE_MIRROR = 10,

    // 左臂主动模式，右臂扭矩镜像跟踪
    MODE_ACTIVE_MIRROR = 11,

    // 双手虚拟弹簧主动模式
    MODE_ACTIVE_SPRING = 12,

    // 双侧主动导纳模式
    MODE_BILATERAL_ACTIVE = 13,

  };

  // ===================== 电机状态缓存 =====================
  //
  // 对外统一使用真实物理量：
  // position : deg
  // velocity : deg/s
  // torque   : Nm
  //
  // 注意：
  // 1. 这里的下标是“控制层数组索引 0~19”
  // 2. 不一定等于 GTN 物理轴号 1~20
  // 3. 真正访问驱动时，应通过 axis_ids_[i] 做映射
  //
  struct MotorStateCache
  {
    // 数组索引 0~19
    // 关节位置，单位 deg
    std::array<double, AXIS_NUM> position{};

    // 数组索引 0~19
    // 关节速度，单位 deg/s
    std::array<double, AXIS_NUM> velocity{};

    // 数组索引 0~19
    // 关节扭矩，单位 Nm
    std::array<double, AXIS_NUM> torque{};

    // 数组索引 0~19
    // 轴是否使能
    std::array<bool, AXIS_NUM> enable{};
  };

  // ===================== 单个六维力传感器 =====================
  //
  // 力单位：N
  // 力矩单位：Nm
  //
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
  //
  // sensor[0~3] / bias[0~3] 的索引约定
  //
  enum ForceSensorIndex : std::size_t
  {
    SENSOR_RIGHT_HAND = 0,
    SENSOR_RIGHT_ARM = 1,
    SENSOR_LEFT_HAND = 2,
    SENSOR_LEFT_ARM = 3,
  };

  // ===================== 传感器状态缓存 =====================
  //
  // sensor : 当前实时值
  // bias   : 已记录的零偏
  // valid  : 当前实时值是否有效
  //
  // 说明：
  // valid == true 表示本轮读取成功
  // valid == false 时，不应拿当前 sensor 直接记录 bias
  //
  struct SensorStateCache
  {
    // 数组索引 0~3
    // 当前实时传感器值
    std::array<ForceSensorData, FORCE_SENSOR_NUM> sensor{};

    // 数组索引 0~3
    // 零偏值
    std::array<ForceSensorData, FORCE_SENSOR_NUM> bias{};

    // 当前传感器缓存是否有效
    bool valid{false};
  };

  // ===================== 控制输出缓存 =====================
  //
  // 这是“算法层 -> 执行层”的统一输出接口。
  //
  // 约定：
  // 1. 数组索引统一为 0~19
  // 2. 算法层统一使用真实物理量
  // 3. target_torque 的单位是 Nm，不是 raw
  // 4. 正常写法为：output.target_torque[6] = 2.0;（2.0 Nm）
  //
  struct ControlOutputCache
  {
    // 数组索引 0~19
    // 目标位置，单位 deg
    std::array<double, AXIS_NUM> target_position{};

    // 数组索引 0~19
    // 目标速度，单位 deg/s
    std::array<double, AXIS_NUM> target_velocity{};

    // 数组索引 0~19
    // 目标扭矩，单位 Nm
    // 正常写法为：output.target_torque[6] = 2.0;（2.0 Nm）
    std::array<double, AXIS_NUM> target_torque{};

    // 数组索引 0~19
    // 目标加速度，单位 deg/s^2
    std::array<double, AXIS_NUM> target_acc{};

    // 数组索引 0~19
    // 目标减速度，单位 deg/s^2
    std::array<double, AXIS_NUM> target_dec{};

    // 数组索引 0~19
    // 平滑时间，单位由底层驱动接口定义
    std::array<short, AXIS_NUM> target_smooth_time{};

    // 数组索引 0~19
    // 是否对该轴启用位置控制
    std::array<bool, AXIS_NUM> use_position{};

    // 数组索引 0~19
    // 是否显式使用速度字段
    // 当前版本主要作为语义保留
    std::array<bool, AXIS_NUM> use_velocity{};

    // 数组索引 0~19
    // 是否对该轴启用扭矩控制
    std::array<bool, AXIS_NUM> use_torque{};

    // 输出是否有效
    // valid == false 时，执行层应跳过本轮控制输出
    bool valid{false};
  };

  // ===================== 单位换算接口 =====================
  //
  // 注意：
  // 这些换算函数传入的是“真实物理轴号 axis（1~20）”
  // 不是控制层数组索引 i（0~19）
  //
  double jointDegToPulse(int axis, double joint_deg);
  double pulseToJointDeg(int axis, double pulse);

  double jointSpeedDegPerSecToPulsePerMs(int axis, double joint_speed_deg_per_sec);
  double pulsePerMsToJointDegPerSec(int axis, double pulse_per_ms);

  double jointAccelDegPerSec2ToPulsePerMs2(int axis, double joint_acc_deg_per_sec2);
  double pulsePerMs2ToJointDegPerSec2(int axis, double pulse_per_ms2);

  // raw -> Nm
  // 用于“读取驱动反馈扭矩”
  double torqueRawToNm(int axis, double raw);

  // Nm -> raw
  // 用于“下发驱动扭矩命令”
  double torqueNmToRaw(int axis, double torque_nm);

  // 重置双手虚拟弹簧模式的状态
  void resetActiveSpringMode();

  // 重置双侧主动模式的状态
  void resetBilateralActiveMode();

  // ===================== 控制算法接口 =====================
  // 不需要需要传感器信息

  // 空闲模式
  void runIdle(const MotorStateCache &state, ControlOutputCache &output);
  // 预设位姿模式
  void runLeftPresetPosition(const MotorStateCache &state, ControlOutputCache &output);
  void runRightPresetPosition(const MotorStateCache &state, ControlOutputCache &output);

  // 需要传感器信息的算法

  // MotorStateCache 提供位置、速度、扭矩等电机状态信息
  // SensorStateCache        提供六维力传感器信息
  // ControlOutputCache      提供输出接口

  // 左边零力模式
  void runZeroForceMode(const MotorStateCache &state, const SensorStateCache &sensor_state, ControlOutputCache &output);
  // 右边零力模式
  void runRightZeroForceMode(const MotorStateCache &state, const SensorStateCache &sensor_state, ControlOutputCache &output);
  // 双臂零力模式
  void runBilateralZeroForceMode(const MotorStateCache &state, const SensorStateCache &sensor_state, ControlOutputCache &output);
  // 主动模式
  void runActiveMode(const MotorStateCache &state, const SensorStateCache &sensor_state, ControlOutputCache &output);
  // 左边正弦被动模式（PID跟踪）
  void runPassiveMode(const MotorStateCache &state, const SensorStateCache &sensor_state, ControlOutputCache &output);
  // 左边正弦被动模式（PD+前馈跟踪）
  void runPassivePDFFMode(const MotorStateCache &state, const SensorStateCache &sensor_state, ControlOutputCache &output);
  // 左边正弦被动模式（扭矩输出）
  void runPassiveTorqueMode(const MotorStateCache &state, const SensorStateCache &sensor_state, ControlOutputCache &output);
  // 双臂镜像模式
  void runMirrorMode(const MotorStateCache &state, const SensorStateCache &sensor_state, ControlOutputCache &output);
  // 主动镜像模式
  void runActiveMirrorMode(const MotorStateCache &state, const SensorStateCache &sensor_state, ControlOutputCache &output);
  // 双侧主动导纳模式
  void runBilateralActiveMode(const MotorStateCache &state, const SensorStateCache &sensor_state, ControlOutputCache &output);
  // 双手虚拟弹簧主动模式
  void runActiveSpringMode(const MotorStateCache &state, const SensorStateCache &sensor_state, ControlOutputCache &output);

} // namespace upperlimb_robot