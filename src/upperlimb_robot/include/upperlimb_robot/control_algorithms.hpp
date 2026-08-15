#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <memory>
#include <string>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

namespace upperlimb_robot
{

  // ===================== 系统常量定义 =====================

  // 总轴数
  // 控制层内部数组索引统一使用 0~19
  static constexpr std::size_t AXIS_NUM = 20;

  // 六维力传感器数量
  // 传感器数组索引统一使用 0~3
  static constexpr std::size_t FORCE_SENSOR_NUM = 4;

  // ===================== 统一重力补偿参数 =====================
  //
  // 所有使用 Pinocchio 重力力矩的控制算法都共用这里的参数。
  // 左右臂分别配置，并且每条臂的 7 个主动关节分别配置。
  //
  // 修正公式：
  //   tau_gravity[i] = gain[i] * tau_gravity_model[i] + bias_nm[i]
  //
  // 参数顺序：
  //   左臂：joint1  ~ joint7
  //   右臂：joint11 ~ joint17
  //
  // 注意：bias_nm 使用 Pinocchio 模型方向，右臂最终发送时的整体负号
  // 仍由各控制算法原有的驱动方向映射负责。
  struct GravityCompensationParameters
  {
    std::array<double, 7> gain{};
    std::array<double, 7> bias_nm{};
  };

  extern const GravityCompensationParameters LEFT_ARM_GRAVITY_COMPENSATION;
  extern const GravityCompensationParameters RIGHT_ARM_GRAVITY_COMPENSATION;

  // ===================== 统一关节摩擦补偿参数 =====================
  //
  // 所有扭矩控制算法共用这里的摩擦补偿参数。
  // 左右臂分别配置，并且每条臂的 7 个主动关节分别配置。
  //
  // 补偿公式：
  //   tau_friction[i]
  //       = coulomb_nm[i] * tanh(dq[i] / velocity_smooth_rad_s[i])
  //       + viscous_nm_per_rad_s[i] * dq[i]
  //
  // 参数含义：
  //   coulomb_nm              -> 低速近似恒定的库仑摩擦补偿，单位 Nm；
  //   viscous_nm_per_rad_s    -> 随速度增加的粘性摩擦补偿，单位 Nm/(rad/s)；
  //   velocity_smooth_rad_s   -> 零速度附近的方向平滑范围，单位 rad/s。
  //
  // tau_friction 与实际关节速度方向相同，用于抵消机构摩擦。
  // 右臂最终发送时的整体负号仍由各控制算法原有的驱动方向映射负责。
  struct FrictionCompensationParameters
  {
    std::array<double, 7> coulomb_nm{};
    std::array<double, 7> viscous_nm_per_rad_s{};
    std::array<double, 7> velocity_smooth_rad_s{};
  };

  extern const FrictionCompensationParameters LEFT_ARM_FRICTION_COMPENSATION;
  extern const FrictionCompensationParameters RIGHT_ARM_FRICTION_COMPENSATION;

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

    // 右臂主动导纳模式
    MODE_RIGHT_ACTIVE = 14,

    // 双手空间虚拟弹簧主动模式
    MODE_ACTIVE_SPRING_3D = 15,

    // 毕设虚拟弹簧刚度实验模式
    MODE_EXPERIMENT_SPRING_STIFFNESS = 16,

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
  // 5. 允许不同轴使用不同控制方式，例如左臂位置模式、右臂扭矩模式；
  //    但同一个轴不能同时把 use_position 和 use_torque 设为 true。
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

    // ===================== 虚拟弹簧调试输出 =====================
    // 不参与控制执行，只用于 RobotState 发布给 GUI。
    // spring_debug_mode:
    // 0 -> 无弹簧
    // 1 -> X 向一维虚拟弹簧 MODE_ACTIVE_SPRING
    // 2 -> 空间三维虚拟弹簧 MODE_ACTIVE_SPRING_3D
    bool spring_debug_valid{false};
    int spring_debug_mode{0};

    double spring_e_x{0.0};
    double spring_e_y{0.0};
    double spring_e_z{0.0};
    double spring_e_norm{0.0};

    double spring_f_x{0.0};
    double spring_f_y{0.0};
    double spring_f_z{0.0};
    double spring_f_norm{0.0};
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

  // 重置双手空间虚拟弹簧模式的状态
  void resetActiveSpring3DMode();

  // 重置双侧主动模式的状态
  void resetBilateralActiveMode();

  // 重置右臂主动模式的导纳状态
  void resetRightActiveMode();

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
  // 右臂主动模式
  void runRightActiveMode(const MotorStateCache &state, const SensorStateCache &sensor_state, ControlOutputCache &output);
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

  // 双手空间虚拟弹簧主动模式
  void runActiveSpring3DMode(const MotorStateCache &state, const SensorStateCache &sensor_state, ControlOutputCache &output);

  namespace experiment
  {

    // ===================== 自由运动虚拟弹簧可视化实验 =====================
    //
    // 该实验继续沿用原来的 MODE_EXPERIMENT_SPRING_STIFFNESS 模式编号，
    // 但实验目的已经从“右侧固定端刚度标定”改为“自由运动直观验证”：
    //
    // 1. 左臂 joint1~joint7 使用位置模式，沿世界坐标 X 方向低速单次移动 10 cm；
    // 2. 右臂 joint11~joint17 使用扭矩模式，不再要求外部支架固定；
    // 3. 右臂基础输出包含重力补偿、关节阻尼和很弱的初始姿态保持；
    // 4. 虚拟弹簧根据左右大臂传感器 frame 的实际相对位移计算；
    // 5. 弹簧力通过右大臂 LOCAL Jacobian 做 J^T F，形成右臂关节弹簧力矩；
    // 6. 取消“右侧漂移超限退出”，右臂允许被左臂通过虚拟弹簧拖动；
    // 7. 右侧漂移仍然记录到 CSV，只作为观察量，不作为故障条件；
    // 8. 该模式用于验证方向、Jacobian 映射和拖动效果，不用于正式固定端刚度标定。
    //
    // 主动关节映射：
    // 右臂：控制层索引 0~6；
    // 左臂：控制层索引 10~16。
    //
    // 不可控末端关节：
    // 右臂 7~9、左臂 17~19，始终不发送控制命令。

    enum class ExperimentMode
    {
      Idle,
      ConstantTorque,
      SpringStiffness
    };

    enum class SpringLevel
    {
      Off,
      Low,
      Medium,
      High
    };

    enum class TrajectoryPhase
    {
      // 进入模式后先保持初始姿态，完成重力补偿接管和刚度渐入。
      Ready,

      // 左臂笛卡尔目标从初始位置低速移动到 10 cm 目标位置。
      Moving,

      // 左臂到达 10 cm 目标位置后持续保持，直到用户手动退出模式。
      // 当左臂实际大臂 frame 进入目标位置容差后，CSV 自动写入最后一行并关闭；
      // 控制仍继续运行，虚拟弹簧和右臂扭矩输出不会因为停止记录而停止。
      Holding
    };

    struct SpringExperimentConfig
    {
      // ===================== X 向虚拟弹簧参数 =====================
      //
      // 本实验只使用一个固定刚度。
      //
      // 左臂目标位移为 0.10 m 时，最大理论弹簧力为：
      //
      //   F = K × x = 300 N/m × 0.10 m = 30 N
      //
      // SpringLevel::Low / Medium / High 都使用该数值；
      // SpringLevel::Off 仍表示关闭虚拟弹簧。
      double spring_stiffness{300.0};

      // 笛卡尔相对速度阻尼，单位 N/(m/s)。
      // 它只抑制左右大臂沿实验轴的快速相对运动，
      // 在本次低速运动中影响很小，在快速抖动时提高稳定性。
      double spring_cartesian_damping{8.0};

      // 虚拟弹簧力绝对值上限，单位 N。
      // 300 N/m × 0.10 m = 30 N，因此设置为 30 N。
      double spring_force_limit{30.0};

      // 实际相对形变量的软件限幅，单位 m。
      // 左臂目标为 0.10 m，这里留出 0.02 m 余量。
      double extension_limit{0.12};

      // 相对速度的软件限幅，单位 m/s。
      double extension_rate_limit{0.50};

      // ===================== 左臂位置模式参数 =====================
      //
      // 左臂通过三维差分逆运动学跟踪大臂传感器 frame 的笛卡尔位置。
      //
      // v_cmd = target_velocity + left_cartesian_kp × position_error
      double left_cartesian_kp{2.0};

      // 笛卡尔速度命令合成限幅，单位 m/s。
      double left_cartesian_speed_limit{0.015};

      // 阻尼最小二乘逆运动学阻尼。
      double left_ik_damping{0.03};

      // 单关节差分逆运动学速度限幅，单位 rad/s。
      double left_joint_speed_limit{0.25};

      // 单关节相对进入实验时姿态的最大变化，单位 rad。
      // 0.60 rad 约为 34.4°。
      double left_joint_range_limit{0.60};

      // 关节位置目标最多领先实际关节位置的距离，单位 rad。
      // 0.05236 rad 约为 3°。
      double left_joint_lead_limit{0.0523598776};

      // 发送给执行层 Trap 位置规划器的参数。
      // 速度单位 deg/s，加速度和减速度单位 deg/s²。
      double left_position_velocity{12.0};
      double left_position_acc{30.0};
      double left_position_dec{30.0};
      short left_position_smooth_time{25};

      // 左臂目标与实际大臂 frame 的最大允许误差，单位 m。
      double left_tracking_error_limit{0.03};

      // Holding 阶段允许的左臂位置误差，单位 m。
      double left_position_tolerance{0.002};

      // ===================== 左臂单次 10 cm 低速运动 =====================
      //
      // 世界坐标 direction_world 默认为 +X。
      // 当前世界坐标定义：+X 向机器人左侧。
      Eigen::Vector3d direction_world{1.0, 0.0, 0.0};

      // 左臂从进入模式时的位置出发，只向指定方向移动一次。
      // 单位：m。0.10 m = 10 cm。
      double target_displacement{0.10};

      // 左臂笛卡尔目标移动速度，单位 m/s。
      // 0.005 m/s = 5 mm/s，移动 10 cm 大约需要 20 s。
      double move_speed{0.005};

      // 进入模式后先保持初始姿态，单位 s。
      double ready_time{1.0};

      // 虚拟弹簧刚度从 0 线性增加到 300 N/m 的时间，单位 s。
      double stiffness_ramp_time{1.5};

      // ===================== 右臂自由扭矩控制 =====================
      //
      // 数组顺序始终对应：
      //
      //   下标 0 -> 右臂 joint11 -> 控制层索引 0
      //   下标 1 -> 右臂 joint12 -> 控制层索引 1
      //   下标 2 -> 右臂 joint13 -> 控制层索引 2
      //   下标 3 -> 右臂 joint14 -> 控制层索引 3
      //   下标 4 -> 右臂 joint15 -> 控制层索引 4
      //   下标 5 -> 右臂 joint16 -> 控制层索引 5
      //   下标 6 -> 右臂 joint17 -> 控制层索引 6
      //
      // 右臂重力补偿不在本实验配置中重复保存。
      // 所有模式统一使用：
      //   RIGHT_ARM_GRAVITY_COMPENSATION.gain
      //   RIGHT_ARM_GRAVITY_COMPENSATION.bias_nm
      //
      // 右臂每个关节的最终模型方向力矩为：
      //
      // tau_gravity_compensated[i]
      //     = RIGHT_ARM_GRAVITY_COMPENSATION.gain[i] * gravity_model[i]
      //     + RIGHT_ARM_GRAVITY_COMPENSATION.bias_nm[i]
      //
      // tau_friction[i]
      //     = RIGHT_ARM_FRICTION_COMPENSATION.coulomb_nm[i]
      //       * tanh(dq[i] / RIGHT_ARM_FRICTION_COMPENSATION.velocity_smooth_rad_s[i])
      //     + RIGHT_ARM_FRICTION_COMPENSATION.viscous_nm_per_rad_s[i] * dq[i]
      //
      // tau_model[i]
      //     = tau_gravity_compensated[i]
      //     + tau_friction[i]
      //     + tau_spring[i]
      //     + tau_posture[i]
      //     - right_joint_damping[i] * dq[i]
      //
      // 之后代码仍按当前右臂驱动方向约定发送：
      //   tau_sent[i] = -tau_model[i]

      // --------------------- 每轴关节阻尼 ---------------------
      //
      // 单位：Nm/(rad/s)。
      // 该项只抑制运动速度和振荡，不能在静止时提供持续支撑力矩。
      // 若某轴下沉速度很快但重力比例已基本正确，可适度提高该轴阻尼。
      std::array<double, 7> right_joint_damping{
          0.80, // joint11
          0.80, // joint12
          0.80, // joint13
          0.80, // joint14
          0.80, // joint15
          0.80, // joint16
          0.80  // joint17
      };

      // --------------------- 每轴弱姿态保持刚度 ---------------------
      //
      // 单位：Nm/rad。
      //
      // tau_posture[i] = right_posture_kp[i] * (q_initial[i] - q_actual[i])
      //
      // 该项只用于抑制重力模型误差导致的慢漂移。
      // 数值过大会使右臂变硬，降低被虚拟弹簧拖动的直观效果。
      std::array<double, 7> right_posture_kp{
          1.0, // joint11
          1.0, // joint12
          1.0, // joint13
          1.0, // joint14
          1.0, // joint15
          1.0, // joint16
          1.0  // joint17
      };

      // --------------------- 每轴姿态保持力矩限幅 ---------------------
      //
      // 单位：Nm。
      // 限制弱姿态保持项，避免右臂偏离较大时产生过强回拉。
      std::array<double, 7> right_posture_torque_limit{
          1.5, // joint11
          1.5, // joint12
          1.5, // joint13
          1.5, // joint14
          1.5, // joint15
          1.5, // joint16
          1.5  // joint17
      };

      // ===================== 控制周期保护 =====================
      double dt_min{0.001};
      double dt_max{0.05};
    };

    class ExperimentControl
    {
    public:
      ExperimentControl() = default;

      void run(const MotorStateCache &state,
               const SensorStateCache &sensor_state,
               double dt_s,
               ControlOutputCache &output);

      void setExperimentMode(ExperimentMode mode);
      void setSpringLevel(SpringLevel level);
      void start();
      void stop();
      void reset();

      void setConstantTorque(std::size_t joint_index, double torque_nm);

      double selectedStiffness() const;

    private:
      // 每个控制周期内统一保存 Pinocchio 计算结果。
      struct RobotKinematics
      {
        Eigen::VectorXd q;
        Eigen::VectorXd v;

        Eigen::Vector3d p_left{Eigen::Vector3d::Zero()};
        Eigen::Vector3d p_right{Eigen::Vector3d::Zero()};

        Eigen::Vector3d v_left{Eigen::Vector3d::Zero()};
        Eigen::Vector3d v_right{Eigen::Vector3d::Zero()};

        // 左臂使用 LOCAL_WORLD_ALIGNED Jacobian 做差分逆运动学。
        Eigen::MatrixXd J_left_lwa;

        // 右臂使用 LOCAL Jacobian 映射局部传感器 frame 下的弹簧力。
        Eigen::MatrixXd J_right_local;

        Eigen::VectorXd gravity;

        // 右大臂传感器 frame 到世界坐标系的旋转矩阵。
        Eigen::Matrix3d right_rotation{Eigen::Matrix3d::Identity()};
      };

      // 简单数值工具。
      static double clampBetween(double value, double lo, double hi);
      static double clampAbs(double value, double limit);
      static bool finiteVec3(const Eigen::Vector3d &v);
      static bool finiteMatrix(const Eigen::MatrixXd &m);
      static Eigen::Vector3d limitNorm(const Eigen::Vector3d &v, double limit);

      double validDt(double dt_s) const;
      bool normalizedAxis(Eigen::Vector3d &axis) const;

      // Pinocchio 初始化和状态读取仍单独保留。
      // 这两部分包含大量模型索引检查，放进主循环会使核心控制逻辑难以阅读。
      bool initPinocchioOnce();
      bool loadJointIndex(const std::string &joint_name, int &q_id, int &v_id);
      bool readRobotState(const MotorStateCache &state, RobotKinematics &kin);

      void setSafeFault(const char *message);

      // 新的自由运动可视化实验绝大部分逻辑集中在本函数中：
      // 配置检查、初始化、单次低速轨迹、刚度渐入、弹簧力、J^T F、
      // 左臂逆运动学、右臂重力补偿、输出、CSV 和调试打印。
      void runSpringStiffnessExperiment(const MotorStateCache &state,
                                        const SensorStateCache &sensor_state,
                                        double dt_s,
                                        ControlOutputCache &output);

      void runConstantTorqueExperiment(ControlOutputCache &output);
      void runIdleExperiment(ControlOutputCache &output);

      ExperimentMode mode_{ExperimentMode::Idle};
      SpringLevel spring_level_{SpringLevel::Off};
      TrajectoryPhase phase_{TrajectoryPhase::Ready};
      SpringExperimentConfig config_;

      bool started_{false};
      bool initialized_{false};
      bool fault_{false};
      bool config_validated_{false};

      std::size_t debug_counter_{0};

      double phase_time_{0.0};
      double current_stiffness_{0.0};

      // 进入实验时记录的左右大臂 frame 位置。
      Eigen::Vector3d initial_left_position_{Eigen::Vector3d::Zero()};
      Eigen::Vector3d initial_right_position_{Eigen::Vector3d::Zero()};

      // 虚拟弹簧自然相对位置：
      // initial_relative_position = initial_left_position - initial_right_position。
      Eigen::Vector3d initial_relative_position_{Eigen::Vector3d::Zero()};

      // 左臂当前笛卡尔目标及目标速度。
      Eigen::Vector3d target_left_position_{Eigen::Vector3d::Zero()};
      Eigen::Vector3d target_left_velocity_{Eigen::Vector3d::Zero()};

      // 左臂位置命令状态。
      std::array<double, 7> initial_left_joint_position_rad_{};
      std::array<double, 7> left_joint_position_command_rad_{};

      // 右臂进入实验时的关节角。
      // 仅用于很弱的防慢漂姿态保持，不是刚性位置锁定。
      std::array<double, 7> initial_right_joint_position_rad_{};

      // 单轴恒扭矩调试参数。
      std::size_t torque_joint_index_{0};
      double torque_nm_{0.0};
      bool constant_torque_enabled_{false};

      // Pinocchio 模型。
      bool pin_initialized_{false};
      bool pin_init_failed_{false};
      pinocchio::Model model_;
      std::unique_ptr<pinocchio::Data> data_;
      pinocchio::FrameIndex left_frame_id_{0};
      pinocchio::FrameIndex right_frame_id_{0};

      // 仅包含 14 个主动关节。
      static constexpr std::array<std::size_t, 7> left_ctrl_id_ = {
          10, 11, 12, 13, 14, 15, 16};

      static constexpr std::array<std::size_t, 7> right_ctrl_id_ = {
          0, 1, 2, 3, 4, 5, 6};

      std::array<int, 7> left_q_id_ = {-1, -1, -1, -1, -1, -1, -1};
      std::array<int, 7> left_v_id_ = {-1, -1, -1, -1, -1, -1, -1};
      std::array<int, 7> right_q_id_ = {-1, -1, -1, -1, -1, -1, -1};
      std::array<int, 7> right_v_id_ = {-1, -1, -1, -1, -1, -1, -1};

      // 每次进入实验生成独立 CSV。
      std::ofstream csv_;
      bool csv_initialized_{false};
      std::size_t csv_row_count_{0};
    };

  } // namespace experiment

} // namespace upperlimb_robot