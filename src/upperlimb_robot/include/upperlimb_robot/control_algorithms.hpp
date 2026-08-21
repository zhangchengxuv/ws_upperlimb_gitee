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

    // 毕设 X 向虚拟弹簧刚度实验模式
    MODE_EXPERIMENT_SPRING_STIFFNESS = 16,

    // 毕设空间三维虚拟弹簧实验模式
    MODE_EXPERIMENT_SPATIAL_SPRING = 17,

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
    // 1 -> X 向一维虚拟弹簧 / 16号 X 向刚度实验
    // 2 -> 空间三维虚拟弹簧 MODE_ACTIVE_SPRING_3D
    // 3 -> 17号各向同性空间弹簧实验
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

    // ===================== 人工准静态虚拟弹簧刚度实验 =====================
    //
    // 该实验继续沿用 MODE_EXPERIMENT_SPRING_STIFFNESS 模式编号，实验方式改为：
    //
    // 1. 左臂 joint1~joint7 使用位置模式，固定在进入实验时的关节位置；
    // 2. 右臂 joint11~joint17 使用扭矩模式，由实验者沿世界坐标 X 方向缓慢移动；
    // 3. 进入实验后先在 K=0 条件下等待右臂稳定，然后重新记录弹簧自然位置；
    // 4. Ready 结束后直接启用唯一目标刚度 spring_stiffness；
    // 5. 虚拟弹簧根据左右大臂传感器 frame 的实际相对位移计算；
    // 6. 弹簧力通过右大臂 LOCAL Jacobian 做 J^T F，形成右臂关节弹簧力矩；
    // 7. 右臂不再加入弱姿态保持项，避免额外关节刚度污染力-位移关系；
    // 8. 六维力传感器只用于记录实际交互力，不反馈到控制器；
    // 9. CSV 持续记录到用户退出实验，便于完成加载/卸载全过程。
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
      SpringStiffness,
      SpatialSpring
    };

    // 为兼容现有 GUI / RobotControlNode 接口保留 SpringLevel。
    // Off 表示关闭虚拟弹簧；Low / Medium / High 均只表示“开启”，
    // 实际目标刚度唯一由 SpringExperimentConfig::spring_stiffness 决定。
    enum class SpringLevel
    {
      Off,
      Low,
      Medium,
      High
    };

    enum class TrajectoryPhase
    {
      // 左臂已经固定，虚拟弹簧关闭，等待右臂稳定。
      Ready,

      // Ready 结束时重新置零，然后直接启用目标刚度。
      // 实验者在该阶段缓慢移动右臂并记录力-位移数据。
      Measuring
    };

    struct SpringExperimentConfig
    {
      // ===================== 唯一目标刚度 =====================
      //
      // 16号 X 向实验和 17号空间弹簧实验共用这一组参数。
      // 17号采用各向同性空间弹簧：Kx = Ky = Kz = spring_stiffness。
      // 实验时只需要修改这一处，例如：100 / 200 / 300 N/m。
      double spring_stiffness{300.0};

      // 笛卡尔相对速度阻尼，单位 N/(m/s)。
      // 17号同样采用 Bx = By = Bz = spring_cartesian_damping。
      // 准静态移动时影响很小，主要用于快速运动时增加稳定性。
      double spring_cartesian_damping{8.0};

      // 不再对虚拟弹簧力和形变量做软件限幅。
      // Measuring 阶段始终按 F = K*x + B*xdot 计算。
      // 安全保护仍由右臂逐关节最终扭矩限幅负责。

      // 相对速度的软件限幅，单位 m/s。
      // 16号：限制轴向 extension_rate；
      // 17号：限制三维相对速度向量的模长。
      // 只用于抑制速度毛刺对 B*velocity 的放大，不限制弹簧位移或弹簧力。
      double extension_rate_limit{0.50};

      // ===================== 实验方向 =====================
      // 世界坐标 +X 当前定义为机器人左侧。
      // 左臂固定时，右臂沿 -X 移动会得到正 extension；沿 +X 移动得到负 extension。
      Eigen::Vector3d direction_world{1.0, 0.0, 0.0};

      // ===================== 实验时序 =====================
      // 进入实验后，先关闭虚拟弹簧并等待右臂稳定。
      // ready_time 结束时重新记录左右 frame 相对位置作为 extension=0，
      // 随后直接进入 Measuring 并启用 spring_stiffness。
      double ready_time{1.0};

      // ===================== 左臂固定位置模式参数 =====================
      // 左臂 7 个主动关节始终保持进入实验时的关节位置。
      double left_position_velocity{12.0};
      double left_position_acc{30.0};
      double left_position_dec{30.0};
      short left_position_smooth_time{25};

      // ===================== 右臂人工移动扭矩控制 =====================
      // 右臂最终模型方向力矩：
      //
      // tau_model = tau_gravity
      //           + tau_friction
      //           + tau_spring
      //           - right_joint_damping * dq
      //
      // 不加入关节姿态保持项，避免产生额外等效刚度。
      // 当前统一粘性摩擦补偿为 +0.4*dq。
      // 原来的 0.8 阻尼会形成约 -0.4*dq 的净速度阻尼，
      // 对虚拟弹簧自动回拉阻碍较明显，因此这里降低到接近 0.4。
      //
      // 现在只保留少量净稳定阻尼：
      // joint11/12: 0.50 -> 净约 -0.10*dq
      // joint13~15: 0.45 -> 净约 -0.05*dq
      // joint16/17: 0.40 -> 与 +0.4*dq 基本抵消
      std::array<double, 7> right_joint_damping{
          0.50, // joint11：近端主承载关节，保留约 0.10 Nm/(rad/s) 净稳定阻尼
          0.50, // joint12：近端主承载关节，保留约 0.10 Nm/(rad/s) 净稳定阻尼
          0.45, // joint13：保留约 0.05 Nm/(rad/s) 净稳定阻尼
          0.45, // joint14：保留约 0.05 Nm/(rad/s) 净稳定阻尼
          0.45, // joint15：保留约 0.05 Nm/(rad/s) 净稳定阻尼
          0.40, // joint16：与统一 +0.4*dq 粘性补偿基本抵消
          0.40  // joint17：与统一 +0.4*dq 粘性补偿基本抵消
      };

      // ===================== 右臂最终关节扭矩限幅 =====================
      //
      // 16号 X 向刚度实验和 17号空间弹簧实验共用这一组软件安全上限。
      // 数组顺序严格对应：joint11、joint12、joint13、joint14、joint15、joint16、joint17。
      // 单位统一为 Nm。
      //
      // 原实验上限：30 / 20 / 16 / 12 / 12 / 12 / 8 Nm。
      // 在较大位移下，虚拟弹簧产生的 tau_spring = J^T * F 会与重力、
      // 摩擦补偿和关节阻尼共同叠加。如果总模型力矩较早碰到旧上限，
      // 实际发送扭矩就会被裁剪，从而使理想的弹簧力-位移关系提前失真。
      //
      // 本版适度放宽为：40 / 30 / 24 / 18 / 18 / 18 / 12 Nm。
      // 这样可以给弹簧力矩留下更大的实现余量，但仍保留每个关节独立的软件限幅。
      // torque_saturated 会继续记录“理想合成力矩是否超过该上限”。
      //
      // 重要：这里是软件指令上限，不代表电机/减速器允许的额定或峰值扭矩。
      // 实机使用前必须确认这些数值不超过对应执行器和驱动器允许范围。
      std::array<double, 7> right_torque_limit{
          40.0, // joint11：近端主承载关节
          30.0, // joint12：近端主承载关节
          24.0, // joint13
          18.0, // joint14
          18.0, // joint15
          18.0, // joint16
          12.0  // joint17：远端关节，保留较低的软件保护上限
      };

      // ===================== 控制周期保护 =====================
      // 限制控制器采用的有效 dt，防止异常周期放大速度/阻尼离散计算误差。
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

      // 人工准静态刚度实验主体：
      // 左臂固定、右臂人工移动、稳定后重新置零并直接启用目标刚度、
      // 弹簧力、J^T F、右臂重力/摩擦/阻尼、CSV 和调试打印。
      void runSpringStiffnessExperiment(const MotorStateCache &state,
                                        const SensorStateCache &sensor_state,
                                        double dt_s,
                                        ControlOutputCache &output);

      // 17号各向同性空间弹簧实验：
      // e = (p_left - p_right) - initial_relative_position
      // F_world = K * e + B * e_dot
      // Kx = Ky = Kz，Bx = By = Bz。
      void runSpatialSpringExperiment(const MotorStateCache &state,
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

      // 进入实验时记录的左右大臂 frame 位置。
      Eigen::Vector3d initial_left_position_{Eigen::Vector3d::Zero()};
      Eigen::Vector3d initial_right_position_{Eigen::Vector3d::Zero()};

      // 虚拟弹簧自然相对位置：
      // initial_relative_position = initial_left_position - initial_right_position。
      Eigen::Vector3d initial_relative_position_{Eigen::Vector3d::Zero()};

      // 保留原有成员以兼容安全保持逻辑；本实验不再生成左臂笛卡尔运动轨迹。
      Eigen::Vector3d target_left_position_{Eigen::Vector3d::Zero()};
      Eigen::Vector3d target_left_velocity_{Eigen::Vector3d::Zero()};

      // 左臂位置命令状态。
      std::array<double, 7> initial_left_joint_position_rad_{};
      std::array<double, 7> left_joint_position_command_rad_{};

      // 保留右臂初始关节角用于调试/兼容，不参与姿态保持控制。
      std::array<double, 7> initial_right_joint_position_rad_{};

      // Ready 结束重新置零时记录的右侧轴向力。
      // 供 16号 X 向刚度实验使用。
      double initial_measured_right_axial_force_{0.0};

      // Ready 结束重新置零时记录的右侧三维世界坐标实测力。
      // 供 17号空间弹簧实验计算 zeroed force 和径向实测力。
      Eigen::Vector3d initial_measured_right_force_world_{Eigen::Vector3d::Zero()};

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