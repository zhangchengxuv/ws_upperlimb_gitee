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
    MODE_ACTIVE_SPRING_3D = 16,

    // 毕设虚拟弹簧刚度实验模式
    MODE_EXPERIMENT_SPRING_STIFFNESS = 15,

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

    // ===================== 毕设刚度实验模式 =====================
    //
    // 这个小实验名称：SpringStiffnessExperiment
    // 中文可称为“低中高虚拟弹簧刚度实验”。
    //
    // 注意：
    // 1. 这里只声明实验控制类；
    // 2. 具体算法实现放在 control_algorithms.cpp；
    // 3. 底层控制循环、电机输出、安全限幅仍沿用 RobotControlNode；
    // 4. 本实验只控制14个主动关节：右臂索引0~6、左臂索引10~16；
    // 5. 每条臂最后三个关节不参与控制：右臂7~9、左臂17~19。

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
      // 保持初始姿态：先等待 ready_time，再完成刚度渐入。
      Ready,

      // 目标位置沿实验轴以受限速度向当前阶梯移动。
      Moving,

      // 到达阶梯后，等待实际位置和速度稳定并保持 hold_time。
      Holding,

      // 最后一阶完成，不自动重启，也不自动切换刚度等级。
      Finished
    };

    struct SpringExperimentConfig
    {
      // 刚度单位 N/m。默认值只是软件初始值，未经过实机安全验证。
      double k_low{100.0};
      double k_medium{300.0};
      double k_high{500.0};

      // 左臂任务空间 PD，单位分别为 N/m 和 N*s/m。
      double left_kp_x{300.0};
      double left_kd_x{25.0};
      double left_kp_yz{60.0};
      double left_kd_yz{8.0};

      // 关节阻尼，单位 Nm/(rad/s)，只作为小阻尼。
      double joint_damping{0.35};

      Eigen::Vector3d direction_world{1.0, 0.0, 0.0};

      std::array<double, 7> step_displacements{
          0.0,
          0.005,
          0.010,
          0.015,
          0.010,
          0.005,
          0.0};

      // 左臂任务空间目标最大移动速度，单位 m/s。
      double move_speed{0.003};

      // 进入实验后保持 K=0 的静止等待时间，单位 s。
      double ready_time{1.0};

      // 每个阶梯在实际位置/速度稳定后继续保持的时间，单位 s。
      double hold_time{2.0};

      // 所选刚度从0线性渐入、结束或故障后线性渐出的时间，单位 s。
      double stiffness_ramp_time{1.0};

      double extension_limit{0.015};
      double spring_force_limit{10.0};

      double left_task_force_limit{12.0};
      double left_tracking_error_limit{0.01};
      double right_drift_limit{0.002};

      // Holding 阶段只有在实际位置和速度稳定后才开始计时。
      double position_tolerance{0.001};
      double velocity_threshold{0.002};

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
      struct RobotKinematics
      {
        Eigen::VectorXd q;
        Eigen::VectorXd v;
        Eigen::Vector3d p_left{Eigen::Vector3d::Zero()};
        Eigen::Vector3d p_right{Eigen::Vector3d::Zero()};
        Eigen::Vector3d v_left{Eigen::Vector3d::Zero()};
        Eigen::Vector3d v_right{Eigen::Vector3d::Zero()};
        Eigen::MatrixXd J_left_lwa;
        Eigen::MatrixXd J_right_local;
        Eigen::VectorXd gravity;
        Eigen::Matrix3d right_rotation{Eigen::Matrix3d::Identity()};
      };

      static double clampBetween(double value, double lo, double hi);
      static double clampAbs(double value, double limit);
      static bool finiteVec3(const Eigen::Vector3d &v);
      static bool finiteMatrix(const Eigen::MatrixXd &m);
      static Eigen::Vector3d limitNorm(const Eigen::Vector3d &v, double limit);

      double validDt(double dt_s) const;
      bool normalizedAxis(Eigen::Vector3d &axis) const;
      bool checkConfig();

      bool initPinocchioOnce();
      bool loadJointIndex(const std::string &joint_name, int &q_id, int &v_id);
      bool readRobotState(const MotorStateCache &state, RobotKinematics &kin);
      void initializeExperiment(const RobotKinematics &kin);

      void updateTrajectory(double dt,
                            const Eigen::Vector3d &axis_world,
                            const RobotKinematics &kin);
      void updateStiffness(double dt);

      Eigen::VectorXd computeLeftTrackingTorque(
          const RobotKinematics &kin,
          const Eigen::Vector3d &axis_world);

      Eigen::VectorXd computeRightSpringTorque(
          const RobotKinematics &kin,
          const Eigen::Vector3d &axis_world,
          double &extension,
          double &spring_force_scalar);

      bool writeTorqueOutput(const RobotKinematics &kin,
                             const Eigen::VectorXd &tau_left_track,
                             const Eigen::VectorXd &tau_right_spring,
                             ControlOutputCache &output);
      void writeZeroTorqueOutput(ControlOutputCache &output);

      double measuredRightAxialForce(const SensorStateCache &sensor_state,
                                     const RobotKinematics &kin,
                                     const Eigen::Vector3d &axis_world) const;

      void writeLog(double dt,
                    const RobotKinematics &kin,
                    double extension,
                    double theoretical_right_axial_force,
                    double measured_right_axial_force,
                    double right_drift,
                    bool torque_saturated);
      void openLogFile();

      const char *springLevelName() const;
      const char *trajectoryPhaseName() const;
      void setSafeFault(const char *message);

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

      std::size_t step_index_{0};

      double phase_time_{0.0};
      double current_stiffness_{0.0};

      Eigen::Vector3d initial_left_position_{Eigen::Vector3d::Zero()};
      Eigen::Vector3d initial_right_position_{Eigen::Vector3d::Zero()};
      Eigen::Vector3d initial_relative_position_{Eigen::Vector3d::Zero()};
      Eigen::Vector3d target_left_position_{Eigen::Vector3d::Zero()};
      Eigen::Vector3d target_left_velocity_{Eigen::Vector3d::Zero()};

      std::size_t torque_joint_index_{0};
      double torque_nm_{0.0};
      bool constant_torque_enabled_{false};

      bool pin_initialized_{false};
      bool pin_init_failed_{false};
      pinocchio::Model model_;
      std::unique_ptr<pinocchio::Data> data_;
      pinocchio::FrameIndex left_frame_id_{0};
      pinocchio::FrameIndex right_frame_id_{0};

      // 仅包含14个主动关节。不可控末端索引7~9、17~19不在映射中。
      static constexpr std::array<std::size_t, 7> left_ctrl_id_ = {
          10, 11, 12, 13, 14, 15, 16};
      static constexpr std::array<std::size_t, 7> right_ctrl_id_ = {
          0, 1, 2, 3, 4, 5, 6};

      std::array<int, 7> left_q_id_ = {-1, -1, -1, -1, -1, -1, -1};
      std::array<int, 7> left_v_id_ = {-1, -1, -1, -1, -1, -1, -1};
      std::array<int, 7> right_q_id_ = {-1, -1, -1, -1, -1, -1, -1};
      std::array<int, 7> right_v_id_ = {-1, -1, -1, -1, -1, -1, -1};

      std::ofstream csv_;
      bool csv_initialized_{false};
      std::size_t csv_row_count_{0};
    };

  } // namespace experiment

} // namespace upperlimb_robot