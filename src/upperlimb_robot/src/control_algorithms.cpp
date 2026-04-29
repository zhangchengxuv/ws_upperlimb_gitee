#include "upperlimb_robot/control_algorithms.hpp"

#include <cstdlib>
#include <iostream>
#include <memory>

#include <fstream>
#include <chrono>
#include <ctime>
#include <iomanip>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <Eigen/Core>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/spatial/force.hpp>

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

  // ===================== 主动模式 =====================
  void runActiveMode(const MotorStateCache &state, const SensorStateCache &sensor_state, ControlOutputCache &output)
  {
    // ===================== 输出初始化 =====================
    //
    // 每次进入先清空 output，
    // 然后只对需要控制的关节写入扭矩输出或位置保持输出。
    //
    output = ControlOutputCache{};
    output.valid = false;

    // ===================== 左臂 joint1~joint7 =====================
    //
    // 当前实验设定：
    // 1. URDF 中 joint1~3 已经改成 revolute，joint8~10 保持 fixed
    // 2. Pinocchio 模型现在有 7 个可动关节：joint1~joint7
    // 3. 这里 joint1~7 全部参与主动模式扭矩控制
    //
    constexpr std::size_t IDX_J1 = 10; // joint1 -> 左臂控制层索引
    constexpr std::size_t IDX_J2 = 11; // joint2 -> 左臂控制层索引
    constexpr std::size_t IDX_J3 = 12; // joint3 -> 左臂控制层索引
    constexpr std::size_t IDX_J4 = 13; // joint4 -> 左臂控制层索引
    constexpr std::size_t IDX_J5 = 14; // joint5 -> 左臂控制层索引
    constexpr std::size_t IDX_J6 = 15; // joint6 -> 左臂控制层索引
    constexpr std::size_t IDX_J7 = 16; // joint7 -> 左臂控制层索引

    // ===================== Pinocchio 懒加载上下文 =====================
    //
    // 不另外写很多函数，直接在本函数里静态初始化。
    // 第一次进入时加载 URDF、查 joint/frame id；
    // 后续周期只做实时计算。
    //
    static bool pin_initialized = false;
    static bool pin_init_failed = false;
    static pinocchio::Model model;
    static std::unique_ptr<pinocchio::Data> data;
    static pinocchio::FrameIndex frame_id_arm = 0;
    static pinocchio::FrameIndex frame_id_hand = 0;

    // ===================== 导纳内部状态 =====================
    //
    // q_adm  : 导纳生成的参考关节角（rad）
    // dq_adm : 导纳生成的参考关节角速度（rad/s）
    //
    // 说明：
    // 1. 这里使用静态变量保存导纳内部状态
    // 2. 第一次进入主动模式时，用当前实际姿态初始化，避免一上电就跳变
    //
    static bool admittance_initialized = false;
    static Eigen::VectorXd q_adm;
    static Eigen::VectorXd dq_adm;

    if (!pin_initialized && !pin_init_failed)
    {
      try
      {
        // ===================== URDF 路径 =====================
        // upperlimb_robot/urdf/left_arm_experiment.urdf
        //
        std::string urdf_path =
            ament_index_cpp::get_package_share_directory("upperlimb_robot") +
            "/urdf/left_arm_experiment.urdf";

        // ===================== 加载 URDF =====================
        //
        // joint1~3 已改成 revolute，joint8~10 保持 fixed，
        // model 里应有 joint1~7 这 7 个可动关节
        //
        pinocchio::urdf::buildModel(urdf_path, model);
        data = std::make_unique<pinocchio::Data>(model);

        // ===================== 获取两个六维力传感器 frame =====================
        //
        // ft_sensor_frame     -> 大臂绑缚件
        // ft_sensor_frame_2   -> 手部绑缚件
        //
        frame_id_arm = model.getFrameId("ft_sensor_frame");
        frame_id_hand = model.getFrameId("ft_sensor_frame_2");

        if (frame_id_arm >= model.frames.size())
        {
          std::cerr << "[runActiveMode] 没找到 frame: ft_sensor_frame" << std::endl;
          pin_init_failed = true;
        }
        else if (frame_id_hand >= model.frames.size())
        {
          std::cerr << "[runActiveMode] 没找到 frame: ft_sensor_frame_2" << std::endl;
          pin_init_failed = true;
        }
        else if (model.nq != 7 || model.nv != 7)
        {
          std::cerr
              << "[runActiveMode] 当前 Pinocchio 模型自由度不是 7，"
              << "请确认 joint1~3 已改成 revolute，joint8~10 保持 fixed。"
              << " model.nq=" << model.nq << " model.nv=" << model.nv
              << std::endl;
          pin_init_failed = true;
        }
        else
        {
          pin_initialized = true;
          std::cout
              << "[runActiveMode] Pinocchio 初始化成功。"
              << " urdf_path=" << urdf_path
              << " nq=" << model.nq
              << " nv=" << model.nv
              << std::endl;
        }
      }
      catch (const std::exception &e)
      {
        std::cerr << "[runActiveMode] Pinocchio 初始化异常: " << e.what() << std::endl;
        pin_init_failed = true;
      }
    }

    if (!pin_initialized || pin_init_failed || !data)
    {
      // 初始化失败则本轮不输出
      output.valid = false;
      return;
    }

    // ===================== 传感器有效性检查 =====================
    //
    // 当前传感器无效时，不要输出扭矩。
    //
    if (!sensor_state.valid)
    {
      output.valid = false;
      return;
    }

    // ===================== 组装 q / dq =====================
    //
    // 注意：
    // 1. state.position / velocity 的单位是 deg / deg/s
    // 2. Pinocchio 这里使用 rad / rad/s
    // 3. 当前模型自由度为 7：
    //    q = [joint1, joint2, joint3, joint4, joint5, joint6, joint7]
    //
    constexpr double DEG2RAD = M_PI / 180.0;

    Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);

    q(0) = state.position[IDX_J1] * DEG2RAD;
    q(1) = state.position[IDX_J2] * DEG2RAD;
    q(2) = state.position[IDX_J3] * DEG2RAD;
    q(3) = state.position[IDX_J4] * DEG2RAD;
    q(4) = state.position[IDX_J5] * DEG2RAD;
    q(5) = state.position[IDX_J6] * DEG2RAD;
    q(6) = state.position[IDX_J7] * DEG2RAD;

    v(0) = state.velocity[IDX_J1] * DEG2RAD;
    v(1) = state.velocity[IDX_J2] * DEG2RAD;
    v(2) = state.velocity[IDX_J3] * DEG2RAD;
    v(3) = state.velocity[IDX_J4] * DEG2RAD;
    v(4) = state.velocity[IDX_J5] * DEG2RAD;
    v(5) = state.velocity[IDX_J6] * DEG2RAD;
    v(6) = state.velocity[IDX_J7] * DEG2RAD;

    // ===================== 导纳状态初始化 =====================
    //
    // 第一次进入主动模式时，用当前实际姿态初始化导纳参考。
    //
    if (!admittance_initialized)
    {
      q_adm = q;
      dq_adm = Eigen::VectorXd::Zero(model.nv);
      admittance_initialized = true;
    }

    // ===================== 读取并去偏两个六维力传感器 =====================
    //
    // 左臂：
    // SENSOR_LEFT_HAND = 2
    // SENSOR_LEFT_ARM  = 3
    //
    // 约定：
    // 1. sensor_state.sensor[] 是实时值
    // 2. sensor_state.bias[]   是进入模式时记录的零偏
    // 3. 这里直接做 raw - bias
    //
    const auto &raw_hand = sensor_state.sensor[SENSOR_LEFT_HAND];
    const auto &bias_hand = sensor_state.bias[SENSOR_LEFT_HAND];

    const auto &raw_arm = sensor_state.sensor[SENSOR_LEFT_ARM];
    const auto &bias_arm = sensor_state.bias[SENSOR_LEFT_ARM];

    double fx_arm = raw_arm.fx - bias_arm.fx;
    double fy_arm = raw_arm.fy - bias_arm.fy;
    double fz_arm = raw_arm.fz - bias_arm.fz;
    double mx_arm = raw_arm.mx - bias_arm.mx;
    double my_arm = raw_arm.my - bias_arm.my;
    double mz_arm = raw_arm.mz - bias_arm.mz;

    double fx_hand = raw_hand.fx - bias_hand.fx;
    double fy_hand = raw_hand.fy - bias_hand.fy;
    double fz_hand = raw_hand.fz - bias_hand.fz;
    double mx_hand = raw_hand.mx - bias_hand.mx;
    double my_hand = raw_hand.my - bias_hand.my;
    double mz_hand = raw_hand.mz - bias_hand.mz;

    // ===================== 传感器方向符号修正 =====================
    //
    // 顺序：
    // [Fx, Fy, Fz, Mx, My, Mz]
    //
    // 如果后面发现某个方向“人往前推，系统却反着帮”，
    // 优先在这里改符号，不要先去改导纳参数。
    //
    static const std::array<double, 6> arm_sign = {
        1.0, 1.0, 1.0,
        1.0, 1.0, 1.0};

    static const std::array<double, 6> hand_sign = {
        1.0, 1.0, 1.0,
        1.0, 1.0, 1.0};

    fx_arm *= arm_sign[0];
    fy_arm *= arm_sign[1];
    fz_arm *= arm_sign[2];
    mx_arm *= arm_sign[3];
    my_arm *= arm_sign[4];
    mz_arm *= arm_sign[5];

    fx_hand *= hand_sign[0];
    fy_hand *= hand_sign[1];
    fz_hand *= hand_sign[2];
    mx_hand *= hand_sign[3];
    my_hand *= hand_sign[4];
    mz_hand *= hand_sign[5];

    // ===================== 死区 =====================
    //
    // 力 / 力矩很小时直接置 0，防止噪声抖动。
    //
    const double force_deadzone = 1.0;   // N
    const double torque_deadzone = 0.05; // Nm

    auto apply_deadzone = [](double x, double dz) -> double
    {
      return (std::abs(x) < dz) ? 0.0 : x;
    };

    fx_arm = apply_deadzone(fx_arm, force_deadzone);
    fy_arm = apply_deadzone(fy_arm, force_deadzone);
    fz_arm = apply_deadzone(fz_arm, force_deadzone);
    mx_arm = apply_deadzone(mx_arm, torque_deadzone);
    my_arm = apply_deadzone(my_arm, torque_deadzone);
    mz_arm = apply_deadzone(mz_arm, torque_deadzone);

    fx_hand = apply_deadzone(fx_hand, force_deadzone);
    fy_hand = apply_deadzone(fy_hand, force_deadzone);
    fz_hand = apply_deadzone(fz_hand, force_deadzone);
    mx_hand = apply_deadzone(mx_hand, torque_deadzone);
    my_hand = apply_deadzone(my_hand, torque_deadzone);
    mz_hand = apply_deadzone(mz_hand, torque_deadzone);

    // ===================== 限幅 =====================
    //
    // 防止异常值把控制打飞。
    //
    const double force_limit = 80.0; // N
    const double moment_limit = 8.0; // Nm

    auto clamp_abs = [](double x, double lim) -> double
    {
      if (x > lim)
        return lim;
      if (x < -lim)
        return -lim;
      return x;
    };

    auto clamp_between = [](double x, double lo, double hi) -> double
    {
      if (x < lo)
        return lo;
      if (x > hi)
        return hi;
      return x;
    };

    fx_arm = clamp_abs(fx_arm, force_limit);
    fy_arm = clamp_abs(fy_arm, force_limit);
    fz_arm = clamp_abs(fz_arm, force_limit);
    mx_arm = clamp_abs(mx_arm, moment_limit);
    my_arm = clamp_abs(my_arm, moment_limit);
    mz_arm = clamp_abs(mz_arm, moment_limit);

    fx_hand = clamp_abs(fx_hand, force_limit);
    fy_hand = clamp_abs(fy_hand, force_limit);
    fz_hand = clamp_abs(fz_hand, force_limit);
    mx_hand = clamp_abs(mx_hand, moment_limit);
    my_hand = clamp_abs(my_hand, moment_limit);
    mz_hand = clamp_abs(mz_hand, moment_limit);

    // ===================== 计算两个传感器 frame 的 Jacobian =====================
    //
    // 把“末端（传感器）空间的力/速度”映射到“关节空间”
    //
    Eigen::MatrixXd J_arm = Eigen::MatrixXd::Zero(6, model.nv);
    Eigen::MatrixXd J_hand = Eigen::MatrixXd::Zero(6, model.nv);

    pinocchio::computeFrameJacobian(model, *data, q, frame_id_arm, pinocchio::LOCAL, J_arm);
    pinocchio::computeFrameJacobian(model, *data, q, frame_id_hand, pinocchio::LOCAL, J_hand);

    // ===================== 构造 Pinocchio 空间力 =====================
    //
    // Pinocchio 空间力的顺序是：
    // [angular; linear]
    //
    pinocchio::Force f_arm = pinocchio::Force::Zero();
    f_arm.linear() << fx_arm, fy_arm, fz_arm;
    f_arm.angular() << mx_arm, my_arm, mz_arm;

    pinocchio::Force f_hand = pinocchio::Force::Zero();
    f_hand.linear() << fx_hand, fy_hand, fz_hand;
    f_hand.angular() << mx_hand, my_hand, mz_hand;

    // ===================== 外力映射到关节空间 =====================
    //
    // 数学意义：
    // tau_ext = J_arm^T * F_arm + J_hand^T * F_hand
    //
    // 这里得到的是“患者作用到系统上的等效关节力矩”，
    // 后面作为导纳模型的输入。
    //
    Eigen::VectorXd tau_arm = J_arm.transpose() * f_arm.toVector();
    Eigen::VectorXd tau_hand = J_hand.transpose() * f_hand.toVector();
    Eigen::VectorXd tau_ext = tau_arm + tau_hand;

    // ===================== 导纳模型 =====================
    //
    // 采用关节空间导纳：
    //
    //   M_a * ddq_adm + D_a * dq_adm + K_a * (q_adm - q) = tau_ext
    //
    // 其中：
    // q        -> 当前实际关节角
    // q_adm    -> 导纳生成的参考关节角
    // dq_adm   -> 导纳生成的参考关节角速度
    // tau_ext  -> 由六维力传感器映射得到的等效关节交互力矩
    //
    // 物理意义：
    // 1. M_a 决定“推起来有多沉”
    // 2. D_a 决定“推起来有多黏”
    // 3. K_a 决定“是否有回中/回稳趋势”
    //
    // 数值积分：
    // ddq_adm -> dq_adm -> q_adm
    //
    const double dt = 1.0 / 90.0; // 当前控制线程频率约为 90 Hz

    Eigen::VectorXd M_a = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd D_a = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd K_a = Eigen::VectorXd::Zero(model.nv);

    // 导纳参数
    // 背部这 3 个关节比较重，前臂这 4 个关节比较轻。
    // 对手端交互会产生更大耦合
    M_a(0) = 4.0;
    M_a(1) = 4.0;
    M_a(2) = 3.0;
    M_a(3) = 1.5;
    M_a(4) = 1.5;
    M_a(5) = 1.2;
    M_a(6) = 0.8;

    D_a(0) = 45.0;
    D_a(1) = 45.0;
    D_a(2) = 35.0;
    D_a(3) = 20.0;
    D_a(4) = 20.0;
    D_a(5) = 18.0;
    D_a(6) = 12.0;

    K_a(0) = 18.0;
    K_a(1) = 18.0;
    K_a(2) = 14.0;
    K_a(3) = 8.0;
    K_a(4) = 8.0;
    K_a(5) = 6.0;
    K_a(6) = 4.0;

    Eigen::VectorXd ddq_adm = Eigen::VectorXd::Zero(model.nv);

    for (int i = 0; i < model.nv; ++i)
    {
      ddq_adm(i) =
          (tau_ext(i) - D_a(i) * dq_adm(i) - K_a(i) * (q_adm(i) - q(i))) /
          M_a(i);
    }

    // 加速度 → 速度 v = v + a * dt
    dq_adm = dq_adm + ddq_adm * dt;
    // 速度 → 位置 q = q + v * dt
    q_adm = q_adm + dq_adm * dt;

    // ===================== 导纳参考限位 =====================
    //
    // 防止导纳内部状态在长时间交互下跑飞。
    // 这里给一个保守范围，后续可按实际关节安全范围继续调整。
    //
    const double q1_min = -30.0 * DEG2RAD;
    const double q1_max = 30.0 * DEG2RAD;
    const double q2_min = -30.0 * DEG2RAD;
    const double q2_max = 30.0 * DEG2RAD;
    const double q3_min = -30.0 * DEG2RAD;
    const double q3_max = 30.0 * DEG2RAD;
    const double q4_min = -60.0 * DEG2RAD;
    const double q4_max = 60.0 * DEG2RAD;
    const double q5_min = -60.0 * DEG2RAD;
    const double q5_max = 60.0 * DEG2RAD;
    const double q6_min = -60.0 * DEG2RAD;
    const double q6_max = 60.0 * DEG2RAD;
    const double q7_min = 0.0 * DEG2RAD;
    const double q7_max = 120.0 * DEG2RAD;

    q_adm(0) = clamp_between(q_adm(0), q1_min, q1_max);
    q_adm(1) = clamp_between(q_adm(1), q2_min, q2_max);
    q_adm(2) = clamp_between(q_adm(2), q3_min, q3_max);
    q_adm(3) = clamp_between(q_adm(3), q4_min, q4_max);
    q_adm(4) = clamp_between(q_adm(4), q5_min, q5_max);
    q_adm(5) = clamp_between(q_adm(5), q6_min, q6_max);
    q_adm(6) = clamp_between(q_adm(6), q7_min, q7_max);

    // 同时限制参考速度，避免导纳速度积累过大
    const double dq_adm_limit_j123 = 1.0;  // rad/s
    const double dq_adm_limit_j4567 = 1.5; // rad/s

    dq_adm(0) = clamp_abs(dq_adm(0), dq_adm_limit_j123);
    dq_adm(1) = clamp_abs(dq_adm(1), dq_adm_limit_j123);
    dq_adm(2) = clamp_abs(dq_adm(2), dq_adm_limit_j123);
    dq_adm(3) = clamp_abs(dq_adm(3), dq_adm_limit_j4567);
    dq_adm(4) = clamp_abs(dq_adm(4), dq_adm_limit_j4567);
    dq_adm(5) = clamp_abs(dq_adm(5), dq_adm_limit_j4567);
    dq_adm(6) = clamp_abs(dq_adm(6), dq_adm_limit_j4567);

    // ===================== 重力补偿 =====================
    //
    // g(q) 是系统在当前姿态下抵消自身重力所需的关节力矩。
    //
    // 数学意义：
    //   g(q) = computeGeneralizedGravity(model, data, q)
    //
    // 物理意义：
    // 1. 不让患者额外承担外骨骼自身重量
    // 2. 让导纳看到的交互更接近“患者真实意图”
    //
    Eigen::VectorXd g = pinocchio::computeGeneralizedGravity(model, *data, q);

    // ===================== 扭矩跟踪控制 =====================
    //
    // 最终控制律：
    //
    //   tau_cmd = g(q) + Kp * (q_adm - q) + Kv * (dq_adm - v)
    //
    // 其中：
    // 1. g(q)                     -> 重力补偿
    // 2. Kp * (q_adm - q)         -> 把系统拉向导纳生成的参考位置
    // 3. Kv * (dq_adm - v)        -> 提供速度跟踪与阻尼稳定性
    //
    // 这意味着：
    // 1. 导纳负责“根据人的力生成想去的位置/速度”
    // 2. 扭矩控制负责“把这个想去的位置/速度真正实现出来”
    //
    Eigen::VectorXd Kp = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd Kv = Eigen::VectorXd::Zero(model.nv);

    // 扭矩控制参数
    Kp(0) = 20.0;
    Kp(1) = 20.0;
    Kp(2) = 16.0;
    Kp(3) = 18.0;
    Kp(4) = 18.0;
    Kp(5) = 16.0;
    Kp(6) = 10.0;

    Kv(0) = 4.0;
    Kv(1) = 4.0;
    Kv(2) = 3.0;
    Kv(3) = 2.5;
    Kv(4) = 2.5;
    Kv(5) = 2.0;
    Kv(6) = 1.5;

    Eigen::VectorXd tau_cmd = g + Kp.asDiagonal() * (q_adm - q) + Kv.asDiagonal() * (dq_adm - v);

    // ===================== 最终扭矩限幅 =====================
    //
    // 输出给执行层前，限制每个关节的目标扭矩。
    //
    const double joint_torque_limit_j1 = 24.0;
    const double joint_torque_limit_j2 = 24.0;
    const double joint_torque_limit_j3 = 18.0;
    const double joint_torque_limit_j4 = 12.0;
    const double joint_torque_limit_j5 = 12.0;
    const double joint_torque_limit_j6 = 12.0;
    const double joint_torque_limit_j7 = 8.0;

    tau_cmd(0) = clamp_abs(tau_cmd(0), joint_torque_limit_j1);
    tau_cmd(1) = clamp_abs(tau_cmd(1), joint_torque_limit_j2);
    tau_cmd(2) = clamp_abs(tau_cmd(2), joint_torque_limit_j3);
    tau_cmd(3) = clamp_abs(tau_cmd(3), joint_torque_limit_j4);
    tau_cmd(4) = clamp_abs(tau_cmd(4), joint_torque_limit_j5);
    tau_cmd(5) = clamp_abs(tau_cmd(5), joint_torque_limit_j6);
    tau_cmd(6) = clamp_abs(tau_cmd(6), joint_torque_limit_j7);

    // ===================== 输出到控制层 =====================
    //
    // 这里对 joint1~7 对应的控制层索引全部写扭矩。
    // 这些轴在主动模式下使用扭矩控制。
    //
    output.use_torque[IDX_J1] = true;
    output.use_torque[IDX_J2] = true;
    output.use_torque[IDX_J3] = true;
    output.use_torque[IDX_J4] = true;
    output.use_torque[IDX_J5] = true;
    output.use_torque[IDX_J6] = true;
    output.use_torque[IDX_J7] = true;

    output.target_torque[IDX_J1] = tau_cmd(0);
    output.target_torque[IDX_J2] = tau_cmd(1);
    output.target_torque[IDX_J3] = tau_cmd(2);
    output.target_torque[IDX_J4] = tau_cmd(3);
    output.target_torque[IDX_J5] = tau_cmd(4);
    output.target_torque[IDX_J6] = tau_cmd(5);
    output.target_torque[IDX_J7] = tau_cmd(6);

    // ===================== 末端其余关节固定 =====================
    //
    // 固定左臂物理轴18、19、20，避免模式切换后末端发飘。
    //
    output.use_position[17] = true;
    output.target_position[17] = 0.0;
    output.target_velocity[17] = pulsePerMsToJointSpeedDegPerSecMagnitude(18, 1500.0);
    output.target_acc[17] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(18, 5.0);
    output.target_dec[17] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(18, 0.25);
    output.target_smooth_time[17] = 25;

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

    // ===================== 输出有效 =====================
    output.valid = true;
  }

  // ===================== 左臂零力模式 =====================
  void runZeroForceMode(const MotorStateCache &state, const SensorStateCache &sensor_state, ControlOutputCache &output)
  {
    // ===================== 输出初始化 =====================
    //
    // 这里每次进入都先清空 output，
    // 然后只对需要控制的关节写入扭矩输出。
    //
    output = ControlOutputCache{};
    output.valid = false;

    // ===================== 左臂 joint1~joint7 =====================
    //
    // 当前实验设定：
    // 1. URDF 中 joint1~3 已经改成 revolute，joint8~10 保持 fixed
    // 2. Pinocchio 模型现在有 7 个可动关节：joint1~joint7
    // 3. 这里 joint1~7 全部参与零力模式扭矩控制
    //
    // 注意：
    // 下面这 7 个控制层索引，必须与你当前左臂真实映射一致。
    //
    constexpr std::size_t IDX_J1 = 10; // joint1 -> 左臂控制层索引
    constexpr std::size_t IDX_J2 = 11; // joint2 -> 左臂控制层索引
    constexpr std::size_t IDX_J3 = 12; // joint3 -> 左臂控制层索引
    constexpr std::size_t IDX_J4 = 13; // joint4 -> 左臂控制层索引
    constexpr std::size_t IDX_J5 = 14; // joint5 -> 左臂控制层索引
    constexpr std::size_t IDX_J6 = 15; // joint6 -> 左臂控制层索引
    constexpr std::size_t IDX_J7 = 16; // joint7 -> 左臂控制层索引

    // ===================== Pinocchio 懒加载上下文 =====================
    //
    // 不另外写很多函数，直接在本函数里静态初始化。
    // 第一次进入时加载 URDF、查 joint/frame id；
    // 后续周期只做实时计算。
    //
    static bool pin_initialized = false;
    static bool pin_init_failed = false;
    static pinocchio::Model model;
    static std::unique_ptr<pinocchio::Data> data;
    static pinocchio::FrameIndex frame_id_arm = 0;
    static pinocchio::FrameIndex frame_id_hand = 0;

    if (!pin_initialized && !pin_init_failed)
    {
      try
      {
        // ===================== URDF 路径 =====================
        // upperlimb_robot/urdf/left_arm_experiment.urdf
        //
        std::string urdf_path =
            ament_index_cpp::get_package_share_directory("upperlimb_robot") +
            "/urdf/left_arm_experiment.urdf";

        // ===================== 加载 URDF =====================
        //
        // joint1~3 已改成 revolute，joint8~10 保持 fixed，
        // model 里应有 joint1~7 这 7 个可动关节
        //
        pinocchio::urdf::buildModel(urdf_path, model);
        data = std::make_unique<pinocchio::Data>(model);

        // ===================== 获取两个六维力传感器 frame =====================
        //
        // ft_sensor_frame     -> 大臂绑缚件
        // ft_sensor_frame_2   -> 手部绑缚件
        //
        frame_id_arm = model.getFrameId("ft_sensor_frame");
        frame_id_hand = model.getFrameId("ft_sensor_frame_2");

        if (frame_id_arm >= model.frames.size())
        {
          std::cerr << "[runZeroForceMode] 没找到 frame: ft_sensor_frame" << std::endl;
          pin_init_failed = true;
        }
        else if (frame_id_hand >= model.frames.size())
        {
          std::cerr << "[runZeroForceMode] 没找到 frame: ft_sensor_frame_2" << std::endl;
          pin_init_failed = true;
        }
        else if (model.nq != 7 || model.nv != 7)
        {
          std::cerr
              << "[runZeroForceMode] 当前 Pinocchio 模型自由度不是 7，"
              << "请确认 joint1~3 已改成 revolute，joint8~10 保持 fixed。"
              << " model.nq=" << model.nq << " model.nv=" << model.nv
              << std::endl;
          pin_init_failed = true;
        }
        else
        {
          pin_initialized = true;
          std::cout
              << "[runZeroForceMode] Pinocchio 初始化成功。"
              << " urdf_path=" << urdf_path
              << " nq=" << model.nq
              << " nv=" << model.nv
              << std::endl;
        }
      }
      catch (const std::exception &e)
      {
        std::cerr << "[runZeroForceMode] Pinocchio 初始化异常: " << e.what() << std::endl;
        pin_init_failed = true;
      }
    }

    if (!pin_initialized || pin_init_failed || !data)
    {
      // 初始化失败则本轮不输出
      output.valid = false;
      return;
    }

    // ===================== 传感器有效性检查 =====================
    //
    // 当前传感器无效时，不要输出扭矩。
    //
    if (!sensor_state.valid)
    {
      output.valid = false;
      return;
    }

    // ===================== 组装 q / dq =====================
    //
    // 1. state.position / velocity 的单位是 deg / deg/s
    // 2. Pinocchio 这里使用 rad / rad/s
    // 3. 当前模型自由度为 7：
    //    q = [joint1, joint2, joint3, joint4, joint5, joint6, joint7]
    //
    constexpr double DEG2RAD = M_PI / 180.0;

    Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);

    q(0) = state.position[IDX_J1] * DEG2RAD;
    q(1) = state.position[IDX_J2] * DEG2RAD;
    q(2) = state.position[IDX_J3] * DEG2RAD;
    q(3) = state.position[IDX_J4] * DEG2RAD;
    q(4) = state.position[IDX_J5] * DEG2RAD;
    q(5) = state.position[IDX_J6] * DEG2RAD;
    q(6) = state.position[IDX_J7] * DEG2RAD;

    v(0) = state.velocity[IDX_J1] * DEG2RAD;
    v(1) = state.velocity[IDX_J2] * DEG2RAD;
    v(2) = state.velocity[IDX_J3] * DEG2RAD;
    v(3) = state.velocity[IDX_J4] * DEG2RAD;
    v(4) = state.velocity[IDX_J5] * DEG2RAD;
    v(5) = state.velocity[IDX_J6] * DEG2RAD;
    v(6) = state.velocity[IDX_J7] * DEG2RAD;

    // ===================== 读取并去偏两个六维力传感器 =====================
    //
    // 左臂：
    // SENSOR_LEFT_HAND = 2
    // SENSOR_LEFT_ARM  = 3
    //
    // 约定：
    // 1. sensor_state.sensor[] 是实时值
    // 2. sensor_state.bias[]   是进入零力模式时记录的零偏
    // 3. 这里直接做 raw - bias
    //
    const auto &raw_hand = sensor_state.sensor[SENSOR_LEFT_HAND];
    const auto &bias_hand = sensor_state.bias[SENSOR_LEFT_HAND];

    const auto &raw_arm = sensor_state.sensor[SENSOR_LEFT_ARM];
    const auto &bias_arm = sensor_state.bias[SENSOR_LEFT_ARM];

    double fx_arm = raw_arm.fx - bias_arm.fx;
    double fy_arm = raw_arm.fy - bias_arm.fy;
    double fz_arm = raw_arm.fz - bias_arm.fz;
    double mx_arm = raw_arm.mx - bias_arm.mx;
    double my_arm = raw_arm.my - bias_arm.my;
    double mz_arm = raw_arm.mz - bias_arm.mz;

    double fx_hand = raw_hand.fx - bias_hand.fx;
    double fy_hand = raw_hand.fy - bias_hand.fy;
    double fz_hand = raw_hand.fz - bias_hand.fz;
    double mx_hand = raw_hand.mx - bias_hand.mx;
    double my_hand = raw_hand.my - bias_hand.my;
    double mz_hand = raw_hand.mz - bias_hand.mz;

    // ===================== 传感器方向符号修正 =====================
    //
    // 顺序：
    // [Fx, Fy, Fz, Mx, My, Mz]
    //
    static const std::array<double, 6> arm_sign = {
        1.0, 1.0, 1.0,
        1.0, 1.0, 1.0};

    static const std::array<double, 6> hand_sign = {
        1.0, 1.0, 1.0,
        1.0, 1.0, 1.0};

    fx_arm *= arm_sign[0];
    fy_arm *= arm_sign[1];
    fz_arm *= arm_sign[2];
    mx_arm *= arm_sign[3];
    my_arm *= arm_sign[4];
    mz_arm *= arm_sign[5];

    fx_hand *= hand_sign[0];
    fy_hand *= hand_sign[1];
    fz_hand *= hand_sign[2];
    mx_hand *= hand_sign[3];
    my_hand *= hand_sign[4];
    mz_hand *= hand_sign[5];

    // ===================== 死区 =====================
    //
    // 力 / 力矩很小时直接置 0，防止噪声抖动。
    //
    const double force_deadzone = 1.0;   // N
    const double torque_deadzone = 0.05; // Nm

    auto apply_deadzone = [](double x, double dz) -> double
    {
      return (std::abs(x) < dz) ? 0.0 : x;
    };

    fx_arm = apply_deadzone(fx_arm, force_deadzone);
    fy_arm = apply_deadzone(fy_arm, force_deadzone);
    fz_arm = apply_deadzone(fz_arm, force_deadzone);
    mx_arm = apply_deadzone(mx_arm, torque_deadzone);
    my_arm = apply_deadzone(my_arm, torque_deadzone);
    mz_arm = apply_deadzone(mz_arm, torque_deadzone);

    fx_hand = apply_deadzone(fx_hand, force_deadzone);
    fy_hand = apply_deadzone(fy_hand, force_deadzone);
    fz_hand = apply_deadzone(fz_hand, force_deadzone);
    mx_hand = apply_deadzone(mx_hand, torque_deadzone);
    my_hand = apply_deadzone(my_hand, torque_deadzone);
    mz_hand = apply_deadzone(mz_hand, torque_deadzone);

    // ===================== 限幅 =====================
    //
    // 防止异常值把控制打飞。
    //
    const double force_limit = 80.0; // N
    const double moment_limit = 8.0; // Nm

    auto clamp_abs = [](double x, double lim) -> double
    {
      if (x > lim)
        return lim;
      if (x < -lim)
        return -lim;
      return x;
    };

    fx_arm = clamp_abs(fx_arm, force_limit);
    fy_arm = clamp_abs(fy_arm, force_limit);
    fz_arm = clamp_abs(fz_arm, force_limit);
    mx_arm = clamp_abs(mx_arm, moment_limit);
    my_arm = clamp_abs(my_arm, moment_limit);
    mz_arm = clamp_abs(mz_arm, moment_limit);

    fx_hand = clamp_abs(fx_hand, force_limit);
    fy_hand = clamp_abs(fy_hand, force_limit);
    fz_hand = clamp_abs(fz_hand, force_limit);
    mx_hand = clamp_abs(mx_hand, moment_limit);
    my_hand = clamp_abs(my_hand, moment_limit);
    mz_hand = clamp_abs(mz_hand, moment_limit);

    // ===================== 计算重力项 =====================
    //
    // g(q) 的维度是 7，对应 joint1~7。
    // Eigen::VectorXd g 本质上就是一个：动态大小的一维向量（列向量），里面存的是每个关节的重力力矩
    // computeGeneralizedGravity:
    //     model → 机器人结构 + 质量
    //     data  → 中间缓存
    //     q     → 当前关节角（rad）
    //     g(q)  → 每个关节的重力力矩
    Eigen::VectorXd g = pinocchio::computeGeneralizedGravity(model, *data, q);

    // ===================== 计算两个传感器 frame 的 Jacobian =====================
    //
    // 把“末端（传感器）空间的力/速度”映射到“关节空间”
    //
    // 创建两个大小为 6 × nv 的矩阵，并全部初始化为 0
    // 为什么是 6 行？
    // [ vx ]   ← 线速度 x
    // [ vy ]
    // [ vz ]
    // [ wx ]   ← 角速度 x
    // [ wy ]
    // [ wz ]
    // 为什么是 nv 列？
    // nv 是模型的自由度数量，这里是 7，对应 joint1~7。
    Eigen::MatrixXd J_arm = Eigen::MatrixXd::Zero(6, model.nv);
    Eigen::MatrixXd J_hand = Eigen::MatrixXd::Zero(6, model.nv);

    pinocchio::computeFrameJacobian(model, *data, q, frame_id_arm, pinocchio::LOCAL, J_arm);
    pinocchio::computeFrameJacobian(model, *data, q, frame_id_hand, pinocchio::LOCAL, J_hand);

    // ===================== 构造 Pinocchio 空间力 =====================
    //
    // Pinocchio 空间力的顺序是：
    // [angular; linear]
    //
    // 直接用 Force 对象，最后 toVector()，避免顺序写错。

    // 创建一个 空间力（Force）对象 f_arm，并初始化为零
    pinocchio::Force f_arm = pinocchio::Force::Zero();
    // 把 fx_arm、fy_arm、fz_arm 填入 f_arm 的线性部分，把 mx_arm、my_arm、mz_arm 填入 f_arm 的角部分
    f_arm.linear() << fx_arm, fy_arm, fz_arm;
    f_arm.angular() << mx_arm, my_arm, mz_arm;

    pinocchio::Force f_hand = pinocchio::Force::Zero();
    f_hand.linear() << fx_hand, fy_hand, fz_hand;
    f_hand.angular() << mx_hand, my_hand, mz_hand;

    // ===================== 外力映射到关节扭矩 =====================
    //
    // 数学意义:τ=J^T*F
    // tau_ext = J_arm^T * F_arm + J_hand^T * F_hand
    //
    Eigen::VectorXd tau_arm = J_arm.transpose() * f_arm.toVector();
    Eigen::VectorXd tau_hand = J_hand.transpose() * f_hand.toVector();

    Eigen::VectorXd tau_ext = tau_arm + tau_hand;

    // ===================== 关节阻尼项 =====================
    //
    // 防止零力模式下出现漂、轻微自激或关节发飘。
    //
    // 创建一个长度为 nv 的向量：
    Eigen::VectorXd damping = Eigen::VectorXd::Zero(model.nv);

    // 给每个关节设置阻尼系数：
    // 背部零力不是“完全无阻尼”，而是“有一定粘滞感，但不会死硬”
    // 肩部和肘部继续保留较强跟随性
    damping(0) = 0.0;
    damping(1) = 0.4;
    damping(2) = 0.3;
    damping(3) = 0.0;
    damping(4) = 0.0;
    damping(5) = 0.2;
    damping(6) = 0.3;

    // 计算阻尼力矩：tau_damping = -damping * v，反速度方向
    // damping.asDiagonal()
    // 把向量变成对角矩阵，再乘 v，就相当于每个元素分别乘以对应的速度
    Eigen::VectorXd tau_damping = -(damping.asDiagonal() * v);

    // ===================== 最终关节扭矩 =====================
    //
    // 当前采用：
    // tau = g(q) + tau_ext + tau_damping
    //
    // 如果方向整体反了，
    // 优先检查：
    // 1. 传感器符号 arm_sign / hand_sign
    // 2. 电机 torque raw 映射方向
    //
    Eigen::VectorXd tau = g + tau_ext + tau_damping;

    // ===================== 最终扭矩限幅 =====================
    //
    // 输出给执行层前，限制每个关节的目标扭矩。
    //
    const double joint_torque_limit_j1 = 30.0;
    const double joint_torque_limit_j2 = 20.0;
    const double joint_torque_limit_j3 = 16.0;
    const double joint_torque_limit_j4 = 12.0;
    const double joint_torque_limit_j5 = 12.0;
    const double joint_torque_limit_j6 = 12.0;
    const double joint_torque_limit_j7 = 8.0;

    tau(0) = clamp_abs(tau(0), joint_torque_limit_j1);
    tau(1) = clamp_abs(tau(1), joint_torque_limit_j2);
    tau(2) = clamp_abs(tau(2), joint_torque_limit_j3);
    tau(3) = clamp_abs(tau(3), joint_torque_limit_j4);
    tau(4) = clamp_abs(tau(4), joint_torque_limit_j5);
    tau(5) = clamp_abs(tau(5), joint_torque_limit_j6);
    tau(6) = clamp_abs(tau(6), joint_torque_limit_j7);

    // ===================== 输出到控制层 =====================
    //
    // 这里对 joint1~7 对应的控制层索引写扭矩。
    // 其他关节默认保持 0，不参与零力控制。
    //
    output.use_torque[IDX_J1] = true;
    output.use_torque[IDX_J2] = true;
    output.use_torque[IDX_J3] = true;
    output.use_torque[IDX_J4] = true;
    output.use_torque[IDX_J5] = true;
    output.use_torque[IDX_J6] = true;
    output.use_torque[IDX_J7] = true;

    output.target_torque[IDX_J1] = tau(0);
    output.target_torque[IDX_J2] = tau(1);
    output.target_torque[IDX_J3] = tau(2);
    output.target_torque[IDX_J4] = tau(3);
    output.target_torque[IDX_J5] = tau(4);
    output.target_torque[IDX_J6] = tau(5);
    output.target_torque[IDX_J7] = tau(6);

    // 末端其余关节固定，避免切模式后还能晃动
    output.use_position[17] = true;
    output.target_position[17] = 0.0;
    output.target_velocity[17] = pulsePerMsToJointSpeedDegPerSecMagnitude(18, 1500.0);
    output.target_acc[17] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(18, 5.0);
    output.target_dec[17] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(18, 0.25);
    output.target_smooth_time[17] = 25;

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

    // ===================== 输出有效 =====================
    output.valid = true;
  }

  // ===================== 右臂零力模式 =====================
  void runRightZeroForceMode(const MotorStateCache &state, const SensorStateCache &sensor_state, ControlOutputCache &output)
  {
    // ===================== 输出初始化 =====================
    //
    // 每次进入先清空 output，
    // 然后只对右臂需要控制的关节写入输出。
    //
    output = ControlOutputCache{};
    output.valid = false;

    // ===================== 右臂 joint11~joint20 的控制层索引 =====================
    //
    // 当前默认约定：
    // 右臂控制层索引 0~9
    //
    // joint11 ~ joint17 : 可动关节
    // joint18 ~ joint20 : 固定末端
    //
    constexpr std::size_t IDX_J11 = 0;
    constexpr std::size_t IDX_J12 = 1;
    constexpr std::size_t IDX_J13 = 2;
    constexpr std::size_t IDX_J14 = 3;
    constexpr std::size_t IDX_J15 = 4;
    constexpr std::size_t IDX_J16 = 5;
    constexpr std::size_t IDX_J17 = 6;
    constexpr std::size_t IDX_J18 = 7;
    constexpr std::size_t IDX_J19 = 8;
    constexpr std::size_t IDX_J20 = 9;

    // ===================== Pinocchio 懒加载上下文 =====================
    //
    // 第一次进入时加载右臂 URDF、查 frame id；
    // 后续周期只做实时计算。
    //
    static bool pin_initialized = false;
    static bool pin_init_failed = false;
    static pinocchio::Model model;
    static std::unique_ptr<pinocchio::Data> data;
    static pinocchio::FrameIndex frame_id_arm = 0;
    static pinocchio::FrameIndex frame_id_hand = 0;

    if (!pin_initialized && !pin_init_failed)
    {
      try
      {
        // ===================== URDF 路径 =====================
        //
        // upperlimb_robot/urdf/right_arm_experiment.urdf
        //
        std::string urdf_path =
            ament_index_cpp::get_package_share_directory("upperlimb_robot") +
            "/urdf/right_arm_experiment.urdf";

        // ===================== 加载 URDF =====================
        //
        // 当前右臂模型约定：
        // joint11~joint17 为 revolute
        // joint18~joint20 为 fixed
        //
        pinocchio::urdf::buildModel(urdf_path, model);
        data = std::make_unique<pinocchio::Data>(model);

        // ===================== 获取两个六维力传感器 frame =====================
        //
        // ft_rightsensor_frame     -> 大臂绑缚件
        // ft_rightsensor_frame_2   -> 手部绑缚件
        //
        frame_id_arm = model.getFrameId("ft_rightsensor_frame");
        frame_id_hand = model.getFrameId("ft_rightsensor_frame_2");

        if (frame_id_arm >= model.frames.size())
        {
          std::cerr << "[runRightZeroForceMode] 没找到 frame: ft_rightsensor_frame" << std::endl;
          pin_init_failed = true;
        }
        else if (frame_id_hand >= model.frames.size())
        {
          std::cerr << "[runRightZeroForceMode] 没找到 frame: ft_rightsensor_frame_2" << std::endl;
          pin_init_failed = true;
        }
        else if (model.nq != 7 || model.nv != 7)
        {
          std::cerr
              << "[runRightZeroForceMode] 当前 Pinocchio 模型自由度不是 7，"
              << "请确认 right_arm_experiment.urdf 中 joint11~joint17 为 revolute，"
              << "joint18~joint20 为 fixed。"
              << " model.nq=" << model.nq
              << " model.nv=" << model.nv
              << std::endl;
          pin_init_failed = true;
        }
        else
        {
          pin_initialized = true;
          std::cout
              << "[runRightZeroForceMode] Pinocchio 初始化成功。"
              << " urdf_path=" << urdf_path
              << " nq=" << model.nq
              << " nv=" << model.nv
              << std::endl;
        }
      }
      catch (const std::exception &e)
      {
        std::cerr << "[runRightZeroForceMode] Pinocchio 初始化异常: " << e.what() << std::endl;
        pin_init_failed = true;
      }
    }

    if (!pin_initialized || pin_init_failed || !data)
    {
      output.valid = false;
      return;
    }

    // ===================== 传感器有效性检查 =====================
    //
    // 当前传感器无效时，不输出扭矩。
    //
    if (!sensor_state.valid)
    {
      output.valid = false;
      return;
    }

    // ===================== 组装 q / v =====================
    //
    // 注意：
    // 1. state.position / velocity 的单位是 deg / deg/s
    // 2. Pinocchio 这里使用 rad / rad/s
    // 3. 当前右臂模型自由度为 7：
    //    q = [joint11, joint12, joint13, joint14, joint15, joint16, joint17]
    //
    constexpr double DEG2RAD = M_PI / 180.0;

    Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);

    q(0) = state.position[IDX_J11] * DEG2RAD;
    q(1) = state.position[IDX_J12] * DEG2RAD;
    q(2) = state.position[IDX_J13] * DEG2RAD;
    q(3) = state.position[IDX_J14] * DEG2RAD;
    q(4) = state.position[IDX_J15] * DEG2RAD;
    q(5) = state.position[IDX_J16] * DEG2RAD;
    q(6) = state.position[IDX_J17] * DEG2RAD;

    v(0) = state.velocity[IDX_J11] * DEG2RAD;
    v(1) = state.velocity[IDX_J12] * DEG2RAD;
    v(2) = state.velocity[IDX_J13] * DEG2RAD;
    v(3) = state.velocity[IDX_J14] * DEG2RAD;
    v(4) = state.velocity[IDX_J15] * DEG2RAD;
    v(5) = state.velocity[IDX_J16] * DEG2RAD;
    v(6) = state.velocity[IDX_J17] * DEG2RAD;

    // ===================== 读取并去偏两个六维力传感器 =====================
    //
    // 右臂：
    // SENSOR_RIGHT_HAND = 0
    // SENSOR_RIGHT_ARM  = 1
    //
    const auto &raw_hand = sensor_state.sensor[SENSOR_RIGHT_HAND];
    const auto &bias_hand = sensor_state.bias[SENSOR_RIGHT_HAND];

    const auto &raw_arm = sensor_state.sensor[SENSOR_RIGHT_ARM];
    const auto &bias_arm = sensor_state.bias[SENSOR_RIGHT_ARM];

    double fx_arm = raw_arm.fx - bias_arm.fx;
    double fy_arm = raw_arm.fy - bias_arm.fy;
    double fz_arm = raw_arm.fz - bias_arm.fz;
    double mx_arm = raw_arm.mx - bias_arm.mx;
    double my_arm = raw_arm.my - bias_arm.my;
    double mz_arm = raw_arm.mz - bias_arm.mz;

    double fx_hand = raw_hand.fx - bias_hand.fx;
    double fy_hand = raw_hand.fy - bias_hand.fy;
    double fz_hand = raw_hand.fz - bias_hand.fz;
    double mx_hand = raw_hand.mx - bias_hand.mx;
    double my_hand = raw_hand.my - bias_hand.my;
    double mz_hand = raw_hand.mz - bias_hand.mz;

    // ===================== 传感器方向符号修正 =====================
    //
    // 顺序：
    // [Fx, Fy, Fz, Mx, My, Mz]
    //
    // 这里先默认全 1。
    // 如果右臂实机方向出现“人往前推，机械臂反而更顶住”的情况，
    // 优先先改这里。
    //
    static const std::array<double, 6> arm_sign = {
        1.0, 1.0, 1.0,
        1.0, 1.0, 1.0};

    static const std::array<double, 6> hand_sign = {
        1.0, 1.0, 1.0,
        1.0, 1.0, 1.0};

    fx_arm *= arm_sign[0];
    fy_arm *= arm_sign[1];
    fz_arm *= arm_sign[2];
    mx_arm *= arm_sign[3];
    my_arm *= arm_sign[4];
    mz_arm *= arm_sign[5];

    fx_hand *= hand_sign[0];
    fy_hand *= hand_sign[1];
    fz_hand *= hand_sign[2];
    mx_hand *= hand_sign[3];
    my_hand *= hand_sign[4];
    mz_hand *= hand_sign[5];

    // ===================== 死区 =====================
    //
    // 力 / 力矩很小时直接置 0，防止噪声抖动。
    //
    const double force_deadzone = 1.0;   // N
    const double torque_deadzone = 0.05; // Nm

    auto apply_deadzone = [](double x, double dz) -> double
    {
      return (std::abs(x) < dz) ? 0.0 : x;
    };

    fx_arm = apply_deadzone(fx_arm, force_deadzone);
    fy_arm = apply_deadzone(fy_arm, force_deadzone);
    fz_arm = apply_deadzone(fz_arm, force_deadzone);
    mx_arm = apply_deadzone(mx_arm, torque_deadzone);
    my_arm = apply_deadzone(my_arm, torque_deadzone);
    mz_arm = apply_deadzone(mz_arm, torque_deadzone);

    fx_hand = apply_deadzone(fx_hand, force_deadzone);
    fy_hand = apply_deadzone(fy_hand, force_deadzone);
    fz_hand = apply_deadzone(fz_hand, force_deadzone);
    mx_hand = apply_deadzone(mx_hand, torque_deadzone);
    my_hand = apply_deadzone(my_hand, torque_deadzone);
    mz_hand = apply_deadzone(mz_hand, torque_deadzone);

    // ===================== 限幅 =====================
    //
    // 防止异常值把控制打飞。
    //
    const double force_limit = 80.0; // N
    const double moment_limit = 8.0; // Nm

    auto clamp_abs = [](double x, double lim) -> double
    {
      if (x > lim)
        return lim;
      if (x < -lim)
        return -lim;
      return x;
    };

    fx_arm = clamp_abs(fx_arm, force_limit);
    fy_arm = clamp_abs(fy_arm, force_limit);
    fz_arm = clamp_abs(fz_arm, force_limit);
    mx_arm = clamp_abs(mx_arm, moment_limit);
    my_arm = clamp_abs(my_arm, moment_limit);
    mz_arm = clamp_abs(mz_arm, moment_limit);

    fx_hand = clamp_abs(fx_hand, force_limit);
    fy_hand = clamp_abs(fy_hand, force_limit);
    fz_hand = clamp_abs(fz_hand, force_limit);
    mx_hand = clamp_abs(mx_hand, moment_limit);
    my_hand = clamp_abs(my_hand, moment_limit);
    mz_hand = clamp_abs(mz_hand, moment_limit);

    // ===================== 计算重力项 =====================
    //
    // g(q) 对应 joint11~joint17 的重力补偿力矩
    //
    Eigen::VectorXd g = pinocchio::computeGeneralizedGravity(model, *data, q);

    // ===================== 计算两个传感器 frame 的 Jacobian =====================
    //
    Eigen::MatrixXd J_arm = Eigen::MatrixXd::Zero(6, model.nv);
    Eigen::MatrixXd J_hand = Eigen::MatrixXd::Zero(6, model.nv);

    pinocchio::computeFrameJacobian(model, *data, q, frame_id_arm, pinocchio::LOCAL, J_arm);
    pinocchio::computeFrameJacobian(model, *data, q, frame_id_hand, pinocchio::LOCAL, J_hand);

    // ===================== 构造 Pinocchio 空间力 =====================
    //
    // Force 顺序是 [angular; linear]
    //
    pinocchio::Force f_arm = pinocchio::Force::Zero();
    f_arm.linear() << fx_arm, fy_arm, fz_arm;
    f_arm.angular() << mx_arm, my_arm, mz_arm;

    pinocchio::Force f_hand = pinocchio::Force::Zero();
    f_hand.linear() << fx_hand, fy_hand, fz_hand;
    f_hand.angular() << mx_hand, my_hand, mz_hand;

    // ===================== 外力映射到关节扭矩 =====================
    //
    // tau_ext = J_arm^T * F_arm + J_hand^T * F_hand
    //
    Eigen::VectorXd tau_arm = J_arm.transpose() * f_arm.toVector();
    Eigen::VectorXd tau_hand = J_hand.transpose() * f_hand.toVector();
    Eigen::VectorXd tau_ext = tau_arm + tau_hand;

    // ===================== 关节阻尼项 =====================
    //
    // 防止零力模式下出现漂、轻微自激或关节发飘。
    //
    Eigen::VectorXd damping = Eigen::VectorXd::Zero(model.nv);

    // 先直接沿用左臂这套阻尼配置
    damping(0) = 0.0;
    damping(1) = 0.4;
    damping(2) = 0.3;
    damping(3) = 0.0;
    damping(4) = 0.0;
    damping(5) = 0.2;
    damping(6) = 0.3;

    Eigen::VectorXd tau_damping = -(damping.asDiagonal() * v);

    // ===================== 最终关节扭矩 =====================
    //
    // tau = g(q) + tau_ext + tau_damping
    //
    Eigen::VectorXd tau = g + tau_ext + tau_damping;

    // ===================== 最终扭矩限幅 =====================
    //
    // 这里按右臂 joint11~joint17 依次限幅。
    //
    const double joint_torque_limit_j11 = 30.0;
    const double joint_torque_limit_j12 = 20.0;
    const double joint_torque_limit_j13 = 16.0;
    const double joint_torque_limit_j14 = 12.0;
    const double joint_torque_limit_j15 = 12.0;
    const double joint_torque_limit_j16 = 12.0;
    const double joint_torque_limit_j17 = 8.0;

    tau(0) = clamp_abs(-tau(0), joint_torque_limit_j11);
    tau(1) = clamp_abs(-tau(1), joint_torque_limit_j12);
    tau(2) = clamp_abs(-tau(2), joint_torque_limit_j13);
    tau(3) = clamp_abs(-tau(3), joint_torque_limit_j14);
    tau(4) = clamp_abs(-tau(4), joint_torque_limit_j15);
    tau(5) = clamp_abs(-tau(5), joint_torque_limit_j16);
    tau(6) = clamp_abs(-tau(6), joint_torque_limit_j17);

    // ===================== 输出到控制层 =====================
    //
    // joint11~joint17 走扭矩控制
    //
    output.use_torque[IDX_J11] = true;
    output.use_torque[IDX_J12] = true;
    output.use_torque[IDX_J13] = true;
    output.use_torque[IDX_J14] = true;
    output.use_torque[IDX_J15] = true;
    output.use_torque[IDX_J16] = true;
    output.use_torque[IDX_J17] = true;

    output.target_torque[IDX_J11] = tau(0);
    output.target_torque[IDX_J12] = tau(1);
    output.target_torque[IDX_J13] = tau(2);
    output.target_torque[IDX_J14] = tau(3);
    output.target_torque[IDX_J15] = tau(4);
    output.target_torque[IDX_J16] = tau(5);
    output.target_torque[IDX_J17] = tau(6);

    // ===================== 保存右臂零力模式扭矩到 CSV =====================
    static std::ofstream csv_file;
    static bool csv_initialized = false;
    static int log_count = 0;

    if (!csv_initialized)
    {
      csv_file.open("right_zero_force_tau_log.csv", std::ios::out | std::ios::app);

      if (csv_file.is_open())
      {
        csv_file << "wall_time,t,tau1,tau2,tau3,tau4,tau5,tau6,tau7\n";
        csv_initialized = true;
      }
      else
      {
        std::cerr << "[runRightZeroForceMode] 无法打开 right_zero_force_tau_log.csv" << std::endl;
      }
    }

    // 降采样记录，避免每个控制周期都写文件
    log_count++;

    if (csv_initialized && (log_count % 4 == 0))
    {
      static auto t0 = std::chrono::steady_clock::now();

      auto now_sys = std::chrono::system_clock::now();
      auto now_steady = std::chrono::steady_clock::now();

      double t =
          std::chrono::duration<double>(now_steady - t0).count();

      auto now_time_t = std::chrono::system_clock::to_time_t(now_sys);
      auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now_sys.time_since_epoch()) %
                1000;

      char time_str[64];
      std::strftime(
          time_str,
          sizeof(time_str),
          "%Y-%m-%d %H:%M:%S",
          std::localtime(&now_time_t));

      csv_file
          << time_str << "."
          << std::setw(3) << std::setfill('0') << ms.count() << ","
          << t << ","
          << tau(0) << ","
          << tau(1) << ","
          << tau(2) << ","
          << tau(3) << ","
          << tau(4) << ","
          << tau(5) << ","
          << tau(6) << "\n";
    }

    // ===================== 固定末端 joint18~joint20 =====================
    //
    // 避免切模式后末端几个轴乱晃。
    //
    output.use_position[IDX_J18] = true;
    output.target_position[IDX_J18] = 0.0;
    output.target_velocity[IDX_J18] = pulsePerMsToJointSpeedDegPerSecMagnitude(8, 1500.0);
    output.target_acc[IDX_J18] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(8, 5.0);
    output.target_dec[IDX_J18] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(8, 0.25);
    output.target_smooth_time[IDX_J18] = 25;

    output.use_position[IDX_J19] = true;
    output.target_position[IDX_J19] = 0.0;
    output.target_velocity[IDX_J19] = pulsePerMsToJointSpeedDegPerSecMagnitude(9, 1500.0);
    output.target_acc[IDX_J19] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(9, 5.0);
    output.target_dec[IDX_J19] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(9, 0.25);
    output.target_smooth_time[IDX_J19] = 25;

    output.use_position[IDX_J20] = true;
    output.target_position[IDX_J20] = 0.0;
    output.target_velocity[IDX_J20] = pulsePerMsToJointSpeedDegPerSecMagnitude(10, 1500.0);
    output.target_acc[IDX_J20] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(10, 5.0);
    output.target_dec[IDX_J20] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(10, 0.25);
    output.target_smooth_time[IDX_J20] = 25;

    // ===================== 输出有效 =====================
    output.valid = true;
  }

  // ===================== 双侧零力模式 =====================
  void runBilateralZeroForceMode(const MotorStateCache &state, const SensorStateCache &sensor_state, ControlOutputCache &output)
  {
    // ===================== 输出初始化 =====================
    output = ControlOutputCache{};
    output.valid = false;

    // ===================== 控制层索引约定 =====================
    //
    // 右臂：
    // joint11~joint20 -> 控制层索引 0~9
    //
    // 左臂：
    // joint1~joint10  -> 控制层索引 10~19
    //
    // 其中：
    // 右臂可动：joint11~joint17 -> 0~6
    // 右臂固定：joint18~joint20 -> 7~9
    //
    // 左臂可动：joint1~joint7   -> 10~16
    // 左臂固定：joint8~joint10  -> 17~19
    //
    constexpr std::size_t IDX_R_J11 = 0;
    constexpr std::size_t IDX_R_J12 = 1;
    constexpr std::size_t IDX_R_J13 = 2;
    constexpr std::size_t IDX_R_J14 = 3;
    constexpr std::size_t IDX_R_J15 = 4;
    constexpr std::size_t IDX_R_J16 = 5;
    constexpr std::size_t IDX_R_J17 = 6;
    constexpr std::size_t IDX_R_J18 = 7;
    constexpr std::size_t IDX_R_J19 = 8;
    constexpr std::size_t IDX_R_J20 = 9;

    constexpr std::size_t IDX_L_J1 = 10;
    constexpr std::size_t IDX_L_J2 = 11;
    constexpr std::size_t IDX_L_J3 = 12;
    constexpr std::size_t IDX_L_J4 = 13;
    constexpr std::size_t IDX_L_J5 = 14;
    constexpr std::size_t IDX_L_J6 = 15;
    constexpr std::size_t IDX_L_J7 = 16;
    constexpr std::size_t IDX_L_J8 = 17;
    constexpr std::size_t IDX_L_J9 = 18;
    constexpr std::size_t IDX_L_J10 = 19;

    // ===================== Pinocchio 懒加载：左臂 =====================
    static bool left_pin_initialized = false;
    static bool left_pin_init_failed = false;
    static pinocchio::Model left_model;
    static std::unique_ptr<pinocchio::Data> left_data;
    static pinocchio::FrameIndex left_frame_id_arm = 0;
    static pinocchio::FrameIndex left_frame_id_hand = 0;

    // ===================== Pinocchio 懒加载：右臂 =====================
    static bool right_pin_initialized = false;
    static bool right_pin_init_failed = false;
    static pinocchio::Model right_model;
    static std::unique_ptr<pinocchio::Data> right_data;
    static pinocchio::FrameIndex right_frame_id_arm = 0;
    static pinocchio::FrameIndex right_frame_id_hand = 0;

    // ===================== 初始化左臂模型 =====================
    if (!left_pin_initialized && !left_pin_init_failed)
    {
      try
      {
        std::string urdf_path =
            ament_index_cpp::get_package_share_directory("upperlimb_robot") +
            "/urdf/left_arm_experiment.urdf";

        pinocchio::urdf::buildModel(urdf_path, left_model);
        left_data = std::make_unique<pinocchio::Data>(left_model);

        left_frame_id_arm = left_model.getFrameId("ft_sensor_frame");
        left_frame_id_hand = left_model.getFrameId("ft_sensor_frame_2");

        if (left_frame_id_arm >= left_model.frames.size())
        {
          std::cerr << "[runBilateralZeroForceMode] 左臂没找到 frame: ft_sensor_frame" << std::endl;
          left_pin_init_failed = true;
        }
        else if (left_frame_id_hand >= left_model.frames.size())
        {
          std::cerr << "[runBilateralZeroForceMode] 左臂没找到 frame: ft_sensor_frame_2" << std::endl;
          left_pin_init_failed = true;
        }
        else if (left_model.nq != 7 || left_model.nv != 7)
        {
          std::cerr
              << "[runBilateralZeroForceMode] 左臂模型自由度不是 7，"
              << "请确认 left_arm_experiment.urdf 中 joint1~joint7 为可动关节。"
              << " nq=" << left_model.nq
              << " nv=" << left_model.nv
              << std::endl;
          left_pin_init_failed = true;
        }
        else
        {
          left_pin_initialized = true;
          std::cout
              << "[runBilateralZeroForceMode] 左臂 Pinocchio 初始化成功。"
              << " urdf_path=" << urdf_path
              << " nq=" << left_model.nq
              << " nv=" << left_model.nv
              << std::endl;
        }
      }
      catch (const std::exception &e)
      {
        std::cerr << "[runBilateralZeroForceMode] 左臂 Pinocchio 初始化异常: " << e.what() << std::endl;
        left_pin_init_failed = true;
      }
    }

    // ===================== 初始化右臂模型 =====================
    if (!right_pin_initialized && !right_pin_init_failed)
    {
      try
      {
        std::string urdf_path =
            ament_index_cpp::get_package_share_directory("upperlimb_robot") +
            "/urdf/right_arm_experiment.urdf";

        pinocchio::urdf::buildModel(urdf_path, right_model);
        right_data = std::make_unique<pinocchio::Data>(right_model);

        right_frame_id_arm = right_model.getFrameId("ft_rightsensor_frame");
        right_frame_id_hand = right_model.getFrameId("ft_rightsensor_frame_2");

        if (right_frame_id_arm >= right_model.frames.size())
        {
          std::cerr << "[runBilateralZeroForceMode] 右臂没找到 frame: ft_rightsensor_frame" << std::endl;
          right_pin_init_failed = true;
        }
        else if (right_frame_id_hand >= right_model.frames.size())
        {
          std::cerr << "[runBilateralZeroForceMode] 右臂没找到 frame: ft_rightsensor_frame_2" << std::endl;
          right_pin_init_failed = true;
        }
        else if (right_model.nq != 7 || right_model.nv != 7)
        {
          std::cerr
              << "[runBilateralZeroForceMode] 右臂模型自由度不是 7，"
              << "请确认 right_arm_experiment.urdf 中 joint11~joint17 为可动关节。"
              << " nq=" << right_model.nq
              << " nv=" << right_model.nv
              << std::endl;
          right_pin_init_failed = true;
        }
        else
        {
          right_pin_initialized = true;
          std::cout
              << "[runBilateralZeroForceMode] 右臂 Pinocchio 初始化成功。"
              << " urdf_path=" << urdf_path
              << " nq=" << right_model.nq
              << " nv=" << right_model.nv
              << std::endl;
        }
      }
      catch (const std::exception &e)
      {
        std::cerr << "[runBilateralZeroForceMode] 右臂 Pinocchio 初始化异常: " << e.what() << std::endl;
        right_pin_init_failed = true;
      }
    }

    if (!left_pin_initialized || left_pin_init_failed || !left_data)
    {
      output.valid = false;
      return;
    }

    if (!right_pin_initialized || right_pin_init_failed || !right_data)
    {
      output.valid = false;
      return;
    }

    // ===================== 传感器有效性检查 =====================
    if (!sensor_state.valid)
    {
      output.valid = false;
      return;
    }

    constexpr double DEG2RAD = M_PI / 180.0;

    // ============================================================
    // ===================== 左臂：组装 q / v ======================
    // ============================================================
    Eigen::VectorXd q_left = Eigen::VectorXd::Zero(left_model.nq);
    Eigen::VectorXd v_left = Eigen::VectorXd::Zero(left_model.nv);

    q_left(0) = state.position[IDX_L_J1] * DEG2RAD;
    q_left(1) = state.position[IDX_L_J2] * DEG2RAD;
    q_left(2) = state.position[IDX_L_J3] * DEG2RAD;
    q_left(3) = state.position[IDX_L_J4] * DEG2RAD;
    q_left(4) = state.position[IDX_L_J5] * DEG2RAD;
    q_left(5) = state.position[IDX_L_J6] * DEG2RAD;
    q_left(6) = state.position[IDX_L_J7] * DEG2RAD;

    v_left(0) = state.velocity[IDX_L_J1] * DEG2RAD;
    v_left(1) = state.velocity[IDX_L_J2] * DEG2RAD;
    v_left(2) = state.velocity[IDX_L_J3] * DEG2RAD;
    v_left(3) = state.velocity[IDX_L_J4] * DEG2RAD;
    v_left(4) = state.velocity[IDX_L_J5] * DEG2RAD;
    v_left(5) = state.velocity[IDX_L_J6] * DEG2RAD;
    v_left(6) = state.velocity[IDX_L_J7] * DEG2RAD;

    // ============================================================
    // ===================== 右臂：组装 q / v ======================
    // ============================================================
    Eigen::VectorXd q_right = Eigen::VectorXd::Zero(right_model.nq);
    Eigen::VectorXd v_right = Eigen::VectorXd::Zero(right_model.nv);

    q_right(0) = state.position[IDX_R_J11] * DEG2RAD;
    q_right(1) = state.position[IDX_R_J12] * DEG2RAD;
    q_right(2) = state.position[IDX_R_J13] * DEG2RAD;
    q_right(3) = state.position[IDX_R_J14] * DEG2RAD;
    q_right(4) = state.position[IDX_R_J15] * DEG2RAD;
    q_right(5) = state.position[IDX_R_J16] * DEG2RAD;
    q_right(6) = state.position[IDX_R_J17] * DEG2RAD;

    v_right(0) = state.velocity[IDX_R_J11] * DEG2RAD;
    v_right(1) = state.velocity[IDX_R_J12] * DEG2RAD;
    v_right(2) = state.velocity[IDX_R_J13] * DEG2RAD;
    v_right(3) = state.velocity[IDX_R_J14] * DEG2RAD;
    v_right(4) = state.velocity[IDX_R_J15] * DEG2RAD;
    v_right(5) = state.velocity[IDX_R_J16] * DEG2RAD;
    v_right(6) = state.velocity[IDX_R_J17] * DEG2RAD;

    // ============================================================
    // ===================== 读取并去偏：左臂 ======================
    // ============================================================
    const auto &raw_left_hand = sensor_state.sensor[SENSOR_LEFT_HAND];
    const auto &bias_left_hand = sensor_state.bias[SENSOR_LEFT_HAND];

    const auto &raw_left_arm = sensor_state.sensor[SENSOR_LEFT_ARM];
    const auto &bias_left_arm = sensor_state.bias[SENSOR_LEFT_ARM];

    double fx_left_arm = raw_left_arm.fx - bias_left_arm.fx;
    double fy_left_arm = raw_left_arm.fy - bias_left_arm.fy;
    double fz_left_arm = raw_left_arm.fz - bias_left_arm.fz;
    double mx_left_arm = raw_left_arm.mx - bias_left_arm.mx;
    double my_left_arm = raw_left_arm.my - bias_left_arm.my;
    double mz_left_arm = raw_left_arm.mz - bias_left_arm.mz;

    double fx_left_hand = raw_left_hand.fx - bias_left_hand.fx;
    double fy_left_hand = raw_left_hand.fy - bias_left_hand.fy;
    double fz_left_hand = raw_left_hand.fz - bias_left_hand.fz;
    double mx_left_hand = raw_left_hand.mx - bias_left_hand.mx;
    double my_left_hand = raw_left_hand.my - bias_left_hand.my;
    double mz_left_hand = raw_left_hand.mz - bias_left_hand.mz;

    // ============================================================
    // ===================== 读取并去偏：右臂 ======================
    // ============================================================
    const auto &raw_right_hand = sensor_state.sensor[SENSOR_RIGHT_HAND];
    const auto &bias_right_hand = sensor_state.bias[SENSOR_RIGHT_HAND];

    const auto &raw_right_arm = sensor_state.sensor[SENSOR_RIGHT_ARM];
    const auto &bias_right_arm = sensor_state.bias[SENSOR_RIGHT_ARM];

    double fx_right_arm = raw_right_arm.fx - bias_right_arm.fx;
    double fy_right_arm = raw_right_arm.fy - bias_right_arm.fy;
    double fz_right_arm = raw_right_arm.fz - bias_right_arm.fz;
    double mx_right_arm = raw_right_arm.mx - bias_right_arm.mx;
    double my_right_arm = raw_right_arm.my - bias_right_arm.my;
    double mz_right_arm = raw_right_arm.mz - bias_right_arm.mz;

    double fx_right_hand = raw_right_hand.fx - bias_right_hand.fx;
    double fy_right_hand = raw_right_hand.fy - bias_right_hand.fy;
    double fz_right_hand = raw_right_hand.fz - bias_right_hand.fz;
    double mx_right_hand = raw_right_hand.mx - bias_right_hand.mx;
    double my_right_hand = raw_right_hand.my - bias_right_hand.my;
    double mz_right_hand = raw_right_hand.mz - bias_right_hand.mz;

    // ===================== 传感器方向符号修正 =====================
    //
    // 顺序：[Fx, Fy, Fz, Mx, My, Mz]
    //
    static const std::array<double, 6> left_arm_sign = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    static const std::array<double, 6> left_hand_sign = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

    static const std::array<double, 6> right_arm_sign = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    static const std::array<double, 6> right_hand_sign = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

    fx_left_arm *= left_arm_sign[0];
    fy_left_arm *= left_arm_sign[1];
    fz_left_arm *= left_arm_sign[2];
    mx_left_arm *= left_arm_sign[3];
    my_left_arm *= left_arm_sign[4];
    mz_left_arm *= left_arm_sign[5];

    fx_left_hand *= left_hand_sign[0];
    fy_left_hand *= left_hand_sign[1];
    fz_left_hand *= left_hand_sign[2];
    mx_left_hand *= left_hand_sign[3];
    my_left_hand *= left_hand_sign[4];
    mz_left_hand *= left_hand_sign[5];

    fx_right_arm *= right_arm_sign[0];
    fy_right_arm *= right_arm_sign[1];
    fz_right_arm *= right_arm_sign[2];
    mx_right_arm *= right_arm_sign[3];
    my_right_arm *= right_arm_sign[4];
    mz_right_arm *= right_arm_sign[5];

    fx_right_hand *= right_hand_sign[0];
    fy_right_hand *= right_hand_sign[1];
    fz_right_hand *= right_hand_sign[2];
    mx_right_hand *= right_hand_sign[3];
    my_right_hand *= right_hand_sign[4];
    mz_right_hand *= right_hand_sign[5];

    // ===================== 死区 =====================
    const double force_deadzone = 1.0;   // N
    const double torque_deadzone = 0.05; // Nm

    auto apply_deadzone = [](double x, double dz) -> double
    {
      return (std::abs(x) < dz) ? 0.0 : x;
    };

    // 左臂
    fx_left_arm = apply_deadzone(fx_left_arm, force_deadzone);
    fy_left_arm = apply_deadzone(fy_left_arm, force_deadzone);
    fz_left_arm = apply_deadzone(fz_left_arm, force_deadzone);
    mx_left_arm = apply_deadzone(mx_left_arm, torque_deadzone);
    my_left_arm = apply_deadzone(my_left_arm, torque_deadzone);
    mz_left_arm = apply_deadzone(mz_left_arm, torque_deadzone);

    fx_left_hand = apply_deadzone(fx_left_hand, force_deadzone);
    fy_left_hand = apply_deadzone(fy_left_hand, force_deadzone);
    fz_left_hand = apply_deadzone(fz_left_hand, force_deadzone);
    mx_left_hand = apply_deadzone(mx_left_hand, torque_deadzone);
    my_left_hand = apply_deadzone(my_left_hand, torque_deadzone);
    mz_left_hand = apply_deadzone(mz_left_hand, torque_deadzone);

    // 右臂
    fx_right_arm = apply_deadzone(fx_right_arm, force_deadzone);
    fy_right_arm = apply_deadzone(fy_right_arm, force_deadzone);
    fz_right_arm = apply_deadzone(fz_right_arm, force_deadzone);
    mx_right_arm = apply_deadzone(mx_right_arm, torque_deadzone);
    my_right_arm = apply_deadzone(my_right_arm, torque_deadzone);
    mz_right_arm = apply_deadzone(mz_right_arm, torque_deadzone);

    fx_right_hand = apply_deadzone(fx_right_hand, force_deadzone);
    fy_right_hand = apply_deadzone(fy_right_hand, force_deadzone);
    fz_right_hand = apply_deadzone(fz_right_hand, force_deadzone);
    mx_right_hand = apply_deadzone(mx_right_hand, torque_deadzone);
    my_right_hand = apply_deadzone(my_right_hand, torque_deadzone);
    mz_right_hand = apply_deadzone(mz_right_hand, torque_deadzone);

    // ===================== 限幅 =====================
    const double force_limit = 80.0; // N
    const double moment_limit = 8.0; // Nm

    auto clamp_abs = [](double x, double lim) -> double
    {
      if (x > lim)
        return lim;
      if (x < -lim)
        return -lim;
      return x;
    };

    // 左臂
    fx_left_arm = clamp_abs(fx_left_arm, force_limit);
    fy_left_arm = clamp_abs(fy_left_arm, force_limit);
    fz_left_arm = clamp_abs(fz_left_arm, force_limit);
    mx_left_arm = clamp_abs(mx_left_arm, moment_limit);
    my_left_arm = clamp_abs(my_left_arm, moment_limit);
    mz_left_arm = clamp_abs(mz_left_arm, moment_limit);

    fx_left_hand = clamp_abs(fx_left_hand, force_limit);
    fy_left_hand = clamp_abs(fy_left_hand, force_limit);
    fz_left_hand = clamp_abs(fz_left_hand, force_limit);
    mx_left_hand = clamp_abs(mx_left_hand, moment_limit);
    my_left_hand = clamp_abs(my_left_hand, moment_limit);
    mz_left_hand = clamp_abs(mz_left_hand, moment_limit);

    // 右臂
    fx_right_arm = clamp_abs(fx_right_arm, force_limit);
    fy_right_arm = clamp_abs(fy_right_arm, force_limit);
    fz_right_arm = clamp_abs(fz_right_arm, force_limit);
    mx_right_arm = clamp_abs(mx_right_arm, moment_limit);
    my_right_arm = clamp_abs(my_right_arm, moment_limit);
    mz_right_arm = clamp_abs(mz_right_arm, moment_limit);

    fx_right_hand = clamp_abs(fx_right_hand, force_limit);
    fy_right_hand = clamp_abs(fy_right_hand, force_limit);
    fz_right_hand = clamp_abs(fz_right_hand, force_limit);
    mx_right_hand = clamp_abs(mx_right_hand, moment_limit);
    my_right_hand = clamp_abs(my_right_hand, moment_limit);
    mz_right_hand = clamp_abs(mz_right_hand, moment_limit);

    // ============================================================
    // ===================== 左臂：动力学计算 ======================
    // ============================================================
    Eigen::VectorXd g_left = pinocchio::computeGeneralizedGravity(left_model, *left_data, q_left);

    Eigen::MatrixXd J_left_arm = Eigen::MatrixXd::Zero(6, left_model.nv);
    Eigen::MatrixXd J_left_hand = Eigen::MatrixXd::Zero(6, left_model.nv);

    pinocchio::computeFrameJacobian(left_model, *left_data, q_left, left_frame_id_arm, pinocchio::LOCAL, J_left_arm);
    pinocchio::computeFrameJacobian(left_model, *left_data, q_left, left_frame_id_hand, pinocchio::LOCAL, J_left_hand);

    pinocchio::Force f_left_arm = pinocchio::Force::Zero();
    f_left_arm.linear() << fx_left_arm, fy_left_arm, fz_left_arm;
    f_left_arm.angular() << mx_left_arm, my_left_arm, mz_left_arm;

    pinocchio::Force f_left_hand = pinocchio::Force::Zero();
    f_left_hand.linear() << fx_left_hand, fy_left_hand, fz_left_hand;
    f_left_hand.angular() << mx_left_hand, my_left_hand, mz_left_hand;

    Eigen::VectorXd tau_left_arm = J_left_arm.transpose() * f_left_arm.toVector();
    Eigen::VectorXd tau_left_hand = J_left_hand.transpose() * f_left_hand.toVector();
    Eigen::VectorXd tau_left_ext = tau_left_arm + tau_left_hand;

    Eigen::VectorXd damping_left = Eigen::VectorXd::Zero(left_model.nv);
    damping_left(0) = 0.0;
    damping_left(1) = 0.4;
    damping_left(2) = 0.3;
    damping_left(3) = 0.0;
    damping_left(4) = 0.0;
    damping_left(5) = 0.2;
    damping_left(6) = 0.3;

    Eigen::VectorXd tau_left_damping = -(damping_left.asDiagonal() * v_left);
    Eigen::VectorXd tau_left = g_left + tau_left_ext + tau_left_damping;

    // ============================================================
    // ===================== 右臂：动力学计算 ======================
    // ============================================================
    Eigen::VectorXd g_right = pinocchio::computeGeneralizedGravity(right_model, *right_data, q_right);

    Eigen::MatrixXd J_right_arm = Eigen::MatrixXd::Zero(6, right_model.nv);
    Eigen::MatrixXd J_right_hand = Eigen::MatrixXd::Zero(6, right_model.nv);

    pinocchio::computeFrameJacobian(right_model, *right_data, q_right, right_frame_id_arm, pinocchio::LOCAL, J_right_arm);
    pinocchio::computeFrameJacobian(right_model, *right_data, q_right, right_frame_id_hand, pinocchio::LOCAL, J_right_hand);

    pinocchio::Force f_right_arm = pinocchio::Force::Zero();
    f_right_arm.linear() << fx_right_arm, fy_right_arm, fz_right_arm;
    f_right_arm.angular() << mx_right_arm, my_right_arm, mz_right_arm;

    pinocchio::Force f_right_hand = pinocchio::Force::Zero();
    f_right_hand.linear() << fx_right_hand, fy_right_hand, fz_right_hand;
    f_right_hand.angular() << mx_right_hand, my_right_hand, mz_right_hand;

    Eigen::VectorXd tau_right_arm = J_right_arm.transpose() * f_right_arm.toVector();
    Eigen::VectorXd tau_right_hand = J_right_hand.transpose() * f_right_hand.toVector();
    Eigen::VectorXd tau_right_ext = tau_right_arm + tau_right_hand;

    Eigen::VectorXd damping_right = Eigen::VectorXd::Zero(right_model.nv);
    damping_right(0) = 0.0;
    damping_right(1) = 0.4;
    damping_right(2) = 0.3;
    damping_right(3) = 0.0;
    damping_right(4) = 0.0;
    damping_right(5) = 0.2;
    damping_right(6) = 0.3;

    Eigen::VectorXd tau_right_damping = -(damping_right.asDiagonal() * v_right);
    Eigen::VectorXd tau_right = g_right + tau_right_ext + tau_right_damping;

    // ===================== 左臂扭矩限幅 =====================
    tau_left(0) = clamp_abs(tau_left(0), 30.0);
    tau_left(1) = clamp_abs(tau_left(1), 20.0);
    tau_left(2) = clamp_abs(tau_left(2), 16.0);
    tau_left(3) = clamp_abs(tau_left(3), 12.0);
    tau_left(4) = clamp_abs(tau_left(4), 12.0);
    tau_left(5) = clamp_abs(tau_left(5), 12.0);
    tau_left(6) = clamp_abs(tau_left(6), 8.0);

    // ===================== 右臂扭矩限幅 =====================
    tau_right(0) = clamp_abs(-tau_right(0), 30.0);
    tau_right(1) = clamp_abs(-tau_right(1), 20.0);
    tau_right(2) = clamp_abs(-tau_right(2), 16.0);
    tau_right(3) = clamp_abs(-tau_right(3), 12.0);
    tau_right(4) = clamp_abs(-tau_right(4), 12.0);
    tau_right(5) = clamp_abs(-tau_right(5), 12.0);
    tau_right(6) = clamp_abs(-tau_right(6), 8.0);

    // ============================================================
    // ===================== 输出：左臂扭矩 ========================
    // ============================================================
    output.use_torque[IDX_L_J1] = true;
    output.use_torque[IDX_L_J2] = true;
    output.use_torque[IDX_L_J3] = true;
    output.use_torque[IDX_L_J4] = true;
    output.use_torque[IDX_L_J5] = true;
    output.use_torque[IDX_L_J6] = true;
    output.use_torque[IDX_L_J7] = true;

    output.target_torque[IDX_L_J1] = tau_left(0);
    output.target_torque[IDX_L_J2] = tau_left(1);
    output.target_torque[IDX_L_J3] = tau_left(2);
    output.target_torque[IDX_L_J4] = tau_left(3);
    output.target_torque[IDX_L_J5] = tau_left(4);
    output.target_torque[IDX_L_J6] = tau_left(5);
    output.target_torque[IDX_L_J7] = tau_left(6);

    // ============================================================
    // ===================== 输出：右臂扭矩 ========================
    // ============================================================
    output.use_torque[IDX_R_J11] = true;
    output.use_torque[IDX_R_J12] = true;
    output.use_torque[IDX_R_J13] = true;
    output.use_torque[IDX_R_J14] = true;
    output.use_torque[IDX_R_J15] = true;
    output.use_torque[IDX_R_J16] = true;
    output.use_torque[IDX_R_J17] = true;

    output.target_torque[IDX_R_J11] = tau_right(0);
    output.target_torque[IDX_R_J12] = tau_right(1);
    output.target_torque[IDX_R_J13] = tau_right(2);
    output.target_torque[IDX_R_J14] = tau_right(3);
    output.target_torque[IDX_R_J15] = tau_right(4);
    output.target_torque[IDX_R_J16] = tau_right(5);
    output.target_torque[IDX_R_J17] = tau_right(6);

    // ===================== 固定右臂末端 joint18~joint20 =====================
    output.use_position[IDX_R_J18] = true;
    output.target_position[IDX_R_J18] = 0.0;
    output.target_velocity[IDX_R_J18] = pulsePerMsToJointSpeedDegPerSecMagnitude(8, 1500.0);
    output.target_acc[IDX_R_J18] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(8, 5.0);
    output.target_dec[IDX_R_J18] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(8, 0.25);
    output.target_smooth_time[IDX_R_J18] = 25;

    output.use_position[IDX_R_J19] = true;
    output.target_position[IDX_R_J19] = 0.0;
    output.target_velocity[IDX_R_J19] = pulsePerMsToJointSpeedDegPerSecMagnitude(9, 1500.0);
    output.target_acc[IDX_R_J19] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(9, 5.0);
    output.target_dec[IDX_R_J19] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(9, 0.25);
    output.target_smooth_time[IDX_R_J19] = 25;

    output.use_position[IDX_R_J20] = true;
    output.target_position[IDX_R_J20] = 0.0;
    output.target_velocity[IDX_R_J20] = pulsePerMsToJointSpeedDegPerSecMagnitude(10, 1500.0);
    output.target_acc[IDX_R_J20] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(10, 5.0);
    output.target_dec[IDX_R_J20] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(10, 0.25);
    output.target_smooth_time[IDX_R_J20] = 25;

    // ===================== 固定左臂末端 joint8~joint10 =====================
    output.use_position[IDX_L_J8] = true;
    output.target_position[IDX_L_J8] = 0.0;
    output.target_velocity[IDX_L_J8] = pulsePerMsToJointSpeedDegPerSecMagnitude(18, 1500.0);
    output.target_acc[IDX_L_J8] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(18, 5.0);
    output.target_dec[IDX_L_J8] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(18, 0.25);
    output.target_smooth_time[IDX_L_J8] = 25;

    output.use_position[IDX_L_J9] = true;
    output.target_position[IDX_L_J9] = 0.0;
    output.target_velocity[IDX_L_J9] = pulsePerMsToJointSpeedDegPerSecMagnitude(19, 1500.0);
    output.target_acc[IDX_L_J9] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(19, 5.0);
    output.target_dec[IDX_L_J9] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(19, 0.25);
    output.target_smooth_time[IDX_L_J9] = 25;

    output.use_position[IDX_L_J10] = true;
    output.target_position[IDX_L_J10] = 0.0;
    output.target_velocity[IDX_L_J10] = pulsePerMsToJointSpeedDegPerSecMagnitude(20, 1500.0);
    output.target_acc[IDX_L_J10] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(20, 5.0);
    output.target_dec[IDX_L_J10] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(20, 0.25);
    output.target_smooth_time[IDX_L_J10] = 25;

    // ===================== 输出有效 =====================
    output.valid = true;
  }

  // ===================== 左臂预设位姿 =====================
  //
  // 当前实现：
  // 1. 只设置左臂相关控制层索引
  // 2. 左臂控制层索引范围为 10~19
  // 3. 目标位置单位是 deg
  // 4. 目标速度单位是 deg/s
  // 5. 目标加减速度单位是 deg/s^2
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

    // ===================== 左臂 物理轴18 -> 控制层索引17 =====================
    output.use_position[17] = true;
    output.target_position[17] = 0.0;
    output.target_velocity[17] = pulsePerMsToJointSpeedDegPerSecMagnitude(18, 1500.0);
    output.target_acc[17] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(18, 5.0);
    output.target_dec[17] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(18, 0.25);
    output.target_smooth_time[17] = 25;

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

  // ===================== 左臂被动训练模式（PID） =====================
  void runPassiveMode(const MotorStateCache &state, const SensorStateCache &sensor_state, ControlOutputCache &output)
  {
    (void)sensor_state;

    // ===================== 输出初始化 =====================
    //
    // joint1~3（背部）固定
    // joint4~6（肩）做正弦被动训练
    // joint7（肘）做正弦被动训练
    // 腕关节固定
    //
    output = ControlOutputCache{};
    output.valid = false;

    // ===================== 左臂 joint1~7 对应控制层索引 =====================
    //
    // joint1~3 -> 背部关节
    // joint4~6 -> 肩关节自由度
    // joint7   -> 肘关节
    //
    constexpr std::size_t IDX_J1 = 10;
    constexpr std::size_t IDX_J2 = 11;
    constexpr std::size_t IDX_J3 = 12;
    constexpr std::size_t IDX_J4 = 13;
    constexpr std::size_t IDX_J5 = 14;
    constexpr std::size_t IDX_J6 = 15;
    constexpr std::size_t IDX_J7 = 16;

    // ===================== 模式初始化 =====================
    //
    // 第一次进入被动模式时：
    // 1. 记录开始时间
    // 2. 记录各关节初始位置，作为正弦中心位置
    // 3. 清空 PID 积分项和上一时刻误差
    //
    static bool initialized = false;
    static std::chrono::steady_clock::time_point start_time;
    static std::chrono::steady_clock::time_point last_time;

    // 初始中心位置（deg）
    static double q0_j1 = 0.0;
    static double q0_j2 = 0.0;
    static double q0_j3 = 0.0;
    static double q0_j4 = 0.0;
    static double q0_j5 = 0.0;
    static double q0_j6 = 0.0;
    static double q0_j7 = 0.0;

    // PID 积分项
    static double e_int_j4 = 0.0;
    static double e_int_j5 = 0.0;
    static double e_int_j6 = 0.0;
    static double e_int_j7 = 0.0;

    // PID 上一时刻误差
    static double e_prev_j4 = 0.0;
    static double e_prev_j5 = 0.0;
    static double e_prev_j6 = 0.0;
    static double e_prev_j7 = 0.0;

    if (!initialized)
    {
      start_time = std::chrono::steady_clock::now();
      last_time = start_time;

      q0_j1 = state.position[IDX_J1];
      q0_j2 = state.position[IDX_J2];
      q0_j3 = state.position[IDX_J3];
      q0_j4 = state.position[IDX_J4];
      q0_j5 = state.position[IDX_J5];
      q0_j6 = state.position[IDX_J6];
      q0_j7 = state.position[IDX_J7];

      e_int_j4 = 0.0;
      e_int_j5 = 0.0;
      e_int_j6 = 0.0;
      e_int_j7 = 0.0;

      e_prev_j4 = 0.0;
      e_prev_j5 = 0.0;
      e_prev_j6 = 0.0;
      e_prev_j7 = 0.0;

      initialized = true;
    }

    // ===================== 时间计算 =====================
    const auto now = std::chrono::steady_clock::now();
    const double t =
        std::chrono::duration<double>(now - start_time).count();

    double dt =
        std::chrono::duration<double>(now - last_time).count();

    last_time = now;

    // 防止 dt 异常
    if (dt <= 1e-6 || dt > 0.1)
    {
      dt = 0.01;
    }

    // ===================== 当前关节角 =====================
    //
    // 单位：deg
    //
    const double q1 = state.position[IDX_J1];
    const double q2 = state.position[IDX_J2];
    const double q3 = state.position[IDX_J3];
    (void)q1;
    (void)q2;
    (void)q3; // 背部关节保持固定，当前值不参与控制计算
    const double q4 = state.position[IDX_J4];
    const double q5 = state.position[IDX_J5];
    const double q6 = state.position[IDX_J6];
    const double q7 = state.position[IDX_J7];

    // ===================== 正弦参考轨迹参数 =====================
    //
    //
    const double A4 = 8.0;  // deg
    const double A5 = 12.0; // deg
    const double A6 = 10.0; // deg
    const double A7 = 15.0; // deg

    const double f4 = 0.10; // Hz
    const double f5 = 0.10; // Hz
    const double f6 = 0.10; // Hz
    const double f7 = 0.12; // Hz

    const double w4 = 2.0 * M_PI * f4;
    const double w5 = 2.0 * M_PI * f5;
    const double w6 = 2.0 * M_PI * f6;
    const double w7 = 2.0 * M_PI * f7;

    // 相位设置
    //
    // 可以让 shoulder 之间不要完全同相，减小机械耦合突兀感
    //
    const double phase4 = 0.0;
    const double phase5 = M_PI / 2.0;
    const double phase6 = 0.0;
    const double phase7 = 0.0;

    // ===================== 生成参考轨迹 =====================
    //
    // 背部 joint1~3 固定在初始位置
    // 肩 joint4~6、肘 joint7 做正弦
    //
    const double q_ref_j1 = q0_j1;
    const double q_ref_j2 = q0_j2;
    const double q_ref_j3 = q0_j3;

    const double q_ref_j4 = q0_j4 + A4 * std::sin(w4 * t + phase4);
    const double q_ref_j5 = q0_j5 + A5 * std::sin(w5 * t + phase5);
    const double q_ref_j6 = q0_j6 + A6 * std::sin(w6 * t + phase6);
    const double q_ref_j7 = q0_j7 + A7 * std::sin(w7 * t + phase7);

    // ===================== PID 参数 =====================
    //
    // 采用“位置参考 + PID 修正”的方式：
    // q_cmd = q_ref + Kp*e + Ki*∫e + Kd*de
    //
    // 注意：
    // 1. 先小参数
    // 2. Ki 建议先很小或直接 0
    // 3. 若出现抖动，优先减小 Kp、Kd
    //
    const double Kp4 = 0.8;
    const double Ki4 = 0.00;
    const double Kd4 = 0.05;

    const double Kp5 = 0.8;
    const double Ki5 = 0.00;
    const double Kd5 = 0.05;

    const double Kp6 = 0.8;
    const double Ki6 = 0.00;
    const double Kd6 = 0.05;

    const double Kp7 = 0.8;
    const double Ki7 = 0.00;
    const double Kd7 = 0.05;

    // ===================== PID 误差计算 =====================
    double e4 = q_ref_j4 - q4;
    double e5 = q_ref_j5 - q5;
    double e6 = q_ref_j6 - q6;
    double e7 = q_ref_j7 - q7;

    e_int_j4 += e4 * dt;
    e_int_j5 += e5 * dt;
    e_int_j6 += e6 * dt;
    e_int_j7 += e7 * dt;

    // 积分限幅，防止积分饱和
    auto clamp_between = [](double x, double lo, double hi) -> double
    {
      if (x < lo)
        return lo;
      if (x > hi)
        return hi;
      return x;
    };

    e_int_j4 = clamp_between(e_int_j4, -20.0, 20.0);
    e_int_j5 = clamp_between(e_int_j5, -20.0, 20.0);
    e_int_j6 = clamp_between(e_int_j6, -20.0, 20.0);
    e_int_j7 = clamp_between(e_int_j7, -20.0, 20.0);

    const double de4 = (e4 - e_prev_j4) / dt;
    const double de5 = (e5 - e_prev_j5) / dt;
    const double de6 = (e6 - e_prev_j6) / dt;
    const double de7 = (e7 - e_prev_j7) / dt;

    e_prev_j4 = e4;
    e_prev_j5 = e5;
    e_prev_j6 = e6;
    e_prev_j7 = e7;

    // ===================== PID 输出位置命令 =====================
    double q_cmd_j4 = q_ref_j4 + Kp4 * e4 + Ki4 * e_int_j4 + Kd4 * de4;
    double q_cmd_j5 = q_ref_j5 + Kp5 * e5 + Ki5 * e_int_j5 + Kd5 * de5;
    double q_cmd_j6 = q_ref_j6 + Kp6 * e6 + Ki6 * e_int_j6 + Kd6 * de6;
    double q_cmd_j7 = q_ref_j7 + Kp7 * e7 + Ki7 * e_int_j7 + Kd7 * de7;

    // ===================== 安全限幅 =====================
    //
    // 对“相对初始位置”的摆动幅度再做一次保护
    //
    q_cmd_j4 = clamp_between(q_cmd_j4, q0_j4 - 15.0, q0_j4 + 15.0);
    q_cmd_j5 = clamp_between(q_cmd_j5, q0_j5 - 20.0, q0_j5 + 20.0);
    q_cmd_j6 = clamp_between(q_cmd_j6, q0_j6 - 20.0, q0_j6 + 20.0);
    q_cmd_j7 = clamp_between(q_cmd_j7, q0_j7 - 25.0, q0_j7 + 25.0);

    // ===================== 输出：背部 joint1~3 固定 =====================
    output.use_position[IDX_J1] = true;
    output.target_position[IDX_J1] = q_ref_j1;

    output.use_position[IDX_J2] = true;
    output.target_position[IDX_J2] = q_ref_j2;

    output.use_position[IDX_J3] = true;
    output.target_position[IDX_J3] = q_ref_j3;

    // ===================== 输出：肩 joint4~6、肘 joint7 被动轨迹 =====================
    output.use_position[IDX_J4] = true;
    output.target_position[IDX_J4] = q_cmd_j4;

    output.use_position[IDX_J5] = true;
    output.target_position[IDX_J5] = q_cmd_j5;

    output.use_position[IDX_J6] = true;
    output.target_position[IDX_J6] = q_cmd_j6;

    output.use_position[IDX_J7] = true;
    output.target_position[IDX_J7] = q_cmd_j7;

    // ===================== 统一设置速度/加速度参数 =====================
    //
    //
    output.target_velocity[IDX_J1] = pulsePerMsToJointSpeedDegPerSecMagnitude(11, 400.0);
    output.target_acc[IDX_J1] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(11, 0.5);
    output.target_dec[IDX_J1] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(11, 0.25);
    output.target_smooth_time[IDX_J1] = 25;

    output.target_velocity[IDX_J2] = pulsePerMsToJointSpeedDegPerSecMagnitude(12, 500.0);
    output.target_acc[IDX_J2] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(12, 0.5);
    output.target_dec[IDX_J2] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(12, 0.25);
    output.target_smooth_time[IDX_J2] = 25;

    output.target_velocity[IDX_J3] = pulsePerMsToJointSpeedDegPerSecMagnitude(13, 500.0);
    output.target_acc[IDX_J3] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(13, 0.5);
    output.target_dec[IDX_J3] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(13, 0.25);
    output.target_smooth_time[IDX_J3] = 25;

    output.target_velocity[IDX_J4] = pulsePerMsToJointSpeedDegPerSecMagnitude(14, 1200.0);
    output.target_acc[IDX_J4] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(14, 3.0);
    output.target_dec[IDX_J4] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(14, 3.0);
    output.target_smooth_time[IDX_J4] = 25;

    output.target_velocity[IDX_J5] = pulsePerMsToJointSpeedDegPerSecMagnitude(15, 1200.0);
    output.target_acc[IDX_J5] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(15, 3.0);
    output.target_dec[IDX_J5] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(15, 3.0);
    output.target_smooth_time[IDX_J5] = 25;

    output.target_velocity[IDX_J6] = pulsePerMsToJointSpeedDegPerSecMagnitude(16, 1200.0);
    output.target_acc[IDX_J6] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(16, 3.0);
    output.target_dec[IDX_J6] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(16, 3.0);
    output.target_smooth_time[IDX_J6] = 25;

    output.target_velocity[IDX_J7] = pulsePerMsToJointSpeedDegPerSecMagnitude(17, 1200.0);
    output.target_acc[IDX_J7] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(17, 3.0);
    output.target_dec[IDX_J7] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(17, 3.0);
    output.target_smooth_time[IDX_J7] = 25;

    // ===================== 末端关节固定 =====================
    //
    // 固定左臂物理轴18、19、20。
    //
    output.use_position[17] = true;
    output.target_position[17] = 0.0;
    output.target_velocity[17] = pulsePerMsToJointSpeedDegPerSecMagnitude(18, 1500.0);
    output.target_acc[17] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(18, 5.0);
    output.target_dec[17] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(18, 0.25);
    output.target_smooth_time[17] = 25;

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

    // ===================== 输出有效 =====================
    output.valid = true;
  }

  void runPassivePDFFMode(const MotorStateCache &state, const SensorStateCache &sensor_state, ControlOutputCache &output)
  {
    (void)sensor_state;

    // ===================== 输出初始化 =====================
    //
    // 本模式采用位置模式：
    // 1. joint1~3（背部）固定
    // 2. joint4~6（肩）做正弦被动训练
    // 3. joint7（肘）做正弦被动训练
    // 4. 腕关节固定
    //
    // 与 runPassiveMode(...) 的区别：
    // 这里用 PD + 速度前馈修正正弦参考。
    //
    output = ControlOutputCache{};
    output.valid = false;

    // ===================== 左臂 joint1~7 对应控制层索引 =====================
    //
    // joint1~3 -> 背部关节
    // joint4~6 -> 肩关节自由度
    // joint7   -> 肘关节
    //
    constexpr std::size_t IDX_J1 = 10;
    constexpr std::size_t IDX_J2 = 11;
    constexpr std::size_t IDX_J3 = 12;
    constexpr std::size_t IDX_J4 = 13;
    constexpr std::size_t IDX_J5 = 14;
    constexpr std::size_t IDX_J6 = 15;
    constexpr std::size_t IDX_J7 = 16;

    // ===================== 模式初始化 =====================
    //
    // 第一次进入该模式时：
    // 1. 记录起始时间
    // 2. 记录初始位置，作为正弦轨迹中心位置
    //
    static bool initialized = false;
    static std::chrono::steady_clock::time_point start_time;

    // 进入模式时各关节中心位置（deg）
    static double q0_j1 = 0.0;
    static double q0_j2 = 0.0;
    static double q0_j3 = 0.0;
    static double q0_j4 = 0.0;
    static double q0_j5 = 0.0;
    static double q0_j6 = 0.0;
    static double q0_j7 = 0.0;

    if (!initialized)
    {
      start_time = std::chrono::steady_clock::now();

      q0_j1 = state.position[IDX_J1];
      q0_j2 = state.position[IDX_J2];
      q0_j3 = state.position[IDX_J3];
      q0_j4 = state.position[IDX_J4];
      q0_j5 = state.position[IDX_J5];
      q0_j6 = state.position[IDX_J6];
      q0_j7 = state.position[IDX_J7];

      initialized = true;
    }

    // ===================== 当前时间 =====================
    const auto now = std::chrono::steady_clock::now();
    const double t =
        std::chrono::duration<double>(now - start_time).count();

    // ===================== 当前关节位置 / 速度 =====================
    //
    // position -> deg
    // velocity -> deg/s
    //
    const double q1 = state.position[IDX_J1];
    const double q2 = state.position[IDX_J2];
    const double q3 = state.position[IDX_J3];
    const double q4 = state.position[IDX_J4];
    const double q5 = state.position[IDX_J5];
    const double q6 = state.position[IDX_J6];
    const double q7 = state.position[IDX_J7];

    const double dq1 = state.velocity[IDX_J1];
    const double dq2 = state.velocity[IDX_J2];
    const double dq3 = state.velocity[IDX_J3];
    const double dq4 = state.velocity[IDX_J4];
    const double dq5 = state.velocity[IDX_J5];
    const double dq6 = state.velocity[IDX_J6];
    const double dq7 = state.velocity[IDX_J7];

    (void)q1;
    (void)q2;
    (void)q3;
    (void)dq1;
    (void)dq2;
    (void)dq3;

    // ===================== 正弦轨迹参数 =====================
    //
    // 背部 joint1~3 固定，不参与摆动
    // 肩 joint4~6、肘 joint7 参与被动训练
    //
    // 位置参考：
    // q_ref = q0 + A * sin(wt + phase)
    //
    // 速度前馈：
    // dq_ref = A * w * cos(wt + phase)
    //
    // 说明：
    // 1. A 的单位是 deg
    // 2. f 的单位是 Hz
    // 3. w = 2*pi*f，单位是 rad/s
    // 4. 因为 sin/cos 只是无量纲函数，所以 dq_ref 最终单位仍是 deg/s
    //
    const double A4 = 8.0;  // deg
    const double A5 = 12.0; // deg
    const double A6 = 10.0; // deg
    const double A7 = 15.0; // deg

    const double f4 = 0.10; // Hz
    const double f5 = 0.10; // Hz
    const double f6 = 0.10; // Hz
    const double f7 = 0.12; // Hz

    const double w4 = 2.0 * M_PI * f4;
    const double w5 = 2.0 * M_PI * f5;
    const double w6 = 2.0 * M_PI * f6;
    const double w7 = 2.0 * M_PI * f7;

    // 可根据需要设置不同相位
    const double phase4 = 0.0;
    const double phase5 = M_PI / 2.0;
    const double phase6 = 0.0;
    const double phase7 = 0.0;

    // ===================== 位置参考轨迹 =====================
    const double q_ref_j1 = q0_j1;
    const double q_ref_j2 = q0_j2;
    const double q_ref_j3 = q0_j3;

    const double q_ref_j4 = q0_j4 + A4 * std::sin(w4 * t + phase4);
    const double q_ref_j5 = q0_j5 + A5 * std::sin(w5 * t + phase5);
    const double q_ref_j6 = q0_j6 + A6 * std::sin(w6 * t + phase6);
    const double q_ref_j7 = q0_j7 + A7 * std::sin(w7 * t + phase7);

    // ===================== 速度前馈轨迹 =====================
    //
    // 对正弦轨迹直接求导：
    //
    // q_ref  = q0 + A * sin(wt + phase)
    // dq_ref = A * w * cos(wt + phase)
    //
    const double dq_ref_j4 = A4 * w4 * std::cos(w4 * t + phase4);
    const double dq_ref_j5 = A5 * w5 * std::cos(w5 * t + phase5);
    const double dq_ref_j6 = A6 * w6 * std::cos(w6 * t + phase6);
    const double dq_ref_j7 = A7 * w7 * std::cos(w7 * t + phase7);

    // ===================== PD + 前馈参数 =====================
    //
    // 这里不使用 Ki：
    // 1. 当前是周期正弦轨迹，不是静态定点
    // 2. 驱动器底层通常已有位置环
    // 3. 外环再加积分，容易引起慢振荡或发飘
    //
    // 这里的 Kd 乘的是“速度误差”：
    // (dq_ref - dq)
    //
    const double Kp4 = 0.8;
    const double Kd4 = 0.08;

    const double Kp5 = 0.8;
    const double Kd5 = 0.08;

    const double Kp6 = 0.8;
    const double Kd6 = 0.08;

    const double Kp7 = 0.8;
    const double Kd7 = 0.08;

    // ===================== 位置误差 =====================
    const double e4 = q_ref_j4 - q4;
    const double e5 = q_ref_j5 - q5;
    const double e6 = q_ref_j6 - q6;
    const double e7 = q_ref_j7 - q7;

    // ===================== 速度误差 =====================
    //
    // 传统PID 是直接使用位置误差求导得到速度误差，即 de = d(q_ref - q) = dq_ref - dq
    // 直接使用目标速度 - 实际速度
    //
    const double ev4 = dq_ref_j4 - dq4;
    const double ev5 = dq_ref_j5 - dq5;
    const double ev6 = dq_ref_j6 - dq6;
    const double ev7 = dq_ref_j7 - dq7;

    // ===================== 生成控制命令 =====================
    //
    // q_cmd = q_ref + Kp * (q_ref - q) + Kd * (dq_ref - dq)
    //
    // 理解方式：
    // 1. q_ref                    -> 基础轨迹
    // 2. Kp * (q_ref - q)         -> 位置纠偏
    // 3. Kd * (dq_ref - dq)       -> 速度前馈 + 速度误差修正
    //
    double q_cmd_j4 = q_ref_j4 + Kp4 * e4 + Kd4 * ev4;
    double q_cmd_j5 = q_ref_j5 + Kp5 * e5 + Kd5 * ev5;
    double q_cmd_j6 = q_ref_j6 + Kp6 * e6 + Kd6 * ev6;
    double q_cmd_j7 = q_ref_j7 + Kp7 * e7 + Kd7 * ev7;

    // ===================== 安全限幅 =====================
    //
    // 限制最终输出，避免在大误差时超出训练范围
    //
    auto clamp_between = [](double x, double lo, double hi) -> double
    {
      if (x < lo)
        return lo;
      if (x > hi)
        return hi;
      return x;
    };

    q_cmd_j4 = clamp_between(q_cmd_j4, q0_j4 - 15.0, q0_j4 + 15.0);
    q_cmd_j5 = clamp_between(q_cmd_j5, q0_j5 - 20.0, q0_j5 + 20.0);
    q_cmd_j6 = clamp_between(q_cmd_j6, q0_j6 - 20.0, q0_j6 + 20.0);
    q_cmd_j7 = clamp_between(q_cmd_j7, q0_j7 - 25.0, q0_j7 + 25.0);

    // ===================== 输出：背部 joint1~3 固定 =====================
    output.use_position[IDX_J1] = true;
    output.target_position[IDX_J1] = q_ref_j1;

    output.use_position[IDX_J2] = true;
    output.target_position[IDX_J2] = q_ref_j2;

    output.use_position[IDX_J3] = true;
    output.target_position[IDX_J3] = q_ref_j3;

    // ===================== 输出：肩 joint4~6、肘 joint7 正弦被动轨迹 =====================
    output.use_position[IDX_J4] = true;
    output.target_position[IDX_J4] = q_cmd_j4;

    output.use_position[IDX_J5] = true;
    output.target_position[IDX_J5] = q_cmd_j5;

    output.use_position[IDX_J6] = true;
    output.target_position[IDX_J6] = q_cmd_j6;

    output.use_position[IDX_J7] = true;
    output.target_position[IDX_J7] = q_cmd_j7;

    // ===================== 统一设置速度/加速度参数 =====================
    //
    // 背部关节保守一些
    // 肩、肘训练关节稍快一些
    //
    output.target_velocity[IDX_J1] = pulsePerMsToJointSpeedDegPerSecMagnitude(11, 400.0);
    output.target_acc[IDX_J1] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(11, 0.5);
    output.target_dec[IDX_J1] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(11, 0.25);
    output.target_smooth_time[IDX_J1] = 25;

    output.target_velocity[IDX_J2] = pulsePerMsToJointSpeedDegPerSecMagnitude(12, 500.0);
    output.target_acc[IDX_J2] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(12, 0.5);
    output.target_dec[IDX_J2] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(12, 0.25);
    output.target_smooth_time[IDX_J2] = 25;

    output.target_velocity[IDX_J3] = pulsePerMsToJointSpeedDegPerSecMagnitude(13, 500.0);
    output.target_acc[IDX_J3] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(13, 0.5);
    output.target_dec[IDX_J3] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(13, 0.25);
    output.target_smooth_time[IDX_J3] = 25;

    output.target_velocity[IDX_J4] = pulsePerMsToJointSpeedDegPerSecMagnitude(14, 1200.0);
    output.target_acc[IDX_J4] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(14, 3.0);
    output.target_dec[IDX_J4] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(14, 3.0);
    output.target_smooth_time[IDX_J4] = 25;

    output.target_velocity[IDX_J5] = pulsePerMsToJointSpeedDegPerSecMagnitude(15, 1200.0);
    output.target_acc[IDX_J5] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(15, 3.0);
    output.target_dec[IDX_J5] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(15, 3.0);
    output.target_smooth_time[IDX_J5] = 25;

    output.target_velocity[IDX_J6] = pulsePerMsToJointSpeedDegPerSecMagnitude(16, 1200.0);
    output.target_acc[IDX_J6] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(16, 3.0);
    output.target_dec[IDX_J6] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(16, 3.0);
    output.target_smooth_time[IDX_J6] = 25;

    output.target_velocity[IDX_J7] = pulsePerMsToJointSpeedDegPerSecMagnitude(17, 1200.0);
    output.target_acc[IDX_J7] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(17, 3.0);
    output.target_dec[IDX_J7] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(17, 3.0);
    output.target_smooth_time[IDX_J7] = 25;

    // ===================== 末端关节固定 =====================
    output.use_position[17] = true;
    output.target_position[17] = 0.0;
    output.target_velocity[17] = pulsePerMsToJointSpeedDegPerSecMagnitude(18, 1500.0);
    output.target_acc[17] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(18, 5.0);
    output.target_dec[17] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(18, 0.25);
    output.target_smooth_time[17] = 25;

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

    // ===================== 输出有效 =====================
    output.valid = true;
  }

  // ===================== 扭矩被动模式 =====================
  void runPassiveTorqueMode(const MotorStateCache &state, const SensorStateCache &sensor_state, ControlOutputCache &output)
  {
    (void)sensor_state;

    // ===================== 输出初始化 =====================
    //
    // 本模式采用扭矩模式：
    // 1. joint1~3（背部）默认不参与正弦跟踪，保持零扭矩输出
    // 2. joint4~6（肩）做正弦被动训练
    // 3. joint7（肘）做正弦被动训练
    // 4. 腕关节固定在位置模式
    //
    output = ControlOutputCache{};
    output.valid = false;

    // ===================== 左臂 joint1~7 控制层索引 =====================
    constexpr std::size_t IDX_J1 = 10;
    constexpr std::size_t IDX_J2 = 11;
    constexpr std::size_t IDX_J3 = 12;
    constexpr std::size_t IDX_J4 = 13;
    constexpr std::size_t IDX_J5 = 14;
    constexpr std::size_t IDX_J6 = 15;
    constexpr std::size_t IDX_J7 = 16;

    // ===================== Pinocchio 懒加载上下文 =====================
    //
    // 当前模型按已有工程设置，左臂 joint1~7 为可动关节。
    //
    static bool pin_initialized = false;
    static bool pin_init_failed = false;
    static pinocchio::Model model;
    static std::unique_ptr<pinocchio::Data> data;

    if (!pin_initialized && !pin_init_failed)
    {
      try
      {
        std::string urdf_path =
            ament_index_cpp::get_package_share_directory("upperlimb_robot") +
            "/urdf/left_arm_experiment.urdf";

        pinocchio::urdf::buildModel(urdf_path, model);
        data = std::make_unique<pinocchio::Data>(model);

        if (model.nq != 7 || model.nv != 7)
        {
          std::cerr
              << "[runPassiveTorqueMode] 当前 Pinocchio 模型自由度不是 7，"
              << "model.nq=" << model.nq << " model.nv=" << model.nv
              << std::endl;
          pin_init_failed = true;
        }
        else
        {
          pin_initialized = true;
          std::cout
              << "[runPassiveTorqueMode] Pinocchio 初始化成功。"
              << " nq=" << model.nq
              << " nv=" << model.nv
              << std::endl;
        }
      }
      catch (const std::exception &e)
      {
        std::cerr << "[runPassiveTorqueMode] Pinocchio 初始化异常: " << e.what() << std::endl;
        pin_init_failed = true;
      }
    }

    if (!pin_initialized || pin_init_failed || !data)
    {
      output.valid = false;
      return;
    }

    // ===================== 模式内部状态 =====================
    //
    // 使用调用间隔判断是否属于重新进入模式，
    // 避免额外新增 reset 函数。
    //
    static bool initialized = false;
    static std::chrono::steady_clock::time_point start_time;
    static std::chrono::steady_clock::time_point last_call_time;

    // 进入模式时的中心位置（deg）
    static double q0_j4 = 0.0;
    static double q0_j5 = 0.0;
    static double q0_j6 = 0.0;
    static double q0_j7 = 0.0;

    const auto now = std::chrono::steady_clock::now();

    bool reenter_mode = false;
    if (initialized)
    {
      const double gap =
          std::chrono::duration<double>(now - last_call_time).count();

      if (gap > 0.2)
      {
        reenter_mode = true;
      }
    }

    if (!initialized || reenter_mode)
    {
      start_time = now;
      last_call_time = now;

      q0_j4 = state.position[IDX_J4];
      q0_j5 = state.position[IDX_J5];
      q0_j6 = state.position[IDX_J6];
      q0_j7 = state.position[IDX_J7];

      initialized = true;
    }

    const double t =
        std::chrono::duration<double>(now - start_time).count();
    last_call_time = now;

    // ===================== 组装 q / v =====================
    //
    // Pinocchio 使用 rad / rad/s
    //
    constexpr double DEG2RAD = M_PI / 180.0;

    Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);

    q(0) = state.position[IDX_J1] * DEG2RAD;
    q(1) = state.position[IDX_J2] * DEG2RAD;
    q(2) = state.position[IDX_J3] * DEG2RAD;
    q(3) = state.position[IDX_J4] * DEG2RAD;
    q(4) = state.position[IDX_J5] * DEG2RAD;
    q(5) = state.position[IDX_J6] * DEG2RAD;
    q(6) = state.position[IDX_J7] * DEG2RAD;

    v(0) = state.velocity[IDX_J1] * DEG2RAD;
    v(1) = state.velocity[IDX_J2] * DEG2RAD;
    v(2) = state.velocity[IDX_J3] * DEG2RAD;
    v(3) = state.velocity[IDX_J4] * DEG2RAD;
    v(4) = state.velocity[IDX_J5] * DEG2RAD;
    v(5) = state.velocity[IDX_J6] * DEG2RAD;
    v(6) = state.velocity[IDX_J7] * DEG2RAD;

    // ===================== 正弦参考轨迹 =====================
    //
    // 背部关节不做正弦训练，肩肘关节做正弦参考
    //
    // 幅值 A 是正弦参考轨迹的幅度，单位是 deg
    const double A4 = 8.0 * DEG2RAD;
    const double A5 = 12.0 * DEG2RAD;
    const double A6 = 10.0 * DEG2RAD;
    const double A7 = 30.0 * DEG2RAD;

    const double f4 = 0.10;
    const double f5 = 0.10;
    const double f6 = 0.10;
    const double f7 = 0.12;

    const double w4 = 2.0 * M_PI * f4;
    const double w5 = 2.0 * M_PI * f5;
    const double w6 = 2.0 * M_PI * f6;
    const double w7 = 2.0 * M_PI * f7;

    const double phase4 = 0.0;
    const double phase5 = M_PI / 2.0;
    const double phase6 = 0.0;
    const double phase7 = 0.0;

    Eigen::VectorXd q_ref = q;
    Eigen::VectorXd dq_ref = Eigen::VectorXd::Zero(model.nv);

    // 背部关节固定在当前实际位置，不主动拉动
    q_ref(0) = q(0);
    q_ref(1) = q(1);
    q_ref(2) = q(2);
    dq_ref(0) = 0.0;
    dq_ref(1) = 0.0;
    dq_ref(2) = 0.0;

    q_ref(3) = q0_j4 * DEG2RAD + A4 * std::sin(w4 * t + phase4);
    q_ref(4) = q0_j5 * DEG2RAD + A5 * std::sin(w5 * t + phase5);
    q_ref(5) = q0_j6 * DEG2RAD + A6 * std::sin(w6 * t + phase6);
    q_ref(6) = q0_j7 * DEG2RAD + A7 * std::sin(w7 * t + phase7);

    dq_ref(3) = A4 * w4 * std::cos(w4 * t + phase4);
    dq_ref(4) = A5 * w5 * std::cos(w5 * t + phase5);
    dq_ref(5) = A6 * w6 * std::cos(w6 * t + phase6);
    dq_ref(6) = A7 * w7 * std::cos(w7 * t + phase7);

    // ===================== 重力补偿 =====================
    //
    // g(q) 为当前姿态下的重力力矩
    //
    Eigen::VectorXd g = pinocchio::computeGeneralizedGravity(model, *data, q);

    // ===================== 关节空间弹簧阻尼参数 =====================
    //
    // tau_cmd = g + Kp*(q_ref-q) + Kd*(dq_ref-v)
    //
    // 背部关节不做主训练，因此给较小或零跟踪刚度
    //
    Eigen::VectorXd Kp = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd Kd = Eigen::VectorXd::Zero(model.nv);

    // 背部
    Kp(0) = 0.0;
    Kp(1) = 0.0;
    Kp(2) = 0.0;

    Kd(0) = 0.5;
    Kd(1) = 0.5;
    Kd(2) = 0.3;

    // 肩肘
    Kp(3) = 18.0;
    Kp(4) = 18.0;
    Kp(5) = 16.0;
    //
    Kp(6) = 2.5;

    Kd(3) = 2.5;
    Kd(4) = 2.5;
    Kd(5) = 2.0;
    //
    Kd(6) = 1.5;

    // ===================== 计算扭矩命令 =====================
    Eigen::VectorXd tau_cmd =
        g + Kp.asDiagonal() * (q_ref - q) + Kd.asDiagonal() * (dq_ref - v);

    // ===================== 额外阻尼保护 =====================
    //
    // 在训练关节上增加少量纯阻尼，防止轻微发飘
    //
    Eigen::VectorXd damping = Eigen::VectorXd::Zero(model.nv);
    damping(3) = 0.3;
    damping(4) = 0.3;
    damping(5) = 0.2;
    damping(6) = 0.2;

    tau_cmd += -(damping.asDiagonal() * v);
    // ===================== 基础扭矩（for循环版） =====================
    //
    // 给 joint4~7 增加一个随目标速度方向变化的小扭矩
    // 不参与误差计算，只做辅助驱动
    // 在低速区关闭，避免换向抖动
    //

    const double vel_deadband = 0.05; // rad/s

    // joint4~7 的基础扭矩幅值
    double base_tau[4];
    base_tau[0] = 0.4; // joint4
    base_tau[1] = 0.4; // joint5
    base_tau[2] = 0.3; // joint6
    base_tau[3] = 0.2; // joint7

    // 遍历 joint4~7（对应索引 3~6）
    for (int i = 0; i < 4; ++i)
    {
      const int idx = 3 + i;

      double tau_base = 0.0;

      // 判断目标速度方向
      if (dq_ref(idx) > vel_deadband)
      {
        tau_base = base_tau[i];
      }
      else if (dq_ref(idx) < -vel_deadband)
      {
        tau_base = -base_tau[i];
      }
      else
      {
        tau_base = 0.0;
      }

      // 叠加到总扭矩
      tau_cmd(idx) += tau_base;
    }

    // ===================== 扭矩限幅 =====================
    //
    // 先保守设置，后续再根据实际效果调
    //
    auto clamp_abs = [](double x, double lim) -> double
    {
      if (x > lim)
        return lim;
      if (x < -lim)
        return -lim;
      return x;
    };

    const double joint_torque_limit_j1 = 8.0;
    const double joint_torque_limit_j2 = 8.0;
    const double joint_torque_limit_j3 = 6.0;
    const double joint_torque_limit_j4 = 10.0;
    const double joint_torque_limit_j5 = 10.0;
    const double joint_torque_limit_j6 = 8.0;
    const double joint_torque_limit_j7 = 6.0;

    tau_cmd(0) = clamp_abs(tau_cmd(0), joint_torque_limit_j1);
    tau_cmd(1) = clamp_abs(tau_cmd(1), joint_torque_limit_j2);
    tau_cmd(2) = clamp_abs(tau_cmd(2), joint_torque_limit_j3);
    tau_cmd(3) = clamp_abs(tau_cmd(3), joint_torque_limit_j4);
    tau_cmd(4) = clamp_abs(tau_cmd(4), joint_torque_limit_j5);
    tau_cmd(5) = clamp_abs(tau_cmd(5), joint_torque_limit_j6);
    tau_cmd(6) = clamp_abs(tau_cmd(6), joint_torque_limit_j7);

    // ===================== 输出到控制层 =====================
    //
    // 左臂 joint1~7 采用扭矩输出
    //
    // output.use_torque[IDX_J1] = true;
    // output.use_torque[IDX_J2] = true;
    // output.use_torque[IDX_J3] = true;
    // output.use_torque[IDX_J4] = true;
    // output.use_torque[IDX_J5] = true;
    // output.use_torque[IDX_J6] = true;
    output.use_torque[IDX_J7] = true;

    // output.target_torque[IDX_J1] = tau_cmd(0);
    // output.target_torque[IDX_J2] = tau_cmd(1);
    // output.target_torque[IDX_J3] = tau_cmd(2);
    // output.target_torque[IDX_J4] = tau_cmd(3);
    // output.target_torque[IDX_J5] = tau_cmd(4);
    // output.target_torque[IDX_J6] = tau_cmd(5);
    output.target_torque[IDX_J7] = tau_cmd(6);

    // ===================== 关节固定在位置模式 =====================
    //
    //
    // 背部关节 joint1~3 固定在当前位置，防止发飘
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

    // 肩部关节 joint4~6 固定到预设角度，防止发飘
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

    // 防止末端未参与训练关节乱动
    //
    output.use_position[17] = true;
    output.target_position[17] = 0.0;
    output.target_velocity[17] = pulsePerMsToJointSpeedDegPerSecMagnitude(18, 1500.0);
    output.target_acc[17] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(18, 5.0);
    output.target_dec[17] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(18, 0.25);
    output.target_smooth_time[17] = 25;

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

    output.valid = true;
  }

  // ===================== 镜像模式 =====================
  //
  // 当前实现目标：
  // 1. 左臂作为示教侧，沿用现有左臂零力模式输出
  // 2. 右臂作为执行侧，使用扭矩模式跟踪左臂动作
  // 3. 右臂跟踪采用：重力补偿 + 关节空间 PD 扭矩
  // 4. 右臂末端索引 8/9 继续固定，索引 7 保持默认不控制
  //
  // 说明：
  // 1. 左右关节镜像方向先通过 mirror_sign[] 手工配置
  // 2. 某个关节方向反，只改 mirror_sign 对应项即可
  //
  void runMirrorMode(const MotorStateCache &state, const SensorStateCache &sensor_state, ControlOutputCache &output)
  {
    // ===================== 输出初始化 =====================
    output = ControlOutputCache{};
    output.valid = false;

    // ===================== 左臂控制层索引 10~16 =====================
    constexpr std::size_t IDX_L1 = 10;
    constexpr std::size_t IDX_L2 = 11;
    constexpr std::size_t IDX_L3 = 12;
    constexpr std::size_t IDX_L4 = 13;
    constexpr std::size_t IDX_L5 = 14;
    constexpr std::size_t IDX_L6 = 15;
    constexpr std::size_t IDX_L7 = 16;

    // ===================== 右臂控制层索引 0~6 =====================
    constexpr std::size_t IDX_R1 = 0;
    constexpr std::size_t IDX_R2 = 1;
    constexpr std::size_t IDX_R3 = 2;
    constexpr std::size_t IDX_R4 = 3;
    constexpr std::size_t IDX_R5 = 4;
    constexpr std::size_t IDX_R6 = 5;
    constexpr std::size_t IDX_R7 = 6;

    // ===================== 右臂末端索引 =====================
    constexpr std::size_t IDX_R9 = 8;
    constexpr std::size_t IDX_R10 = 9;

    // ===================== 第一步：先把左臂零力输出合进来 =====================
    ControlOutputCache left_output{};
    runZeroForceMode(state, sensor_state, left_output);

    if (left_output.valid)
    {
      for (std::size_t i = 10; i < 20; ++i)
      {
        output.use_position[i] = left_output.use_position[i];
        output.target_position[i] = left_output.target_position[i];
        output.target_velocity[i] = left_output.target_velocity[i];
        output.target_acc[i] = left_output.target_acc[i];
        output.target_dec[i] = left_output.target_dec[i];
        output.target_smooth_time[i] = left_output.target_smooth_time[i];

        output.use_torque[i] = left_output.use_torque[i];
        output.target_torque[i] = left_output.target_torque[i];
      }
    }
    else
    {
      // ===================== 左臂零力失败时的兜底 =====================
      output.use_torque[IDX_L1] = true;
      output.use_torque[IDX_L2] = true;
      output.use_torque[IDX_L3] = true;
      output.use_torque[IDX_L4] = true;
      output.use_torque[IDX_L5] = true;
      output.use_torque[IDX_L6] = true;
      output.use_torque[IDX_L7] = true;

      output.target_torque[IDX_L1] = 0.0;
      output.target_torque[IDX_L2] = 0.0;
      output.target_torque[IDX_L3] = 0.0;
      output.target_torque[IDX_L4] = 0.0;
      output.target_torque[IDX_L5] = 0.0;
      output.target_torque[IDX_L6] = 0.0;
      output.target_torque[IDX_L7] = 0.0;

      output.use_position[17] = true;
      output.target_position[17] = 0.0;
      output.target_velocity[17] = pulsePerMsToJointSpeedDegPerSecMagnitude(18, 1500.0);
      output.target_acc[17] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(18, 5.0);
      output.target_dec[17] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(18, 0.25);
      output.target_smooth_time[17] = 25;

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

    // ===================== 第二步：右臂 Pinocchio 懒加载 =====================
    static bool pin_initialized = false;
    static bool pin_init_failed = false;
    static pinocchio::Model model;
    static std::unique_ptr<pinocchio::Data> data;

    if (!pin_initialized && !pin_init_failed)
    {
      try
      {
        std::string urdf_path =
            ament_index_cpp::get_package_share_directory("upperlimb_robot") +
            "/urdf/right_arm_experiment.urdf";

        pinocchio::urdf::buildModel(urdf_path, model);
        data = std::make_unique<pinocchio::Data>(model);

        if (model.nq != 7 || model.nv != 7)
        {
          std::cerr
              << "[runMirrorMode] 当前右臂 Pinocchio 模型自由度不是 7，"
              << "model.nq=" << model.nq
              << " model.nv=" << model.nv
              << std::endl;
          pin_init_failed = true;
        }
        else
        {
          pin_initialized = true;
          std::cout
              << "[runMirrorMode] Pinocchio 初始化成功。"
              << " nq=" << model.nq
              << " nv=" << model.nv
              << std::endl;
        }
      }
      catch (const std::exception &e)
      {
        std::cerr << "[runMirrorMode] Pinocchio 初始化异常: " << e.what() << std::endl;
        pin_init_failed = true;
      }
    }

    if (!pin_initialized || pin_init_failed || !data)
    {
      output.valid = false;
      return;
    }

    // ===================== 第三步：组装左参考 / 右当前状态 =====================
    constexpr double DEG2RAD = M_PI / 180.0;

    const std::array<std::size_t, 7> left_idx = {
        IDX_L1, IDX_L2, IDX_L3, IDX_L4, IDX_L5, IDX_L6, IDX_L7};

    const std::array<std::size_t, 7> right_idx = {
        IDX_R1, IDX_R2, IDX_R3, IDX_R4, IDX_R5, IDX_R6, IDX_R7};

    // ===================== 镜像方向与零位偏置 =====================
    //
    // 如果某个关节镜像方向反了，只改对应项为 -1.0
    //
    std::array<double, 7> mirror_sign = {
        1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

    std::array<double, 7> mirror_offset_deg = {
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    Eigen::VectorXd q_ref_raw = Eigen::VectorXd::Zero(7);
    Eigen::VectorXd dq_ref_raw = Eigen::VectorXd::Zero(7);
    Eigen::VectorXd q_r = Eigen::VectorXd::Zero(7);
    Eigen::VectorXd dq_r_raw = Eigen::VectorXd::Zero(7);

    for (int i = 0; i < 7; ++i)
    {
      const double q_left_deg = state.position[left_idx[i]];
      const double dq_left_deg = state.velocity[left_idx[i]];
      const double q_right_deg = state.position[right_idx[i]];
      const double dq_right_deg = state.velocity[right_idx[i]];

      const double q_ref_deg =
          mirror_sign[i] * q_left_deg + mirror_offset_deg[i];
      const double dq_ref_deg =
          mirror_sign[i] * dq_left_deg;

      q_ref_raw(i) = q_ref_deg * DEG2RAD;
      dq_ref_raw(i) = dq_ref_deg * DEG2RAD;

      q_r(i) = q_right_deg * DEG2RAD;
      dq_r_raw(i) = dq_right_deg * DEG2RAD;
    }

    // ===================== 第四步：对参考和速度做低通滤波 =====================
    //
    // 左臂参考 q_ref / dq_ref 帧间跳变
    // 右臂速度 dq_r 噪声大
    //
    static bool filter_initialized = false;
    static Eigen::VectorXd q_ref_filt = Eigen::VectorXd::Zero(7);
    static Eigen::VectorXd dq_ref_filt = Eigen::VectorXd::Zero(7);
    static Eigen::VectorXd dq_r_filt = Eigen::VectorXd::Zero(7);
    static Eigen::VectorXd tau_cmd_filt = Eigen::VectorXd::Zero(7);

    if (!filter_initialized)
    {
      q_ref_filt = q_ref_raw;
      dq_ref_filt = dq_ref_raw;
      dq_r_filt = dq_r_raw;
      tau_cmd_filt = Eigen::VectorXd::Zero(7);
      filter_initialized = true;
    }

    // 越大越平滑，不容易抽搐
    const double alpha_q_ref = 0.85;
    const double alpha_dq_ref = 0.90;
    const double alpha_dq_r = 0.90;
    const double alpha_tau = 0.80;

    q_ref_filt = alpha_q_ref * q_ref_filt + (1.0 - alpha_q_ref) * q_ref_raw;
    dq_ref_filt = alpha_dq_ref * dq_ref_filt + (1.0 - alpha_dq_ref) * dq_ref_raw;
    dq_r_filt = alpha_dq_r * dq_r_filt + (1.0 - alpha_dq_r) * dq_r_raw;

    // ===================== 右臂重力补偿 =====================
    const Eigen::VectorXd g =
        pinocchio::computeGeneralizedGravity(model, *data, q_r);

    // ===================== 第六步：关节空间 PD 扭矩跟踪 =====================
    //
    // 这里保留你当前整体负号方向。
    // 但把 Kd 降下来，避免速度噪声导致随机抽搐。
    //
    Eigen::VectorXd Kp = Eigen::VectorXd::Zero(7);
    Eigen::VectorXd Kd = Eigen::VectorXd::Zero(7);

    // 先保持较强跟随，但不过分激进
    Kp << 22.0, 14.0, 28.0, 28.0, 28.0, 23.0, 8.0;
    Kd << 0.5, 0.4, 1.5, 1.5, 1.5, 1, 0.5;

    Eigen::VectorXd tau_cmd_raw =
        -(g +
          Kp.asDiagonal() * (q_ref_filt - q_r) +
          Kd.asDiagonal() * (dq_ref_filt - dq_r_filt));

    // ===================== 第七步：扭矩限幅 =====================
    //
    // 按你前面那份右臂实测日志，给一个更贴近实际的限幅。
    //
    std::array<double, 7> tau_limit = {
        30.0, 8.0, 6.0, 18, 18, 18, 6};

    for (int i = 0; i < 7; ++i)
    {
      if (tau_cmd_raw(i) > tau_limit[i])
      {
        tau_cmd_raw(i) = tau_limit[i];
      }
      else if (tau_cmd_raw(i) < -tau_limit[i])
      {
        tau_cmd_raw(i) = -tau_limit[i];
      }
    }

    // ===================== 第八步：对最终扭矩再做低通滤波 =====================
    //
    // 这一步对“突然抽一下”非常有效。
    //
    tau_cmd_filt = alpha_tau * tau_cmd_filt + (1.0 - alpha_tau) * tau_cmd_raw;

    // 再做一次保护限幅，防止滤波后越界
    for (int i = 0; i < 7; ++i)
    {
      if (tau_cmd_filt(i) > tau_limit[i])
      {
        tau_cmd_filt(i) = tau_limit[i];
      }
      else if (tau_cmd_filt(i) < -tau_limit[i])
      {
        tau_cmd_filt(i) = -tau_limit[i];
      }
    }

    // ===================== 第九步：写入右臂扭矩输出 =====================
    output.use_torque[IDX_R1] = true;
    output.use_torque[IDX_R2] = true;
    output.use_torque[IDX_R3] = true;
    output.use_torque[IDX_R4] = true;
    output.use_torque[IDX_R5] = true;
    output.use_torque[IDX_R6] = true;
    output.use_torque[IDX_R7] = true;

    output.target_torque[IDX_R1] = tau_cmd_filt(0);
    output.target_torque[IDX_R2] = tau_cmd_filt(1);
    output.target_torque[IDX_R3] = tau_cmd_filt(2);
    output.target_torque[IDX_R4] = tau_cmd_filt(3);
    output.target_torque[IDX_R5] = tau_cmd_filt(4);
    output.target_torque[IDX_R6] = tau_cmd_filt(5);
    output.target_torque[IDX_R7] = tau_cmd_filt(6);

    // ===================== 调试输出 =====================
    //
    //
    std::cout
        << "[runMirrorMode] tau_raw(Nm)= ["
        << tau_cmd_raw(0) << ", "
        << tau_cmd_raw(1) << ", "
        << tau_cmd_raw(2) << ", "
        << tau_cmd_raw(3) << ", "
        << tau_cmd_raw(4) << ", "
        << tau_cmd_raw(5) << ", "
        << tau_cmd_raw(6) << "]  tau_filt(Nm)= ["
        << tau_cmd_filt(0) << ", "
        << tau_cmd_filt(1) << ", "
        << tau_cmd_filt(2) << ", "
        << tau_cmd_filt(3) << ", "
        << tau_cmd_filt(4) << ", "
        << tau_cmd_filt(5) << ", "
        << tau_cmd_filt(6) << "]"
        << std::endl;

    // ===================== 保存 tau_cmd 到 CSV =====================
    static std::ofstream csv_file;
    static bool csv_initialized = false;
    static int log_count = 0;

    if (!csv_initialized)
    {
      csv_file.open("mirror_tau_log.csv", std::ios::out | std::ios::app);

      if (csv_file.is_open())
      {
        csv_file << "wall_time,t,"
                 << "tau_raw1,tau_raw2,tau_raw3,tau_raw4,tau_raw5,tau_raw6,tau_raw7,"
                 << "tau_filt1,tau_filt2,tau_filt3,tau_filt4,tau_filt5,tau_filt6,tau_filt7\n";
        csv_initialized = true;
      }
      else
      {
        std::cerr << "[runMirrorMode] 无法打开 mirror_tau_log.csv" << std::endl;
      }
    }

    log_count++;

    if (csv_initialized && (log_count % 4 == 0))
    {
      static auto t0 = std::chrono::steady_clock::now();

      auto now_sys = std::chrono::system_clock::now();
      auto now_steady = std::chrono::steady_clock::now();

      double t =
          std::chrono::duration<double>(now_steady - t0).count();

      auto now_time_t = std::chrono::system_clock::to_time_t(now_sys);
      auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now_sys.time_since_epoch()) %
                1000;

      char time_str[64];
      std::strftime(
          time_str,
          sizeof(time_str),
          "%Y-%m-%d %H:%M:%S",
          std::localtime(&now_time_t));

      csv_file
          << time_str << "."
          << std::setw(3) << std::setfill('0') << ms.count() << ","
          << t << ","
          << tau_cmd_raw(0) << ","
          << tau_cmd_raw(1) << ","
          << tau_cmd_raw(2) << ","
          << tau_cmd_raw(3) << ","
          << tau_cmd_raw(4) << ","
          << tau_cmd_raw(5) << ","
          << tau_cmd_raw(6) << ","
          << tau_cmd_filt(0) << ","
          << tau_cmd_filt(1) << ","
          << tau_cmd_filt(2) << ","
          << tau_cmd_filt(3) << ","
          << tau_cmd_filt(4) << ","
          << tau_cmd_filt(5) << ","
          << tau_cmd_filt(6) << "\n";
    }

    // ===================== 第十步：右臂末端固定 =====================
    output.use_position[IDX_R9] = true;
    output.target_position[IDX_R9] = 0.0;
    output.target_velocity[IDX_R9] = pulsePerMsToJointSpeedDegPerSecMagnitude(9, 9000.0);
    output.target_acc[IDX_R9] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(9, 5.0);
    output.target_dec[IDX_R9] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(9, 0.25);
    output.target_smooth_time[IDX_R9] = 25;

    output.use_position[IDX_R10] = true;
    output.target_position[IDX_R10] = 0.0;
    output.target_velocity[IDX_R10] = pulsePerMsToJointSpeedDegPerSecMagnitude(10, 3000.0);
    output.target_acc[IDX_R10] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(10, 5.0);
    output.target_dec[IDX_R10] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(10, 0.25);
    output.target_smooth_time[IDX_R10] = 25;

    // ===================== 最终输出有效 =====================
    output.valid = true;
  }

  // 左臂主动右臂镜像模式
  void runActiveMirrorMode(const MotorStateCache &state, const SensorStateCache &sensor_state, ControlOutputCache &output)
  {
    // ===================== 输出初始化 =====================
    output = ControlOutputCache{};
    output.valid = false;

    // ===================== 左臂控制层索引 10~16 =====================
    constexpr std::size_t IDX_L1 = 10;
    constexpr std::size_t IDX_L2 = 11;
    constexpr std::size_t IDX_L3 = 12;
    constexpr std::size_t IDX_L4 = 13;
    constexpr std::size_t IDX_L5 = 14;
    constexpr std::size_t IDX_L6 = 15;
    constexpr std::size_t IDX_L7 = 16;

    // ===================== 右臂控制层索引 0~6 =====================
    constexpr std::size_t IDX_R1 = 0;
    constexpr std::size_t IDX_R2 = 1;
    constexpr std::size_t IDX_R3 = 2;
    constexpr std::size_t IDX_R4 = 3;
    constexpr std::size_t IDX_R5 = 4;
    constexpr std::size_t IDX_R6 = 5;
    constexpr std::size_t IDX_R7 = 6;

    // ===================== 右臂末端索引 =====================
    constexpr std::size_t IDX_R9 = 8;
    constexpr std::size_t IDX_R10 = 9;

    // ===================== 左臂主动模式 =====================
    ControlOutputCache left_output{};
    runActiveMode(state, sensor_state, left_output);

    if (left_output.valid)
    {
      for (std::size_t i = 10; i < 20; ++i)
      {
        output.use_position[i] = left_output.use_position[i];
        output.target_position[i] = left_output.target_position[i];
        output.target_velocity[i] = left_output.target_velocity[i];
        output.target_acc[i] = left_output.target_acc[i];
        output.target_dec[i] = left_output.target_dec[i];
        output.target_smooth_time[i] = left_output.target_smooth_time[i];

        output.use_torque[i] = left_output.use_torque[i];
        output.target_torque[i] = left_output.target_torque[i];
      }
    }
    else
    {
      // ===================== 左臂零力失败时的兜底 =====================
      output.use_torque[IDX_L1] = true;
      output.use_torque[IDX_L2] = true;
      output.use_torque[IDX_L3] = true;
      output.use_torque[IDX_L4] = true;
      output.use_torque[IDX_L5] = true;
      output.use_torque[IDX_L6] = true;
      output.use_torque[IDX_L7] = true;

      output.target_torque[IDX_L1] = 0.0;
      output.target_torque[IDX_L2] = 0.0;
      output.target_torque[IDX_L3] = 0.0;
      output.target_torque[IDX_L4] = 0.0;
      output.target_torque[IDX_L5] = 0.0;
      output.target_torque[IDX_L6] = 0.0;
      output.target_torque[IDX_L7] = 0.0;

      output.use_position[17] = true;
      output.target_position[17] = 0.0;
      output.target_velocity[17] = pulsePerMsToJointSpeedDegPerSecMagnitude(18, 1500.0);
      output.target_acc[17] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(18, 5.0);
      output.target_dec[17] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(18, 0.25);
      output.target_smooth_time[17] = 25;

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

    // ===================== 第二步：右臂 Pinocchio 懒加载 =====================
    static bool pin_initialized = false;
    static bool pin_init_failed = false;
    static pinocchio::Model model;
    static std::unique_ptr<pinocchio::Data> data;

    if (!pin_initialized && !pin_init_failed)
    {
      try
      {
        std::string urdf_path =
            ament_index_cpp::get_package_share_directory("upperlimb_robot") +
            "/urdf/right_arm_experiment.urdf";

        pinocchio::urdf::buildModel(urdf_path, model);
        data = std::make_unique<pinocchio::Data>(model);

        if (model.nq != 7 || model.nv != 7)
        {
          std::cerr
              << "[runMirrorMode] 当前右臂 Pinocchio 模型自由度不是 7，"
              << "model.nq=" << model.nq
              << " model.nv=" << model.nv
              << std::endl;
          pin_init_failed = true;
        }
        else
        {
          pin_initialized = true;
          std::cout
              << "[runMirrorMode] Pinocchio 初始化成功。"
              << " nq=" << model.nq
              << " nv=" << model.nv
              << std::endl;
        }
      }
      catch (const std::exception &e)
      {
        std::cerr << "[runMirrorMode] Pinocchio 初始化异常: " << e.what() << std::endl;
        pin_init_failed = true;
      }
    }

    if (!pin_initialized || pin_init_failed || !data)
    {
      output.valid = false;
      return;
    }

    // ===================== 第三步：组装左参考 / 右当前状态 =====================
    constexpr double DEG2RAD = M_PI / 180.0;

    const std::array<std::size_t, 7> left_idx = {
        IDX_L1, IDX_L2, IDX_L3, IDX_L4, IDX_L5, IDX_L6, IDX_L7};

    const std::array<std::size_t, 7> right_idx = {
        IDX_R1, IDX_R2, IDX_R3, IDX_R4, IDX_R5, IDX_R6, IDX_R7};

    // ===================== 镜像方向与零位偏置 =====================
    //
    // 如果某个关节镜像方向反了，只改对应项为 -1.0
    //
    std::array<double, 7> mirror_sign = {
        1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

    std::array<double, 7> mirror_offset_deg = {
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    Eigen::VectorXd q_ref_raw = Eigen::VectorXd::Zero(7);
    Eigen::VectorXd dq_ref_raw = Eigen::VectorXd::Zero(7);
    Eigen::VectorXd q_r = Eigen::VectorXd::Zero(7);
    Eigen::VectorXd dq_r_raw = Eigen::VectorXd::Zero(7);

    for (int i = 0; i < 7; ++i)
    {
      const double q_left_deg = state.position[left_idx[i]];
      const double dq_left_deg = state.velocity[left_idx[i]];
      const double q_right_deg = state.position[right_idx[i]];
      const double dq_right_deg = state.velocity[right_idx[i]];

      const double q_ref_deg =
          mirror_sign[i] * q_left_deg + mirror_offset_deg[i];
      const double dq_ref_deg =
          mirror_sign[i] * dq_left_deg;

      q_ref_raw(i) = q_ref_deg * DEG2RAD;
      dq_ref_raw(i) = dq_ref_deg * DEG2RAD;

      q_r(i) = q_right_deg * DEG2RAD;
      dq_r_raw(i) = dq_right_deg * DEG2RAD;
    }

    // ===================== 第四步：对参考和速度做低通滤波 =====================
    //
    // 左臂参考 q_ref / dq_ref 帧间跳变
    // 右臂速度 dq_r 噪声大
    //
    static bool filter_initialized = false;
    static Eigen::VectorXd q_ref_filt = Eigen::VectorXd::Zero(7);
    static Eigen::VectorXd dq_ref_filt = Eigen::VectorXd::Zero(7);
    static Eigen::VectorXd dq_r_filt = Eigen::VectorXd::Zero(7);
    static Eigen::VectorXd tau_cmd_filt = Eigen::VectorXd::Zero(7);

    if (!filter_initialized)
    {
      q_ref_filt = q_ref_raw;
      dq_ref_filt = dq_ref_raw;
      dq_r_filt = dq_r_raw;
      tau_cmd_filt = Eigen::VectorXd::Zero(7);
      filter_initialized = true;
    }

    // 越大越平滑，不容易抽搐
    const double alpha_q_ref = 0.85;
    const double alpha_dq_ref = 0.90;
    const double alpha_dq_r = 0.90;
    const double alpha_tau = 0.80;

    q_ref_filt = alpha_q_ref * q_ref_filt + (1.0 - alpha_q_ref) * q_ref_raw;
    dq_ref_filt = alpha_dq_ref * dq_ref_filt + (1.0 - alpha_dq_ref) * dq_ref_raw;
    dq_r_filt = alpha_dq_r * dq_r_filt + (1.0 - alpha_dq_r) * dq_r_raw;

    // ===================== 右臂重力补偿 =====================
    const Eigen::VectorXd g =
        pinocchio::computeGeneralizedGravity(model, *data, q_r);

    // ===================== 第六步：关节空间 PD 扭矩跟踪 =====================
    //
    // 这里保留你当前整体负号方向。
    // 但把 Kd 降下来，避免速度噪声导致随机抽搐。
    //
    Eigen::VectorXd Kp = Eigen::VectorXd::Zero(7);
    Eigen::VectorXd Kd = Eigen::VectorXd::Zero(7);

    // 先保持较强跟随，但不过分激进
    Kp << 22.0, 14.0, 28.0, 28.0, 28.0, 23.0, 8.0;
    Kd << 0.5, 0.4, 1.5, 1.5, 1.5, 1, 0.5;

    Eigen::VectorXd tau_cmd_raw =
        -(g +
          Kp.asDiagonal() * (q_ref_filt - q_r) +
          Kd.asDiagonal() * (dq_ref_filt - dq_r_filt));

    // ===================== 第七步：扭矩限幅 =====================
    //
    // 按你前面那份右臂实测日志，给一个更贴近实际的限幅。
    //
    std::array<double, 7> tau_limit = {
        30.0, 8.0, 6.0, 18, 18, 18, 6};

    for (int i = 0; i < 7; ++i)
    {
      if (tau_cmd_raw(i) > tau_limit[i])
      {
        tau_cmd_raw(i) = tau_limit[i];
      }
      else if (tau_cmd_raw(i) < -tau_limit[i])
      {
        tau_cmd_raw(i) = -tau_limit[i];
      }
    }

    // ===================== 第八步：对最终扭矩再做低通滤波 =====================
    //
    // 这一步对“突然抽一下”非常有效。
    //
    tau_cmd_filt = alpha_tau * tau_cmd_filt + (1.0 - alpha_tau) * tau_cmd_raw;

    // 再做一次保护限幅，防止滤波后越界
    for (int i = 0; i < 7; ++i)
    {
      if (tau_cmd_filt(i) > tau_limit[i])
      {
        tau_cmd_filt(i) = tau_limit[i];
      }
      else if (tau_cmd_filt(i) < -tau_limit[i])
      {
        tau_cmd_filt(i) = -tau_limit[i];
      }
    }

    // ===================== 第九步：写入右臂扭矩输出 =====================
    output.use_torque[IDX_R1] = true;
    output.use_torque[IDX_R2] = true;
    output.use_torque[IDX_R3] = true;
    output.use_torque[IDX_R4] = true;
    output.use_torque[IDX_R5] = true;
    output.use_torque[IDX_R6] = true;
    output.use_torque[IDX_R7] = true;

    output.target_torque[IDX_R1] = tau_cmd_filt(0);
    output.target_torque[IDX_R2] = tau_cmd_filt(1);
    output.target_torque[IDX_R3] = tau_cmd_filt(2);
    output.target_torque[IDX_R4] = tau_cmd_filt(3);
    output.target_torque[IDX_R5] = tau_cmd_filt(4);
    output.target_torque[IDX_R6] = tau_cmd_filt(5);
    output.target_torque[IDX_R7] = tau_cmd_filt(6);

    // ===================== 调试输出 =====================
    //
    //
    std::cout
        << "[runMirrorMode] tau_raw(Nm)= ["
        << tau_cmd_raw(0) << ", "
        << tau_cmd_raw(1) << ", "
        << tau_cmd_raw(2) << ", "
        << tau_cmd_raw(3) << ", "
        << tau_cmd_raw(4) << ", "
        << tau_cmd_raw(5) << ", "
        << tau_cmd_raw(6) << "]  tau_filt(Nm)= ["
        << tau_cmd_filt(0) << ", "
        << tau_cmd_filt(1) << ", "
        << tau_cmd_filt(2) << ", "
        << tau_cmd_filt(3) << ", "
        << tau_cmd_filt(4) << ", "
        << tau_cmd_filt(5) << ", "
        << tau_cmd_filt(6) << "]"
        << std::endl;

    // ===================== 保存 tau_cmd 到 CSV =====================
    static std::ofstream csv_file;
    static bool csv_initialized = false;
    static int log_count = 0;

    if (!csv_initialized)
    {
      csv_file.open("mirror_tau_log.csv", std::ios::out | std::ios::app);

      if (csv_file.is_open())
      {
        csv_file << "wall_time,t,"
                 << "tau_raw1,tau_raw2,tau_raw3,tau_raw4,tau_raw5,tau_raw6,tau_raw7,"
                 << "tau_filt1,tau_filt2,tau_filt3,tau_filt4,tau_filt5,tau_filt6,tau_filt7\n";
        csv_initialized = true;
      }
      else
      {
        std::cerr << "[runMirrorMode] 无法打开 mirror_tau_log.csv" << std::endl;
      }
    }

    log_count++;

    if (csv_initialized && (log_count % 4 == 0))
    {
      static auto t0 = std::chrono::steady_clock::now();

      auto now_sys = std::chrono::system_clock::now();
      auto now_steady = std::chrono::steady_clock::now();

      double t =
          std::chrono::duration<double>(now_steady - t0).count();

      auto now_time_t = std::chrono::system_clock::to_time_t(now_sys);
      auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now_sys.time_since_epoch()) %
                1000;

      char time_str[64];
      std::strftime(
          time_str,
          sizeof(time_str),
          "%Y-%m-%d %H:%M:%S",
          std::localtime(&now_time_t));

      csv_file
          << time_str << "."
          << std::setw(3) << std::setfill('0') << ms.count() << ","
          << t << ","
          << tau_cmd_raw(0) << ","
          << tau_cmd_raw(1) << ","
          << tau_cmd_raw(2) << ","
          << tau_cmd_raw(3) << ","
          << tau_cmd_raw(4) << ","
          << tau_cmd_raw(5) << ","
          << tau_cmd_raw(6) << ","
          << tau_cmd_filt(0) << ","
          << tau_cmd_filt(1) << ","
          << tau_cmd_filt(2) << ","
          << tau_cmd_filt(3) << ","
          << tau_cmd_filt(4) << ","
          << tau_cmd_filt(5) << ","
          << tau_cmd_filt(6) << "\n";
    }

    // ===================== 第十步：右臂末端固定 =====================
    output.use_position[IDX_R9] = true;
    output.target_position[IDX_R9] = 0.0;
    output.target_velocity[IDX_R9] = pulsePerMsToJointSpeedDegPerSecMagnitude(9, 9000.0);
    output.target_acc[IDX_R9] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(9, 5.0);
    output.target_dec[IDX_R9] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(9, 0.25);
    output.target_smooth_time[IDX_R9] = 25;

    output.use_position[IDX_R10] = true;
    output.target_position[IDX_R10] = 0.0;
    output.target_velocity[IDX_R10] = pulsePerMsToJointSpeedDegPerSecMagnitude(10, 3000.0);
    output.target_acc[IDX_R10] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(10, 5.0);
    output.target_dec[IDX_R10] = pulsePerMs2ToJointAccelDegPerSec2Magnitude(10, 0.25);
    output.target_smooth_time[IDX_R10] = 25;

    // ===================== 最终输出有效 =====================
    output.valid = true;
  }

} // namespace upperlimb_robot

//