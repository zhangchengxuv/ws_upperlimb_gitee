#include "upperlimb_robot/control_algorithms.hpp"

#include <cstdlib>
#include <iostream>
#include <memory>

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
    // ===================== 输出初始化 =====================
    //
    // 这里每次进入都先清空 output，
    // 然后只对需要控制的关节写入扭矩输出。
    //
    output = ControlOutputCache{};
    output.valid = false;

    // ===================== 只做左臂 joint4~joint7 =====================
    //
    // 当前实验设定：
    // 1. URDF 中 joint1~3、joint8~10 已经改成 fixed
    // 2. Pinocchio 模型只剩 4 个可动关节：joint4~joint7
    // 3. 这里只控制左臂对应的 4 个控制层索引
    //
    // 注意：
    // 下面这 4 个控制层索引，必须与你当前左臂真实映射一致。
    //
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
        //
        // joint1~3、joint8~10 改成 fixed ，
        // model 里只剩 joint4~7 这 4 个可动关节
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
        else if (model.nq != 4 || model.nv != 4)
        {
          std::cerr
              << "[runZeroForceMode] 当前 Pinocchio 模型自由度不是 4，"
              << "请确认 joint1~3、joint8~10 已改成 fixed。"
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
    // 注意：
    // 1. state.position / velocity 的单位是 deg / deg/s
    // 2. Pinocchio 这里使用 rad / rad/s
    // 3. 因为 fixed joint 不再出现在模型里，所以 q 只有 4 维：
    //    q = [joint4, joint5, joint6, joint7]
    //
    constexpr double DEG2RAD = M_PI / 180.0;

    Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);

    q(0) = state.position[IDX_J4] * DEG2RAD;
    q(1) = state.position[IDX_J5] * DEG2RAD;
    q(2) = state.position[IDX_J6] * DEG2RAD;
    q(3) = state.position[IDX_J7] * DEG2RAD;

    v(0) = state.velocity[IDX_J4] * DEG2RAD;
    v(1) = state.velocity[IDX_J5] * DEG2RAD;
    v(2) = state.velocity[IDX_J6] * DEG2RAD;
    v(3) = state.velocity[IDX_J7] * DEG2RAD;

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
    // g(q) 的维度是 4，对应 joint4~7。
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
    // nv 是模型的自由度数量，这里是 4，对应 joint4~7。
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
    damping(0) = 0.8;
    damping(1) = 0.8;
    damping(2) = 0.8;
    damping(3) = 0.6;
    // 计算阻尼力矩：tau_damping = -damping * v，永远“反着速度方向”
    // damping.asDiagonal()
    // 把 [0.8, 0.8, 0.8, 0.6] 变成:
    // [0.8, 0,   0,   0  ]
    // [0,   0.8, 0,   0  ]
    // [0,   0,   0.8, 0  ]
    // [0,   0,   0,   0.6]
    // 再乘：* v，就相当于每个元素分别乘以对应的速度：
    // [0.8 * v(0), 0.8 * v(1), 0.8 * v(2), 0.6 * v(3)]
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
    const double joint_torque_limit_j4 = 12.0;
    const double joint_torque_limit_j5 = 12.0;
    const double joint_torque_limit_j6 = 12.0;
    const double joint_torque_limit_j7 = 8.0;

    tau(0) = clamp_abs(tau(0), joint_torque_limit_j4);
    tau(1) = clamp_abs(tau(1), joint_torque_limit_j5);
    tau(2) = clamp_abs(tau(2), joint_torque_limit_j6);
    tau(3) = clamp_abs(tau(3), joint_torque_limit_j7);

    // ===================== 输出到控制层 =====================
    //
    // 这里只对 joint4~7 对应的控制层索引写扭矩。
    // 其他关节默认保持 0，不参与零力控制。
    //
    output.use_torque[IDX_J4] = true;
    output.use_torque[IDX_J5] = true;
    output.use_torque[IDX_J6] = true;
    output.use_torque[IDX_J7] = true;

    output.target_torque[IDX_J4] = tau(0);
    output.target_torque[IDX_J5] = tau(1);
    output.target_torque[IDX_J6] = tau(2);
    output.target_torque[IDX_J7] = tau(3);

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