#include "upperlimb_robot/control_algorithms.hpp"

#include <cmath>

namespace upperlimb_robot
{

  namespace
  {

    // 每度对应的脉冲数，单位：pulse/deg
    constexpr double PerAngle00 = 524288.0 / 360.0;
    constexpr double PerAngle01 = 262144.0 / 360.0;
    constexpr double PerAngle02 = 2048.0 / 360.0;

    // 角度与脉冲换算说明：
    //
    // 1. per_angle
    //    表示电机侧每 1 度对应的脉冲数，单位为 pulse/deg。
    //
    // 2. zero_offset
    //    表示电机侧零位偏置角，单位为 deg。
    //    它不是脉冲值，而是人为标定得到的参考零位对应的电机角度。
    //
    // 3. ratio
    //    表示传动比/减速比，定义为：
    //    电机角度 / 关节角度
    //
    // 4. 左臂换算公式
    //    pulse = (zero_offset + ratio * joint_deg) * per_angle
    //    joint_deg = (pulse / per_angle - zero_offset) / ratio
    //
    // 5. 右臂换算公式
    //    pulse = (zero_offset - ratio * joint_deg) * per_angle
    //    joint_deg = (zero_offset - pulse / per_angle) / ratio
    //
    // 6. 左右臂公式符号不同，是因为两侧关节安装方向相反。

    // 左臂零位偏置角，单位：deg
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

    // 扭矩原始值 -> Nm 的比例系数
    // 8、18 轴当前没有可用标定系数，先按 0.0 处理
    struct JointParam
    {
      double per_angle;    // pulse / deg
      double zero_offset;  // deg
      double ratio;        // 电机角 / 关节角
      double torque_scale; // Nm / raw
    };

    // 编译期常量的结构体数组，用于存储 20 个关节的参数表
    constexpr JointParam joint_map[21] = {
        {},

        // 右臂 1~10
        // {per_angle，zero_offset，ratio，torque_scale}
        {PerAngle01, SZRO10, 121.0, 61.0 / 1000.0},  // axis 1
        {PerAngle00, SZRO20, 121.0, 61.0 / 1000.0},  // axis 2
        {PerAngle01, SZRO30, 121.0, 43.68 / 1000.0}, // axis 3
        {PerAngle01, SZRO40, 101.0, 49.0 / 1000.0},  // axis 4
        {PerAngle01, SZRO50, 121.0, 43.68 / 1000.0}, // axis 5
        {PerAngle00, SZRO60, 101.0, 36.4 / 1000.0},  // axis 6
        {PerAngle01, SZRO70, 100.0, 18.0 / 1000.0},  // axis 7
        {PerAngle02, SZRO80, 2048.0, 0.0},           // axis 8
        {PerAngle01, SZRO90, 100.0, 3.9 / 1000.0},   // axis 9
        {PerAngle00, SZRO100, 100.0, 3.9 / 1000.0},  // axis 10

        // 左臂 11~20
        // {per_angle，zero_offset，ratio，torque_scale}
        {PerAngle00, SZO10, 121.0, 61.0 / 1000.0},  // axis 11
        {PerAngle01, SZO20, 121.0, 61.0 / 1000.0},  // axis 12
        {PerAngle01, SZO30, 121.0, 43.68 / 1000.0}, // axis 13
        {PerAngle01, SZO40, 101.0, 49.0 / 1000.0},  // axis 14
        {PerAngle01, SZO50, 121.0, 43.68 / 1000.0}, // axis 15
        {PerAngle00, SZO60, 101.0, 36.4 / 1000.0},  // axis 16
        {PerAngle01, SZO70, 100.0, 18.0 / 1000.0},  // axis 17
        {PerAngle02, SZO80, 2048.0, 0.0},           // axis 18
        {PerAngle01, SZO90, 100.0, 3.9 / 1000.0},   // axis 19
        {PerAngle01, SZO100, 100.0, 3.9 / 1000.0}   // axis 20
    };

    // ===================== 轴类型判断 =====================

    // 判断是否为左臂轴（11~20）
    inline bool isLeftAxis(int axis)
    {
      return axis >= 11 && axis <= 20;
    }

    // 判断是否为右臂轴（1~10）
    inline bool isRightAxis(int axis)
    {
      return axis >= 1 && axis <= 10;
    }

    // ===================== 获取关节参数 =====================

    // 根据轴号获取对应参数（编码器分辨率 / 零位 / 减速比 / 扭矩系数）
    inline const JointParam *getJointParam(int axis)
    {
      if (axis < 1 || axis > 20)
      {
        return nullptr;
      }
      // 只是返回地址（8字节），不会有性能问题
      return &joint_map[axis];
    }

    // ===================== 速度 / 加速度（幅值）换算 =====================
    //
    // Trap 模式下：
    // - 方向由“当前位置 → 目标位置”自动决定
    // - 因此这里速度/加速度只需要“大小”，不用考虑正负号
    //

    // pulse/ms → deg/s（只取幅值）
    inline double pulsePerMsToJointSpeedDegPerSecMagnitude(int axis, double pulse_per_ms)
    {
      const auto *p = getJointParam(axis);
      if (p == nullptr)
      {
        return 0.0;
      }

      // pulse/ms → deg/ms → deg/s
      return std::abs(pulse_per_ms) / (p->per_angle * p->ratio) * 1000.0;
    }

    // pulse/ms² → deg/s²（只取幅值）
    inline double pulsePerMs2ToJointAccelDegPerSec2Magnitude(int axis, double pulse_per_ms2)
    {
      const auto *p = getJointParam(axis);
      if (p == nullptr)
      {
        return 0.0;
      }

      // pulse/ms² → deg/ms² → deg/s²
      return std::abs(pulse_per_ms2) / (p->per_angle * p->ratio) * 1000000.0;
    }

  } // namespace

  // ===================== 位置换算 =====================
  //
  // joint_deg <-> pulse
  //
  // 左臂：
  //   pulse = (zero_offset + ratio * joint_deg) * per_angle
  //
  // 右臂：
  //   pulse = (zero_offset - ratio * joint_deg) * per_angle
  //
  // 差异原因：左右安装方向相反
  //

  // 关节角度（deg）→ 控制器脉冲（pulse）
  double jointDegToPulse(int axis, double joint_deg)
  {
    const auto *p = getJointParam(axis);
    if (p == nullptr)
    {
      return 0.0;
    }

    // 左臂正方向
    if (isLeftAxis(axis))
    {
      return (p->zero_offset + p->ratio * joint_deg) * p->per_angle;
    }

    // 右臂方向反号
    return (p->zero_offset - p->ratio * joint_deg) * p->per_angle;
  }

  // 控制器脉冲（pulse）→ 关节角度（deg）
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
  // joint_speed_deg_per_sec <-> pulse/ms
  //

  // 关节速度（deg/s）→ 控制器速度（pulse/ms）
  double jointSpeedDegPerSecToPulsePerMs(int axis, double joint_speed_deg_per_sec)
  {
    const auto *p = getJointParam(axis);
    if (p == nullptr)
    {
      return 0.0;
    }

    // 仅取幅值（Trap模式方向自动处理）
    return std::abs(joint_speed_deg_per_sec) * p->per_angle * p->ratio / 1000.0;
  }

  // 控制器速度（pulse/ms）→ 关节速度（deg/s）
  double pulsePerMsToJointDegPerSec(int axis, double pulse_per_ms)
  {
    const auto *p = getJointParam(axis);
    if (p == nullptr)
    {
      return 0.0;
    }

    // 左臂正方向
    if (isLeftAxis(axis))
    {
      return (pulse_per_ms / p->per_angle) / p->ratio * 1000.0;
    }

    // 右臂方向反号
    return -(pulse_per_ms / p->per_angle) / p->ratio * 1000.0;
  }

  // ===================== 加速度换算 =====================
  //
  // joint_acc_deg_per_sec2 <-> pulse/ms²
  //

  // 关节加速度（deg/s²）→ 控制器加速度（pulse/ms²）
  double jointAccelDegPerSec2ToPulsePerMs2(int axis, double joint_acc_deg_per_sec2)
  {
    const auto *p = getJointParam(axis);
    if (p == nullptr)
    {
      return 0.0;
    }

    // 仅取幅值
    return std::abs(joint_acc_deg_per_sec2) * p->per_angle * p->ratio / 1000000.0;
  }

  // 控制器加速度（pulse/ms²）→ 关节加速度（deg/s²）
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
  // raw 为驱动器原始值（GTN_GetEcatAxisAtlTorque）
  // Nm 为实际物理力矩
  //

  // 原始扭矩值 → Nm
  double torqueRawToNm(int axis, double raw)
  {
    const auto *p = getJointParam(axis);
    if (p == nullptr)
    {
      return 0.0;
    }

    // torque_scale 为经验标定系数
    return raw * p->torque_scale;
  }

  // Nm → 原始扭矩值
  double torqueNmToRaw(int axis, double torque_nm)
  {
    const auto *p = getJointParam(axis);
    if (p == nullptr || p->torque_scale == 0.0)
    {
      return 0.0;
    }

    return torque_nm / p->torque_scale;
  }

  void runIdle(const MotorStateCache &state, ControlOutputCache &output)
  {
    (void)state;
    output = ControlOutputCache{};
    output.valid = false;
  }

  void runActiveMode(const MotorStateCache &state, ControlOutputCache &output)
  {
    (void)state;
    output = ControlOutputCache{};
    output.valid = false;
  }

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