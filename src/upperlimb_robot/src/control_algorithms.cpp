#include "upperlimb_robot/control_algorithms.hpp"

namespace upperlimb_robot
{

  namespace
  {

    // 每度对应的脉冲数
    constexpr double PerAngle00 = 524288.0 / 360.0;
    constexpr double PerAngle01 = 262144.0 / 360.0;
    constexpr double PerAngle02 = 2048.0 / 360.0;

    // 左侧零位偏置角，单位：deg
    constexpr double SZO10 = 249.46;
    constexpr double SZO20 = 12172.4808;
    constexpr double SZO30 = -12.1935;
    constexpr double SZO40 = -1354.3904;
    constexpr double SZO50 = 23313.27;
    constexpr double SZO60 = -728.71;

    // 原来 axis 17 的目标是 45 deg，现吸收到零位偏置中
    constexpr double SZO70 = 4380.79;

    constexpr double SZO80 = -23.2018;
    constexpr double SZO90 = -23.2018;
    constexpr double SZO100 = -833.12;

    // 右侧零位偏置角，单位：deg
    constexpr double SZRO10 = -1168.8423;
    constexpr double SZRO20 = -100.2187;
    constexpr double SZRO30 = 1009.6985;
    constexpr double SZRO40 = -157.2542;
    constexpr double SZRO50 = 76.6393;
    constexpr double SZRO60 = 1231.8668;

    // 原来 axis 7 的目标是 -15 deg，现吸收到零位偏置中
    constexpr double SZRO70 = 8097.68;

    constexpr double SZRO80 = -447.8906;
    constexpr double SZRO90 = -0.01;

    // 原来 axis 10 的目标是 -12 deg，现吸收到零位偏置中
    constexpr double SZRO100 = 0.02;

    // 关节映射参数
    // per_angle: 电机侧每度对应的脉冲数
    // zero_offset: 电机侧零位偏置角
    // ratio: 电机角 / 关节角
    struct JointParam
    {
      double per_angle;
      double zero_offset;
      double ratio;
    };

    constexpr JointParam joint_map[21] = {
        {},

        // 右侧 1~10
        {PerAngle01, SZRO10, 121.0},  // axis 1
        {PerAngle00, SZRO20, 121.0},  // axis 2
        {PerAngle01, SZRO30, 121.0},  // axis 3
        {PerAngle01, SZRO40, 101.0},  // axis 4
        {PerAngle01, SZRO50, 121.0},  // axis 5
        {PerAngle00, SZRO60, 101.0},  // axis 6
        {PerAngle01, SZRO70, 100.0},  // axis 7
        {PerAngle02, SZRO80, 2048.0}, // axis 8
        {PerAngle01, SZRO90, 100.0},  // axis 9
        {PerAngle00, SZRO100, 100.0}, // axis 10

        // 左侧 11~20
        {PerAngle00, SZO10, 121.0},  // axis 11
        {PerAngle01, SZO20, 121.0},  // axis 12
        {PerAngle01, SZO30, 121.0},  // axis 13
        {PerAngle01, SZO40, 101.0},  // axis 14
        {PerAngle01, SZO50, 121.0},  // axis 15
        {PerAngle00, SZO60, 101.0},  // axis 16
        {PerAngle01, SZO70, 100.0},  // axis 17
        {PerAngle02, SZO80, 2048.0}, // axis 18
        {PerAngle01, SZO90, 100.0},  // axis 19
        {PerAngle01, SZO100, 100.0}  // axis 20
    };

    // 左臂：pulse = (offset + ratio * joint_deg) * per_angle
    inline double leftJointDegToPulse(int axis, double joint_deg)
    {
      if (axis < 11 || axis > 20)
      {
        return 0.0;
      }

      const auto &p = joint_map[axis];
      return (p.zero_offset + p.ratio * joint_deg) * p.per_angle;
    }

    // 左臂：joint_deg = (pulse / per_angle - offset) / ratio
    inline double pulseToLeftJointDeg(int axis, double pulse)
    {
      if (axis < 11 || axis > 20)
      {
        return 0.0;
      }

      const auto &p = joint_map[axis];
      return (pulse / p.per_angle - p.zero_offset) / p.ratio;
    }

    // 右臂：pulse = (offset - ratio * joint_deg) * per_angle
    inline double rightJointDegToPulse(int axis, double joint_deg)
    {
      if (axis < 1 || axis > 10)
      {
        return 0.0;
      }

      const auto &p = joint_map[axis];
      return (p.zero_offset - p.ratio * joint_deg) * p.per_angle;
    }

    // 右臂：joint_deg = (offset - pulse / per_angle) / ratio
    inline double pulseToRightJointDeg(int axis, double pulse)
    {
      if (axis < 1 || axis > 10)
      {
        return 0.0;
      }

      const auto &p = joint_map[axis];
      return (p.zero_offset - pulse / p.per_angle) / p.ratio;
    }

  } // namespace

  // =========================================================左右臂预设位的控制方式=========================================================
  // 左臂预设位
  // 11 12 13 14 15 16 17 19 20
  void runLeftPresetPosition(const MotorStateCache &state, ControlOutputCache &output)
  {
    (void)state;

    output = ControlOutputCache{};
    output.valid = true;

    // axis 11 -> 0 deg
    output.use_position[10] = true;
    output.target_position[10] = leftJointDegToPulse(11, 0.0);
    output.target_velocity[10] = 400.0;
    output.target_acc[10] = 0.5;
    output.target_dec[10] = 0.25;
    output.target_smooth_time[10] = 25;

    // axis 12 -> 0 deg
    output.use_position[11] = true;
    output.target_position[11] = leftJointDegToPulse(12, 0.0);
    output.target_velocity[11] = 500.0;
    output.target_acc[11] = 0.5;
    output.target_dec[11] = 0.25;
    output.target_smooth_time[11] = 25;

    // axis 13 -> 0 deg
    output.use_position[12] = true;
    output.target_position[12] = leftJointDegToPulse(13, 0.0);
    output.target_velocity[12] = 500.0;
    output.target_acc[12] = 0.5;
    output.target_dec[12] = 0.25;
    output.target_smooth_time[12] = 25;

    // axis 14 -> 0 deg
    output.use_position[13] = true;
    output.target_position[13] = leftJointDegToPulse(14, 0.0);
    output.target_velocity[13] = 1500.0;
    output.target_acc[13] = 5.0;
    output.target_dec[13] = 5.0;
    output.target_smooth_time[13] = 25;

    // axis 15 -> 0 deg
    output.use_position[14] = true;
    output.target_position[14] = leftJointDegToPulse(15, 0.0);
    output.target_velocity[14] = 1500.0;
    output.target_acc[14] = 5.0;
    output.target_dec[14] = 5.0;
    output.target_smooth_time[14] = 25;

    // axis 16 -> 0 deg
    output.use_position[15] = true;
    output.target_position[15] = leftJointDegToPulse(16, 0.0);
    output.target_velocity[15] = 1500.0;
    output.target_acc[15] = 5.0;
    output.target_dec[15] = 5.0;
    output.target_smooth_time[15] = 25;

    // axis 17 -> 0 deg
    output.use_position[16] = true;
    output.target_position[16] = leftJointDegToPulse(17, 0.0);
    output.target_velocity[16] = 1500.0;
    output.target_acc[16] = 5.0;
    output.target_dec[16] = 0.25;
    output.target_smooth_time[16] = 25;

    // axis 19 -> 0 deg
    output.use_position[18] = true;
    output.target_position[18] = leftJointDegToPulse(19, 0.0);
    output.target_velocity[18] = 1500.0;
    output.target_acc[18] = 5.0;
    output.target_dec[18] = 0.25;
    output.target_smooth_time[18] = 25;

    // axis 20 -> 0 deg
    output.use_position[19] = true;
    output.target_position[19] = leftJointDegToPulse(20, 0.0);
    output.target_velocity[19] = 1500.0;
    output.target_acc[19] = 5.0;
    output.target_dec[19] = 0.25;
    output.target_smooth_time[19] = 25;
  }

  // 右臂预设位
  // 1 2 3 4 5 6 7 9 10
  void runRightPresetPosition(const MotorStateCache &state, ControlOutputCache &output)
  {
    (void)state;

    output = ControlOutputCache{};
    output.valid = true;

    // axis 1 -> 0 deg
    output.use_position[0] = true;
    output.target_position[0] = rightJointDegToPulse(1, 0.0);
    output.target_velocity[0] = 200.0;
    output.target_acc[0] = 0.5;
    output.target_dec[0] = 0.25;
    output.target_smooth_time[0] = 25;

    // axis 2 -> 0 deg
    output.use_position[1] = true;
    output.target_position[1] = rightJointDegToPulse(2, 0.0);
    output.target_velocity[1] = 500.0;
    output.target_acc[1] = 0.5;
    output.target_dec[1] = 0.25;
    output.target_smooth_time[1] = 25;

    // axis 3 -> 0 deg
    output.use_position[2] = true;
    output.target_position[2] = rightJointDegToPulse(3, 0.0);
    output.target_velocity[2] = 500.0;
    output.target_acc[2] = 0.5;
    output.target_dec[2] = 0.25;
    output.target_smooth_time[2] = 25;

    // axis 4 -> 0 deg
    output.use_position[3] = true;
    output.target_position[3] = rightJointDegToPulse(4, 0.0);
    output.target_velocity[3] = 800.0;
    output.target_acc[3] = 0.5;
    output.target_dec[3] = 0.25;
    output.target_smooth_time[3] = 25;

    // axis 5 -> 0 deg
    output.use_position[4] = true;
    output.target_position[4] = rightJointDegToPulse(5, 0.0);
    output.target_velocity[4] = 600.0;
    output.target_acc[4] = 5.0;
    output.target_dec[4] = 0.25;
    output.target_smooth_time[4] = 25;

    // axis 6 -> 0 deg
    output.use_position[5] = true;
    output.target_position[5] = rightJointDegToPulse(6, 0.0);
    output.target_velocity[5] = 600.0;
    output.target_acc[5] = 5.0;
    output.target_dec[5] = 0.25;
    output.target_smooth_time[5] = 25;

    // axis 7 -> 0 deg
    output.use_position[6] = true;
    output.target_position[6] = rightJointDegToPulse(7, 0.0);
    output.target_velocity[6] = 3600.0;
    output.target_acc[6] = 5.0;
    output.target_dec[6] = 0.25;
    output.target_smooth_time[6] = 25;

    // axis 9 -> 0 deg
    output.use_position[8] = true;
    output.target_position[8] = rightJointDegToPulse(9, 0.0);
    output.target_velocity[8] = 9000.0;
    output.target_acc[8] = 5.0;
    output.target_dec[8] = 0.25;
    output.target_smooth_time[8] = 25;

    // axis 10 -> 0 deg
    output.use_position[9] = true;
    output.target_position[9] = rightJointDegToPulse(10, 0.0);
    output.target_velocity[9] = 3000.0;
    output.target_acc[9] = 5.0;
    output.target_dec[9] = 0.25;
    output.target_smooth_time[9] = 25;
  }
  // ======================================================================================================================================

  // =========================================================空闲模式：不下发任何命令=========================================================
  void runIdle(const MotorStateCache &state, ControlOutputCache &output)
  {
    (void)state;
    output = ControlOutputCache{};
    output.valid = false;
  }
  // ======================================================================================================================================

  // =========================================================主动模式：下发实时控制命令=========================================================
  void runActiveMode(const MotorStateCache &state, ControlOutputCache &output)
  {
    (void)state;
    output = ControlOutputCache{};
    output.valid = false;
  }
  // =======================================================================================================================================

} // namespace upperlimb_robot