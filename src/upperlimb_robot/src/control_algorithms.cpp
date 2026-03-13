#include "upperlimb_robot/control_algorithms.hpp"

namespace upperlimb_robot
{

namespace
{
// ===================== 脉冲比 =====================
constexpr double PerAngle00 = 524288.0 / 360.0;
constexpr double PerAngle01 = 262144.0 / 360.0;
constexpr double PerAngle02 = 2048.0 / 360.0;

// ===================== 竖直零位（SZO） =====================
constexpr double SZO10 = 249.46;
constexpr double SZO20 = 12172.4808;
constexpr double SZO30 = -12.1935;
constexpr double SZO40 = -1354.3904;
constexpr double SZO50 = 23313.27;
constexpr double SZO60 = -728.71;
constexpr double SZO70 = -119.21;
constexpr double SZO90 = -23.2018;
constexpr double SZO100 = -833.12;
}  // namespace

void runIdle(const MotorStateCache & state, ControlOutputCache & output)
{
  (void)state;
  output = ControlOutputCache{};
  output.valid = false;
}

void runActiveMode(const MotorStateCache & state, ControlOutputCache & output)
{
  (void)state;

  // 主动模式暂时只保留框架，不下发控制
  output = ControlOutputCache{};
  output.valid = false;
}

void runLeftPresetPosition(const MotorStateCache & state, ControlOutputCache & output)
{
  (void)state;

  // 先清空输出
  output = ControlOutputCache{};
  output.valid = true;

  // 只控制左侧 9 个轴：
  // 11 12 13 14 15 16 17 19 20
  // 数组下标 = 轴号 - 1

  // axis 11
  output.use_position[10] = true;
  output.target_position[10] = SZO10 * PerAngle00;
  output.target_velocity[10] = 400.0;
  output.target_acc[10] = 0.5;
  output.target_dec[10] = 0.25;
  output.target_smooth_time[10] = 25;

  // axis 12
  output.use_position[11] = true;
  output.target_position[11] = SZO20 * PerAngle01;
  output.target_velocity[11] = 500.0;
  output.target_acc[11] = 0.5;
  output.target_dec[11] = 0.25;
  output.target_smooth_time[11] = 25;

  // axis 13
  output.use_position[12] = true;
  output.target_position[12] = SZO30 * PerAngle01;
  output.target_velocity[12] = 500.0;
  output.target_acc[12] = 0.5;
  output.target_dec[12] = 0.25;
  output.target_smooth_time[12] = 25;

  // axis 14
  output.use_position[13] = true;
  output.target_position[13] = SZO40 * PerAngle01;
  output.target_velocity[13] = 1500.0;
  output.target_acc[13] = 5.0;
  output.target_dec[13] = 5.0;
  output.target_smooth_time[13] = 25;

  // axis 15
  output.use_position[14] = true;
  output.target_position[14] = SZO50 * PerAngle01;
  output.target_velocity[14] = 1500.0;
  output.target_acc[14] = 5.0;
  output.target_dec[14] = 5.0;
  output.target_smooth_time[14] = 25;

  // axis 16
  output.use_position[15] = true;
  output.target_position[15] = SZO60 * PerAngle00;
  output.target_velocity[15] = 1500.0;
  output.target_acc[15] = 5.0;
  output.target_dec[15] = 5.0;
  output.target_smooth_time[15] = 25;

  // axis 17
  output.use_position[16] = true;
  output.target_position[16] = (SZO70 + 45.0 * 100.0) * PerAngle01;
  output.target_velocity[16] = 1500.0;
  output.target_acc[16] = 5.0;
  output.target_dec[16] = 0.25;
  output.target_smooth_time[16] = 25;

  // axis 19
  output.use_position[18] = true;
  output.target_position[18] = SZO90 * PerAngle01;
  output.target_velocity[18] = 1500.0;
  output.target_acc[18] = 5.0;
  output.target_dec[18] = 0.25;
  output.target_smooth_time[18] = 25;

  // axis 20
  output.use_position[19] = true;
  output.target_position[19] = SZO100 * PerAngle01;
  output.target_velocity[19] = 1500.0;
  output.target_acc[19] = 5.0;
  output.target_dec[19] = 0.25;
  output.target_smooth_time[19] = 25;
}

}  // namespace upperlimb_robot