#pragma once

#include "upperlimb_robot/control_algorithms.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/force.hpp>

namespace upperlimb_robot
{
namespace experiment
{

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
  Ready,
  Moving,
  Holding,
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

  double move_speed{0.003};
  double ready_time{1.0};
  double hold_time{2.0};
  double stiffness_ramp_time{1.0};

  double extension_limit{0.015};
  double spring_force_limit{10.0};

  double left_task_force_limit{12.0};
  double left_tracking_error_limit{0.01};
  double right_drift_limit{0.002};

  double dt_min{0.001};
  double dt_max{0.05};
};

// 毕设实验控制库的统一入口。
//
// 本类只负责生成 ControlOutputCache。底层采集、模式切换、力矩发送仍由
// 原有 robot_control_node 完成。
class ExperimentControl
{
public:
  ExperimentControl() = default;

  void run(const MotorStateCache &state,
           const SensorStateCache &sensor_state,
           double dt_s,
           ControlOutputCache &output)
  {
    switch (mode_)
    {
    case ExperimentMode::SpringStiffness:
      runSpringStiffnessExperiment(state, sensor_state, dt_s, output);
      break;

    case ExperimentMode::ConstantTorque:
      runConstantTorqueExperiment(output);
      break;

    case ExperimentMode::Idle:
    default:
      runIdleExperiment(output);
      break;
    }
  }

  // 保留旧接口，兼容已有调用。没有实际 dt 时只执行空闲/恒扭矩测试。
  void run(const MotorStateCache &state,
           const SensorStateCache &sensor_state,
           ControlOutputCache &output)
  {
    (void)state;
    (void)sensor_state;
    runIdleExperiment(output);

    if (mode_ == ExperimentMode::ConstantTorque)
    {
      runConstantTorqueExperiment(output);
    }
  }

  void setExperimentMode(ExperimentMode mode)
  {
    mode_ = mode;
  }

  void setSpringLevel(SpringLevel level)
  {
    spring_level_ = level;
  }

  void start()
  {
    started_ = true;
  }

  void stop()
  {
    started_ = false;
  }

  void reset()
  {
    mode_ = ExperimentMode::Idle;
    spring_level_ = SpringLevel::Off;
    phase_ = TrajectoryPhase::Ready;

    started_ = false;
    initialized_ = false;
    fault_ = false;

    step_index_ = 0;
    phase_time_ = 0.0;
    current_stiffness_ = 0.0;

    initial_left_position_.setZero();
    initial_right_position_.setZero();
    initial_relative_position_.setZero();
    target_left_position_.setZero();
    target_left_velocity_.setZero();

    torque_joint_index_ = 0;
    torque_nm_ = 0.0;
    constant_torque_enabled_ = false;
  }

  void setConstantTorque(std::size_t joint_index, double torque_nm)
  {
    if (joint_index >= AXIS_NUM)
    {
      return;
    }

    torque_joint_index_ = joint_index;
    torque_nm_ = torque_nm;
    constant_torque_enabled_ = true;
  }

  double selectedStiffness() const
  {
    switch (spring_level_)
    {
    case SpringLevel::Low:
      return config_.k_low;
    case SpringLevel::Medium:
      return config_.k_medium;
    case SpringLevel::High:
      return config_.k_high;
    case SpringLevel::Off:
    default:
      return 0.0;
    }
  }

private:
  struct RobotKinematics
  {
    Eigen::VectorXd q;
    Eigen::VectorXd v;
    Eigen::Vector3d p_left{Eigen::Vector3d::Zero()};
    Eigen::Vector3d p_right{Eigen::Vector3d::Zero()};
    Eigen::Vector3d v_left{Eigen::Vector3d::Zero()};
    Eigen::Vector3d v_right{Eigen::Vector3d::Zero()};
    Eigen::MatrixXd J_left_world;
    Eigen::MatrixXd J_right_local;
    Eigen::VectorXd gravity;
    Eigen::Matrix3d right_rotation{Eigen::Matrix3d::Identity()};
  };

  static double clampBetween(double value, double lo, double hi)
  {
    return std::max(lo, std::min(value, hi));
  }

  static double clampAbs(double value, double limit)
  {
    return clampBetween(value, -limit, limit);
  }

  static bool finiteVec3(const Eigen::Vector3d &v)
  {
    return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
  }

  static bool finiteMatrix(const Eigen::MatrixXd &m)
  {
    return m.allFinite();
  }

  static Eigen::Vector3d limitNorm(const Eigen::Vector3d &v, double limit)
  {
    const double n = v.norm();
    if (n > limit && n > 1e-9)
    {
      return v * (limit / n);
    }
    return v;
  }

  double validDt(double dt_s) const
  {
    if (!std::isfinite(dt_s) || dt_s <= 0.0)
    {
      return config_.dt_min;
    }
    return clampBetween(dt_s, config_.dt_min, config_.dt_max);
  }

  bool normalizedAxis(Eigen::Vector3d &axis) const
  {
    axis = config_.direction_world;
    if (!finiteVec3(axis))
    {
      return false;
    }

    const double norm = axis.norm();
    if (norm < 1e-9)
    {
      return false;
    }

    axis /= norm;
    return true;
  }

  bool checkConfig()
  {
    Eigen::Vector3d axis;
    if (!normalizedAxis(axis))
    {
      setSafeFault("[ExperimentControl] direction_world 非法");
      return false;
    }

    if (!(config_.k_low > 0.0 &&
          config_.k_medium > config_.k_low &&
          config_.k_high > config_.k_medium))
    {
      setSafeFault("[ExperimentControl] 刚度配置非法，要求 0 < low < medium < high");
      return false;
    }

    const auto max_step_it =
        std::max_element(config_.step_displacements.begin(),
                         config_.step_displacements.end(),
                         [](double a, double b)
                         {
                           return std::abs(a) < std::abs(b);
                         });

    const double max_step = (max_step_it != config_.step_displacements.end())
                                ? std::abs(*max_step_it)
                                : 0.0;

    if (config_.k_high * max_step > config_.spring_force_limit + 1e-9)
    {
      setSafeFault("[ExperimentControl] 最大刚度乘最大位移超过弹簧力限幅");
      return false;
    }

    if (config_.stiffness_ramp_time <= 1e-6 ||
        config_.move_speed <= 0.0 ||
        config_.left_task_force_limit <= 0.0 ||
        config_.spring_force_limit <= 0.0)
    {
      setSafeFault("[ExperimentControl] 速度、力限幅或刚度渐变时间配置非法");
      return false;
    }

    return true;
  }

  bool initPinocchioOnce()
  {
    if (pin_initialized_)
    {
      return true;
    }
    if (pin_init_failed_)
    {
      return false;
    }

    try
    {
      const std::string urdf_path =
          ament_index_cpp::get_package_share_directory("upperlimb_robot") +
          "/urdf/robot_arms.urdf";

      pinocchio::urdf::buildModel(urdf_path, model_);
      data_ = std::make_unique<pinocchio::Data>(model_);

      left_frame_id_ = model_.getFrameId("ft_sensor_frame");
      right_frame_id_ = model_.getFrameId("ft_rightsensor_frame");

      bool ok = true;
      ok = ok && loadJointIndex("joint1", left_q_id_[0], left_v_id_[0]);
      ok = ok && loadJointIndex("joint2", left_q_id_[1], left_v_id_[1]);
      ok = ok && loadJointIndex("joint3", left_q_id_[2], left_v_id_[2]);
      ok = ok && loadJointIndex("joint4", left_q_id_[3], left_v_id_[3]);
      ok = ok && loadJointIndex("joint5", left_q_id_[4], left_v_id_[4]);
      ok = ok && loadJointIndex("joint6", left_q_id_[5], left_v_id_[5]);
      ok = ok && loadJointIndex("joint7", left_q_id_[6], left_v_id_[6]);

      ok = ok && loadJointIndex("joint11", right_q_id_[0], right_v_id_[0]);
      ok = ok && loadJointIndex("joint12", right_q_id_[1], right_v_id_[1]);
      ok = ok && loadJointIndex("joint13", right_q_id_[2], right_v_id_[2]);
      ok = ok && loadJointIndex("joint14", right_q_id_[3], right_v_id_[3]);
      ok = ok && loadJointIndex("joint15", right_q_id_[4], right_v_id_[4]);
      ok = ok && loadJointIndex("joint16", right_q_id_[5], right_v_id_[5]);
      ok = ok && loadJointIndex("joint17", right_q_id_[6], right_v_id_[6]);

      if (left_frame_id_ >= model_.frames.size())
      {
        std::cerr << "[ExperimentControl] 找不到 frame: ft_sensor_frame" << std::endl;
        ok = false;
      }
      if (right_frame_id_ >= model_.frames.size())
      {
        std::cerr << "[ExperimentControl] 找不到 frame: ft_rightsensor_frame" << std::endl;
        ok = false;
      }
      if (model_.nq != 14 || model_.nv != 14)
      {
        std::cerr << "[ExperimentControl] 双臂模型自由度不是 14, nq="
                  << model_.nq << " nv=" << model_.nv << std::endl;
        ok = false;
      }

      if (!ok)
      {
        pin_init_failed_ = true;
        setSafeFault("[ExperimentControl] Pinocchio 初始化失败");
        return false;
      }

      pin_initialized_ = true;
      std::cout << "[ExperimentControl] Pinocchio 初始化成功" << std::endl;
      return true;
    }
    catch (const std::exception &e)
    {
      std::cerr << "[ExperimentControl] Pinocchio 初始化异常: "
                << e.what() << std::endl;
      pin_init_failed_ = true;
      setSafeFault("[ExperimentControl] Pinocchio 初始化异常");
      return false;
    }
  }

  bool loadJointIndex(const std::string &joint_name, int &q_id, int &v_id)
  {
    const pinocchio::JointIndex jid = model_.getJointId(joint_name);
    if (jid >= model_.joints.size())
    {
      std::cerr << "[ExperimentControl] 找不到关节: "
                << joint_name << std::endl;
      return false;
    }

    q_id = model_.joints[jid].idx_q();
    v_id = model_.joints[jid].idx_v();

    if (q_id < 0 || q_id >= model_.nq || v_id < 0 || v_id >= model_.nv)
    {
      std::cerr << "[ExperimentControl] 关节索引非法: "
                << joint_name
                << " q_id=" << q_id
                << " v_id=" << v_id << std::endl;
      return false;
    }

    return true;
  }

  bool readRobotState(const MotorStateCache &state, RobotKinematics &kin)
  {
    constexpr double DEG2RAD = M_PI / 180.0;

    kin.q = Eigen::VectorXd::Zero(model_.nq);
    kin.v = Eigen::VectorXd::Zero(model_.nv);

    for (std::size_t i = 0; i < 7; ++i)
    {
      kin.q(left_q_id_[i]) = state.position[left_ctrl_id_[i]] * DEG2RAD;
      kin.v(left_v_id_[i]) = state.velocity[left_ctrl_id_[i]] * DEG2RAD;

      kin.q(right_q_id_[i]) = state.position[right_ctrl_id_[i]] * DEG2RAD;
      kin.v(right_v_id_[i]) = state.velocity[right_ctrl_id_[i]] * DEG2RAD;
    }

    if (!kin.q.allFinite() || !kin.v.allFinite())
    {
      setSafeFault("[ExperimentControl] q/v 出现 NaN 或 Inf");
      return false;
    }

    const double q_abs_limit = 5.2;
    const double v_abs_limit = 12.0;

    for (int i = 0; i < kin.q.size(); ++i)
    {
      if (std::abs(kin.q(i)) > q_abs_limit)
      {
        setSafeFault("[ExperimentControl] q 超出软件安全检查范围");
        return false;
      }
    }

    for (int i = 0; i < kin.v.size(); ++i)
    {
      if (std::abs(kin.v(i)) > v_abs_limit)
      {
        setSafeFault("[ExperimentControl] v 超出软件安全检查范围");
        return false;
      }
    }

    pinocchio::forwardKinematics(model_, *data_, kin.q, kin.v);
    pinocchio::updateFramePlacements(model_, *data_);

    kin.p_left = data_->oMf[left_frame_id_].translation();
    kin.p_right = data_->oMf[right_frame_id_].translation();
    kin.right_rotation = data_->oMf[right_frame_id_].rotation();

    const pinocchio::Motion left_velocity =
        pinocchio::getFrameVelocity(model_, *data_, left_frame_id_, pinocchio::WORLD);
    const pinocchio::Motion right_velocity =
        pinocchio::getFrameVelocity(model_, *data_, right_frame_id_, pinocchio::WORLD);

    kin.v_left = left_velocity.linear();
    kin.v_right = right_velocity.linear();

    if (!finiteVec3(kin.p_left) || !finiteVec3(kin.p_right) ||
        !finiteVec3(kin.v_left) || !finiteVec3(kin.v_right))
    {
      setSafeFault("[ExperimentControl] frame 位置或速度非法");
      return false;
    }

    kin.J_left_world = Eigen::MatrixXd::Zero(6, model_.nv);
    kin.J_right_local = Eigen::MatrixXd::Zero(6, model_.nv);

    pinocchio::computeFrameJacobian(
        model_, *data_, kin.q, left_frame_id_, pinocchio::WORLD, kin.J_left_world);

    pinocchio::computeFrameJacobian(
        model_, *data_, kin.q, right_frame_id_, pinocchio::LOCAL, kin.J_right_local);

    kin.gravity = pinocchio::computeGeneralizedGravity(model_, *data_, kin.q);

    if (!finiteMatrix(kin.J_left_world) ||
        !finiteMatrix(kin.J_right_local) ||
        !kin.gravity.allFinite())
    {
      setSafeFault("[ExperimentControl] Jacobian 或重力补偿非法");
      return false;
    }

    return true;
  }

  void initializeExperiment(const RobotKinematics &kin)
  {
    initial_left_position_ = kin.p_left;
    initial_right_position_ = kin.p_right;
    initial_relative_position_ = kin.p_left - kin.p_right;

    target_left_position_ = kin.p_left;
    target_left_velocity_.setZero();

    step_index_ = 0;
    phase_ = TrajectoryPhase::Ready;
    phase_time_ = 0.0;
    current_stiffness_ = 0.0;
    initialized_ = true;

    std::cout << "[ExperimentControl] 刚度实验初始化: p_left="
              << initial_left_position_.transpose()
              << " p_right=" << initial_right_position_.transpose()
              << std::endl;
  }

  void updateTrajectory(double dt, const Eigen::Vector3d &axis_world)
  {
    target_left_velocity_.setZero();

    if (fault_ || !started_ || phase_ == TrajectoryPhase::Finished)
    {
      return;
    }

    phase_time_ += dt;

    if (phase_ == TrajectoryPhase::Ready)
    {
      target_left_position_ = initial_left_position_;
      if (phase_time_ >= config_.ready_time)
      {
        phase_ = TrajectoryPhase::Moving;
        phase_time_ = 0.0;
      }
      return;
    }

    if (phase_ == TrajectoryPhase::Moving)
    {
      const double step_displacement = config_.step_displacements[step_index_];
      const Eigen::Vector3d step_target =
          initial_left_position_ + step_displacement * axis_world;

      const Eigen::Vector3d old_target = target_left_position_;
      const Eigen::Vector3d to_target = step_target - target_left_position_;
      const double distance = to_target.norm();
      const double max_move = config_.move_speed * dt;

      if (distance <= max_move || distance < 1e-6)
      {
        target_left_position_ = step_target;
        phase_ = TrajectoryPhase::Holding;
        phase_time_ = 0.0;
      }
      else
      {
        target_left_position_ += to_target * (max_move / distance);
      }

      target_left_velocity_ = (target_left_position_ - old_target) / dt;
      return;
    }

    if (phase_ == TrajectoryPhase::Holding)
    {
      if (phase_time_ >= config_.hold_time)
      {
        phase_time_ = 0.0;
        ++step_index_;

        if (step_index_ >= config_.step_displacements.size())
        {
          phase_ = TrajectoryPhase::Finished;
          started_ = false;
        }
        else
        {
          phase_ = TrajectoryPhase::Moving;
        }
      }
    }
  }

  void updateStiffness(double dt)
  {
    double target_stiffness = selectedStiffness();
    if (!started_ || fault_ || phase_ == TrajectoryPhase::Finished)
    {
      target_stiffness = 0.0;
    }

    const double max_k_rate =
        config_.k_high / std::max(config_.stiffness_ramp_time, 1e-6);
    const double max_change = max_k_rate * dt;

    if (current_stiffness_ < target_stiffness)
    {
      current_stiffness_ =
          std::min(current_stiffness_ + max_change, target_stiffness);
    }
    else
    {
      current_stiffness_ =
          std::max(current_stiffness_ - max_change, target_stiffness);
    }
  }

  Eigen::VectorXd computeLeftTrackingTorque(
      const RobotKinematics &kin,
      const Eigen::Vector3d &axis_world)
  {
    const Eigen::Vector3d position_error =
        target_left_position_ - kin.p_left;
    const Eigen::Vector3d velocity_error =
        target_left_velocity_ - kin.v_left;

    if (position_error.norm() > config_.left_tracking_error_limit)
    {
      setSafeFault("[ExperimentControl] 左臂任务空间跟踪误差超限");
      return Eigen::VectorXd::Zero(model_.nv);
    }

    const Eigen::Vector3d pos_axis =
        axis_world * position_error.dot(axis_world);
    const Eigen::Vector3d vel_axis =
        axis_world * velocity_error.dot(axis_world);

    const Eigen::Vector3d pos_cross = position_error - pos_axis;
    const Eigen::Vector3d vel_cross = velocity_error - vel_axis;

    Eigen::Vector3d F_left =
        config_.left_kp_x * pos_axis +
        config_.left_kd_x * vel_axis +
        config_.left_kp_yz * pos_cross +
        config_.left_kd_yz * vel_cross;

    F_left = limitNorm(F_left, config_.left_task_force_limit);

    if (!finiteVec3(F_left))
    {
      setSafeFault("[ExperimentControl] 左臂任务力非法");
      return Eigen::VectorXd::Zero(model_.nv);
    }

    pinocchio::Force left_wrench = pinocchio::Force::Zero();
    left_wrench.linear() = F_left;
    left_wrench.angular().setZero();

    return kin.J_left_world.transpose() * left_wrench.toVector();
  }

  Eigen::VectorXd computeRightSpringTorque(
      const RobotKinematics &kin,
      const Eigen::Vector3d &axis_world,
      double &extension,
      double &spring_force_scalar)
  {
    const Eigen::Vector3d relative_position = kin.p_left - kin.p_right;
    const Eigen::Vector3d relative_change =
        relative_position - initial_relative_position_;

    extension = axis_world.dot(relative_change);
    extension = clampAbs(extension, config_.extension_limit);

    spring_force_scalar = -current_stiffness_ * extension;
    spring_force_scalar =
        clampAbs(spring_force_scalar, config_.spring_force_limit);

    if (!std::isfinite(extension) || !std::isfinite(spring_force_scalar))
    {
      setSafeFault("[ExperimentControl] 弹簧形变量或弹簧力非法");
      extension = 0.0;
      spring_force_scalar = 0.0;
      return Eigen::VectorXd::Zero(model_.nv);
    }

    // spring_force_world 是作用在左侧的恢复力；右侧受力方向相反。
    const Eigen::Vector3d F_left_world =
        spring_force_scalar * axis_world;
    const Eigen::Vector3d F_right_world = -F_left_world;

    // 严格沿用旧 runActiveSpring3DMode 已确认的右大臂传感器方向：
    // right sensor X = world Z, Y = -world Y, Z = world X。
    const Eigen::Vector3d F_right_sensor(
        F_right_world.z(),
        -F_right_world.y(),
        F_right_world.x());

    pinocchio::Force right_wrench = pinocchio::Force::Zero();
    right_wrench.linear() = F_right_sensor;
    right_wrench.angular().setZero();

    return kin.J_right_local.transpose() * right_wrench.toVector();
  }

  void writeTorqueOutput(const RobotKinematics &kin,
                         const Eigen::VectorXd &tau_left_track,
                         const Eigen::VectorXd &tau_right_spring,
                         ControlOutputCache &output)
  {
    output = ControlOutputCache{};

    if (!tau_left_track.allFinite() || !tau_right_spring.allFinite())
    {
      setSafeFault("[ExperimentControl] 计算得到的实验力矩非法");
      writeZeroTorqueOutput(output);
      return;
    }

    const std::array<double, 7> torque_limit = {
        24.0, 20.0, 16.0, 12.0, 12.0, 12.0, 8.0};

    for (std::size_t i = 0; i < 7; ++i)
    {
      const int left_v_id = left_v_id_[i];
      const int right_v_id = right_v_id_[i];

      const double tau_left =
          kin.gravity(left_v_id) +
          tau_left_track(left_v_id) -
          config_.joint_damping * kin.v(left_v_id);

      const double tau_right =
          kin.gravity(right_v_id) +
          tau_right_spring(right_v_id) -
          config_.joint_damping * kin.v(right_v_id);

      if (!std::isfinite(tau_left) || !std::isfinite(tau_right))
      {
        setSafeFault("[ExperimentControl] 最终输出力矩非法");
        writeZeroTorqueOutput(output);
        return;
      }

      output.use_torque[left_ctrl_id_[i]] = true;
      output.target_torque[left_ctrl_id_[i]] =
          clampAbs(tau_left, torque_limit[i]);

      // 右臂沿用旧主动弹簧模式中的最终输出符号修正。
      output.use_torque[right_ctrl_id_[i]] = true;
      output.target_torque[right_ctrl_id_[i]] =
          clampAbs(-tau_right, torque_limit[i]);
    }

    output.valid = true;
  }

  void writeZeroTorqueOutput(ControlOutputCache &output)
  {
    output = ControlOutputCache{};
    for (std::size_t i = 0; i < 7; ++i)
    {
      output.use_torque[left_ctrl_id_[i]] = true;
      output.target_torque[left_ctrl_id_[i]] = 0.0;

      output.use_torque[right_ctrl_id_[i]] = true;
      output.target_torque[right_ctrl_id_[i]] = 0.0;
    }
    output.valid = true;
  }

  double measuredRightAxialForce(const SensorStateCache &sensor_state,
                                 const RobotKinematics &kin,
                                 const Eigen::Vector3d &axis_world) const
  {
    const auto &raw = sensor_state.sensor[SENSOR_RIGHT_ARM];
    const auto &bias = sensor_state.bias[SENSOR_RIGHT_ARM];

    const Eigen::Vector3d right_force_sensor(
        raw.fx - bias.fx,
        raw.fy - bias.fy,
        raw.fz - bias.fz);

    if (!finiteVec3(right_force_sensor))
    {
      return 0.0;
    }

    const Eigen::Vector3d right_force_world =
        kin.right_rotation * right_force_sensor;

    if (!finiteVec3(right_force_world))
    {
      return 0.0;
    }

    return axis_world.dot(right_force_world);
  }

  void writeLog(double dt,
                const RobotKinematics &kin,
                double extension,
                double spring_force_scalar,
                double measured_right_axial_force,
                double right_drift)
  {
    if (!csv_initialized_)
    {
      openLogFile();
    }

    if (!csv_initialized_ || !csv_.is_open())
    {
      return;
    }

    const double target_step =
        (step_index_ < config_.step_displacements.size())
            ? config_.step_displacements[step_index_]
            : 0.0;

    csv_ << springLevelName()
         << "," << trajectoryPhaseName()
         << "," << step_index_
         << "," << target_step
         << "," << target_left_position_.x()
         << "," << kin.p_left.x()
         << "," << kin.p_right.x()
         << "," << current_stiffness_
         << "," << extension
         << "," << spring_force_scalar
         << "," << measured_right_axial_force
         << "," << right_drift
         << "," << (fault_ ? 1 : 0)
         << "," << dt
         << "\n";
    csv_.flush();
  }

  void openLogFile()
  {
    auto now_sys = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now_sys);

    char file_time[64];
    std::strftime(
        file_time,
        sizeof(file_time),
        "%Y%m%d_%H%M%S",
        std::localtime(&now_time_t));

    const char *pwd = std::getenv("PWD");
    const std::string csv_dir =
        (pwd != nullptr) ? std::string(pwd) : std::string(".");

    const std::string csv_path =
        csv_dir + "/spring_stiffness_experiment_" + file_time + ".csv";

    csv_.open(csv_path, std::ios::out);
    if (!csv_.is_open())
    {
      std::cerr << "[ExperimentControl] 无法创建 CSV 日志文件: "
                << csv_path << std::endl;
      return;
    }

    csv_ << "spring_level,"
         << "trajectory_phase,"
         << "step_index,"
         << "target_step_displacement,"
         << "left_target_x,"
         << "left_actual_x,"
         << "right_actual_x,"
         << "current_stiffness,"
         << "extension,"
         << "theoretical_spring_force,"
         << "measured_right_axial_force,"
         << "right_drift,"
         << "fault,"
         << "dt\n";

    csv_initialized_ = true;
    std::cout << "[ExperimentControl] CSV 日志文件: "
              << csv_path << std::endl;
  }

  const char *springLevelName() const
  {
    switch (spring_level_)
    {
    case SpringLevel::Low:
      return "low";
    case SpringLevel::Medium:
      return "medium";
    case SpringLevel::High:
      return "high";
    case SpringLevel::Off:
    default:
      return "off";
    }
  }

  const char *trajectoryPhaseName() const
  {
    switch (phase_)
    {
    case TrajectoryPhase::Ready:
      return "ready";
    case TrajectoryPhase::Moving:
      return "moving";
    case TrajectoryPhase::Holding:
      return "holding";
    case TrajectoryPhase::Finished:
    default:
      return "finished";
    }
  }

  void setSafeFault(const char *message)
  {
    if (!fault_)
    {
      std::cerr << message << std::endl;
    }
    fault_ = true;
    started_ = false;
  }

  void runSpringStiffnessExperiment(const MotorStateCache &state,
                                    const SensorStateCache &sensor_state,
                                    double dt_s,
                                    ControlOutputCache &output)
  {
    output = ControlOutputCache{};

    const double dt = validDt(dt_s);
    updateStiffness(dt);

    if (!sensor_state.valid)
    {
      setSafeFault("[ExperimentControl] 传感器数据无效");
      writeZeroTorqueOutput(output);
      return;
    }

    if (!checkConfig() || !initPinocchioOnce())
    {
      updateStiffness(dt);
      writeZeroTorqueOutput(output);
      return;
    }

    Eigen::Vector3d axis_world;
    if (!normalizedAxis(axis_world))
    {
      setSafeFault("[ExperimentControl] direction_world 归一化失败");
      writeZeroTorqueOutput(output);
      return;
    }

    RobotKinematics kin;
    if (!readRobotState(state, kin))
    {
      updateStiffness(dt);
      writeZeroTorqueOutput(output);
      return;
    }

    if (started_ && !initialized_)
    {
      initializeExperiment(kin);
      writeZeroTorqueOutput(output);
      return;
    }

    if (!initialized_)
    {
      writeZeroTorqueOutput(output);
      return;
    }

    const double right_drift =
        (kin.p_right - initial_right_position_).norm();
    if (right_drift > config_.right_drift_limit)
    {
      setSafeFault("[ExperimentControl] 右侧固定端漂移超限");
    }

    updateTrajectory(dt, axis_world);
    updateStiffness(dt);

    double extension = 0.0;
    double spring_force_scalar = 0.0;

    Eigen::VectorXd tau_left_track = Eigen::VectorXd::Zero(model_.nv);
    Eigen::VectorXd tau_right_spring = Eigen::VectorXd::Zero(model_.nv);

    if (!fault_)
    {
      tau_left_track = computeLeftTrackingTorque(kin, axis_world);
      if (!fault_)
      {
        tau_right_spring =
            computeRightSpringTorque(
                kin, axis_world, extension, spring_force_scalar);
      }
    }
    else
    {
      // fault 后只记录实际形变量，新增实验力矩保持为 0，刚度继续平滑下降。
      const Eigen::Vector3d relative_change =
          (kin.p_left - kin.p_right) - initial_relative_position_;
      extension =
          clampAbs(axis_world.dot(relative_change), config_.extension_limit);
      spring_force_scalar = 0.0;
    }

    const double measured_right_force =
        measuredRightAxialForce(sensor_state, kin, axis_world);

    if (fault_)
    {
      writeZeroTorqueOutput(output);
    }
    else
    {
      writeTorqueOutput(kin, tau_left_track, tau_right_spring, output);
    }

    writeLog(dt, kin, extension, spring_force_scalar,
             measured_right_force, right_drift);

    output.spring_debug_valid = initialized_ && !fault_;
    output.spring_debug_mode = 2;
    output.spring_e_x = extension * axis_world.x();
    output.spring_e_y = extension * axis_world.y();
    output.spring_e_z = extension * axis_world.z();
    output.spring_e_norm = std::abs(extension);
    output.spring_f_x = spring_force_scalar * axis_world.x();
    output.spring_f_y = spring_force_scalar * axis_world.y();
    output.spring_f_z = spring_force_scalar * axis_world.z();
    output.spring_f_norm = std::abs(spring_force_scalar);
  }

  void runConstantTorqueExperiment(ControlOutputCache &output)
  {
    output = ControlOutputCache{};
    if (!constant_torque_enabled_)
    {
      output.valid = false;
      return;
    }

    output.target_torque[torque_joint_index_] = torque_nm_;
    output.use_torque[torque_joint_index_] = true;
    output.valid = true;
  }

  void runIdleExperiment(ControlOutputCache &output)
  {
    output = ControlOutputCache{};
    output.valid = false;
  }

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
};

}  // namespace experiment
}  // namespace upperlimb_robot
