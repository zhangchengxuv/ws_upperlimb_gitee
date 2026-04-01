// Copyright 2024 Zhang Chenxu
// Licensed under the Apache License, Version 2.0

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "upperlimb_robot/msg/control_command.hpp"
#include "upperlimb_robot/msg/system_state.hpp"
#include "upperlimb_robot/msg/robot_state.hpp"

#include "upperlimb_robot/gxn.h"
#include "upperlimb_robot/control_algorithms.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class RobotControlNode : public rclcpp::Node
{
public:
  RobotControlNode() : Node("robot_control_node")
  {
    RCLCPP_INFO(this->get_logger(), "机器人硬件控制节点启动");

    control_command_sub_ =
        this->create_subscription<upperlimb_robot::msg::ControlCommand>(
            "control_command",
            10,
            std::bind(&RobotControlNode::controlCommandCallback, this, std::placeholders::_1));

    system_pub_ =
        this->create_publisher<upperlimb_robot::msg::SystemState>("system_state", 10);

    robot_state_pub_ =
        this->create_publisher<upperlimb_robot::msg::RobotState>("robot_state", 10);

    joint_state_pub_ =
        this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    state_timer_ = this->create_wall_timer(
        500ms, std::bind(&RobotControlNode::publishSystemState, this));

    if (!initController())
    {
      RCLCPP_ERROR(this->get_logger(), "控制器初始化失败");
      publishSystemState();
      return;
    }
    controller_init_ = true;
    publishSystemState();

    if (!initMotors())
    {
      RCLCPP_ERROR(this->get_logger(), "电机初始化失败");
      publishSystemState();
      return;
    }
    joint_init_ = true;

    system_ready_ = controller_init_ && joint_init_;
    publishSystemState();

    RCLCPP_INFO(this->get_logger(), "硬件初始化完成，系统准备就绪");

    running_.store(true);

    state_read_thread_ = std::thread(&RobotControlNode::stateReadLoop, this);
    RCLCPP_INFO(this->get_logger(), "状态采集线程已启动");

    control_thread_ = std::thread(&RobotControlNode::controlLoop, this);
    RCLCPP_INFO(this->get_logger(), "控制线程已启动");
  }

  ~RobotControlNode() override
  {
    shutdownHardware();
  }

  void shutdownHardware()
  {
    if (shutdown_done_)
    {
      return;
    }
    shutdown_done_ = true;

    running_.store(false);

    if (state_read_thread_.joinable())
    {
      state_read_thread_.join();
    }

    if (control_thread_.joinable())
    {
      control_thread_.join();
    }

    fprintf(stderr, "正在安全关闭机器人系统...\n");
    fflush(stderr);

    clearAllTorqueCommands();
    restoreAllAxesToPositionMode();
    const bool axes_ok = stopAllAxesRaw();

    if (axes_ok)
    {
      fprintf(stderr, "所有电机已安全关闭。\n");
    }
    else
    {
      fprintf(stderr, "部分电机关闭失败，请检查上面的错误信息。\n");
    }
    fflush(stderr);

    if (ecat_started_)
    {
      const short rtn = GTN_TerminateEcatComm(core_id_);
      if (rtn == 0)
      {
        fprintf(stderr, "EtherCAT通信已关闭。\n");
      }
      else
      {
        fprintf(stderr, "EtherCAT通信关闭失败，错误码=%d\n", rtn);
      }
      fflush(stderr);
      ecat_started_ = false;
    }

    if (controller_opened_)
    {
      const short rtn = GTN_Close();
      if (rtn == 0)
      {
        fprintf(stderr, "控制器已关闭。\n");
      }
      else
      {
        fprintf(stderr, "控制器关闭失败，错误码=%d\n", rtn);
      }
      fflush(stderr);
      controller_opened_ = false;
    }
  }

private:
  // ===================== 控制命令回调 =====================
  //
  // 这里仅记录“请求模式”。
  // 真正的模式进入/退出动作统一放在控制线程里的状态机中处理，
  // 这样可以避免在 ROS 回调线程里直接改硬件状态。
  //
  void controlCommandCallback(const upperlimb_robot::msg::ControlCommand::SharedPtr msg)
  {
    current_mode_.store(msg->command);
    RCLCPP_INFO(this->get_logger(), "收到控制命令=%d", msg->command);
  }

  // ===================== 模式状态机更新 =====================
  //
  // current_mode_ : 外部请求的新模式
  // applied_mode_ : 已经完成进入动作的当前模式
  //
  // 每轮控制循环开始时调用一次：
  // 1. 如果模式没变，直接返回
  // 2. 先执行旧模式退出动作
  // 3. 再执行新模式进入动作
  // 4. 最后更新 applied_mode_
  //
  void updateModeStateMachine()
  {
    const int requested_mode = current_mode_.load();

    if (requested_mode == applied_mode_)
    {
      return;
    }

    const int old_mode = applied_mode_;

    onModeExit(old_mode, requested_mode);
    onModeEnter(old_mode, requested_mode);

    applied_mode_ = requested_mode;

    RCLCPP_INFO(
        this->get_logger(),
        "模式切换完成: old=%d -> new=%d",
        old_mode,
        requested_mode);
  }

  // ===================== 模式退出动作 =====================
  //
  // 退出扭矩相关模式时：
  // 1. 先把 PT 清零
  // 2. 再显式切回位置模式
  //
  void onModeExit(int old_mode, int new_mode)
  {
    (void)new_mode;

    switch (old_mode)
    {
    case upperlimb_robot::MODE_ZERO_FORCE:
    case upperlimb_robot::MODE_ACTIVE:
      clearAllTorqueCommands();
      restoreAllAxesToPositionMode();
      break;

    case upperlimb_robot::MODE_LEFT_PRESET_POSITION:
    case upperlimb_robot::MODE_RIGHT_PRESET_POSITION:
    case upperlimb_robot::MODE_IDLE:
    default:
      break;
    }
  }

  // ===================== 模式进入动作 =====================
  //
  // 进入零力模式时，只有在传感器数据有效时才允许记录零偏。
  //
  void onModeEnter(int old_mode, int new_mode)
  {
    (void)old_mode;

    switch (new_mode)
    {
    case upperlimb_robot::MODE_ZERO_FORCE:
    {
      const bool ok = captureForceSensorBias();
      if (ok)
      {
        RCLCPP_INFO(this->get_logger(), "进入零力模式，已记录六维力零偏");
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "进入零力模式失败：当前传感器数据无效，未记录零偏");
      }
      break;
    }

    case upperlimb_robot::MODE_ACTIVE:
      break;

    case upperlimb_robot::MODE_LEFT_PRESET_POSITION:
    case upperlimb_robot::MODE_RIGHT_PRESET_POSITION:
    case upperlimb_robot::MODE_IDLE:
    default:
      break;
    }
  }

  // ===================== 控制器初始化 =====================
  bool initController()
  {
    short rtn = GTN_Open();
    if (rtn != 0)
    {
      RCLCPP_ERROR(this->get_logger(), "GTN_Open 打开控制器失败，错误码=%d", rtn);
      return false;
    }
    controller_opened_ = true;
    RCLCPP_INFO(this->get_logger(), "GTN_Open 成功");

    std::string config_path;
    try
    {
      const std::string pkg_share =
          ament_index_cpp::get_package_share_directory("upperlimb_robot");
      config_path = pkg_share + "/config/Gecat.eni";
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "获取包配置目录失败: %s", e.what());
      return false;
    }

    rtn = GTN_LoadConfig(core_id_, const_cast<char *>(config_path.c_str()));
    if (rtn != 0)
    {
      RCLCPP_ERROR(
          this->get_logger(),
          "GTN_LoadConfig 加载配置文件失败，错误码=%d, 路径=%s",
          rtn, config_path.c_str());
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "GTN_LoadConfig 成功");

    rtn = GTN_InitEcatComm(core_id_);
    if (rtn != 0)
    {
      RCLCPP_ERROR(this->get_logger(), "EtherCAT初始化失败，错误码=%d", rtn);
      return false;
    }

    short ready = 0;
    int retry = 0;
    constexpr int max_retry = 100;

    do
    {
      rtn = GTN_IsEcatReady(core_id_, &ready);
      if (rtn != 0)
      {
        RCLCPP_ERROR(this->get_logger(), "检测EtherCAT状态失败，错误码=%d", rtn);
        return false;
      }

      if (ready == 1)
      {
        break;
      }

      rclcpp::sleep_for(50ms);
      retry++;
    } while (retry < max_retry);

    if (ready != 1)
    {
      RCLCPP_ERROR(this->get_logger(), "EtherCAT启动超时");
      return false;
    }

    rtn = GTN_StartEcatComm(core_id_);
    if (rtn != 0)
    {
      RCLCPP_ERROR(this->get_logger(), "启动EtherCAT通信失败，错误码=%d", rtn);
      return false;
    }

    ecat_started_ = true;
    RCLCPP_INFO(this->get_logger(), "EtherCAT通信启动成功");

    return true;
  }

  // ===================== 电机初始化 =====================
  bool initMotors()
  {
    short rtn = 0;

    for (std::size_t i = 0; i < upperlimb_robot::AXIS_NUM; ++i)
    {
      const int axis = axis_ids_[i];

      rtn = GTN_GetEcatEncPos(core_id_, axis, &enc_pos_[i]);
      if (rtn != 0)
      {
        RCLCPP_ERROR(this->get_logger(), "读取轴%d编码器位置失败，错误码=%d", axis, rtn);
        return false;
      }

      rtn = GTN_SetPrfPos(core_id_, axis, enc_pos_[i]);
      if (rtn != 0)
      {
        RCLCPP_ERROR(this->get_logger(), "设置轴%d规划位置失败，错误码=%d", axis, rtn);
        return false;
      }

      rtn = GTN_SetEncPos(core_id_, axis, enc_pos_[i]);
      if (rtn != 0)
      {
        RCLCPP_ERROR(this->get_logger(), "设置轴%d编码器位置失败，错误码=%d", axis, rtn);
        return false;
      }

      rtn = GTN_AxisOn(core_id_, axis);
      if (rtn != 0)
      {
        RCLCPP_ERROR(this->get_logger(), "轴%d使能失败，错误码=%d", axis, rtn);
        return false;
      }

      axis_enabled_[i] = true;

      RCLCPP_INFO(this->get_logger(), "轴%d初始化完成，编码器位置=%d", axis, enc_pos_[i]);
    }

    return true;
  }

  // ===================== 系统状态发布 =====================
  void publishSystemState()
  {
    upperlimb_robot::msg::SystemState msg;
    msg.joint_init = joint_init_;
    msg.controller_init = controller_init_;
    msg.system_ready = system_ready_;
    system_pub_->publish(msg);
  }

  // ===================== 读取 EtherCAT 原始 float 数据 =====================
  bool readFloatRawData(int offset, double &value)
  {
    int raw = 0;
    const short rtn =
        GTN_GetEcatRawData(core_id_, offset, 4, reinterpret_cast<unsigned char *>(&raw));

    if (rtn != 0)
    {
      value = 0.0;
      return false;
    }

    float temp = 0.0f;
    std::memcpy(&temp, &raw, sizeof(float));
    value = static_cast<double>(temp);
    return true;
  }

  // ===================== 读取单个六维力传感器 =====================
  bool readSingleForceSensor(int base_offset, upperlimb_robot::ForceSensorData &data)
  {
    bool ok = true;

    ok &= readFloatRawData(base_offset + 0, data.fx);
    ok &= readFloatRawData(base_offset + 4, data.fy);
    ok &= readFloatRawData(base_offset + 8, data.fz);
    ok &= readFloatRawData(base_offset + 12, data.mx);
    ok &= readFloatRawData(base_offset + 16, data.my);
    ok &= readFloatRawData(base_offset + 20, data.mz);

    return ok;
  }

  // ===================== 读取全部四个六维力传感器 =====================
  bool readAllForceSensors(upperlimb_robot::SensorStateCache &sensor_state)
  {
    bool ok = true;

    ok &= readSingleForceSensor(8, sensor_state.sensor[upperlimb_robot::SENSOR_RIGHT_HAND]);
    ok &= readSingleForceSensor(40, sensor_state.sensor[upperlimb_robot::SENSOR_RIGHT_ARM]);
    ok &= readSingleForceSensor(72, sensor_state.sensor[upperlimb_robot::SENSOR_LEFT_HAND]);
    ok &= readSingleForceSensor(104, sensor_state.sensor[upperlimb_robot::SENSOR_LEFT_ARM]);

    sensor_state.valid = ok;
    return ok;
  }

  // ===================== 记录当前传感器零偏 =====================
  //
  // 只有在 latest_sensor_state_.valid == true 时，才允许记录零偏。
  // 这样可以避免把未初始化值、读失败值或脏值写进 bias。
  //
  bool captureForceSensorBias()
  {
    std::lock_guard<std::mutex> lock(sensor_mutex_);

    if (!latest_sensor_state_.valid)
    {
      return false;
    }

    latest_sensor_state_.bias = latest_sensor_state_.sensor;
    return true;
  }

  // ===================== 六维力去偏 =====================
  upperlimb_robot::ForceSensorData subtractBias(
      const upperlimb_robot::ForceSensorData &raw,
      const upperlimb_robot::ForceSensorData &bias) const
  {
    upperlimb_robot::ForceSensorData out;
    out.fx = raw.fx - bias.fx;
    out.fy = raw.fy - bias.fy;
    out.fz = raw.fz - bias.fz;
    out.mx = raw.mx - bias.mx;
    out.my = raw.my - bias.my;
    out.mz = raw.mz - bias.mz;
    return out;
  }

  // ===================== 传感器状态转成 24 维数组 =====================
  std::array<double, 24> buildForceSensorArray(
      const upperlimb_robot::SensorStateCache &sensor_state) const
  {
    std::array<double, 24> out{};

    const auto right_hand = subtractBias(
        sensor_state.sensor[upperlimb_robot::SENSOR_RIGHT_HAND],
        sensor_state.bias[upperlimb_robot::SENSOR_RIGHT_HAND]);

    const auto right_arm = subtractBias(
        sensor_state.sensor[upperlimb_robot::SENSOR_RIGHT_ARM],
        sensor_state.bias[upperlimb_robot::SENSOR_RIGHT_ARM]);

    const auto left_hand = subtractBias(
        sensor_state.sensor[upperlimb_robot::SENSOR_LEFT_HAND],
        sensor_state.bias[upperlimb_robot::SENSOR_LEFT_HAND]);

    const auto left_arm = subtractBias(
        sensor_state.sensor[upperlimb_robot::SENSOR_LEFT_ARM],
        sensor_state.bias[upperlimb_robot::SENSOR_LEFT_ARM]);

    std::size_t idx = 0;

    auto fill_one = [&](const upperlimb_robot::ForceSensorData &s)
    {
      out[idx++] = s.fx;
      out[idx++] = s.fy;
      out[idx++] = s.fz;
      out[idx++] = s.mx;
      out[idx++] = s.my;
      out[idx++] = s.mz;
    };

    fill_one(right_hand);
    fill_one(right_arm);
    fill_one(left_hand);
    fill_one(left_arm);

    return out;
  }

  // ===================== 发布 RobotState =====================
  void publishRobotState(
      const upperlimb_robot::MotorStateCache &state,
      const upperlimb_robot::SensorStateCache &sensor_state)
  {
    upperlimb_robot::msg::RobotState msg;

    msg.position.assign(state.position.begin(), state.position.end());
    msg.velocity.assign(state.velocity.begin(), state.velocity.end());
    msg.torque.assign(state.torque.begin(), state.torque.end());
    msg.enabled.assign(state.enable.begin(), state.enable.end());

    const auto force_array = buildForceSensorArray(sensor_state);
    msg.force_sensor.assign(force_array.begin(), force_array.end());

    robot_state_pub_->publish(msg);
    joint_state_pub_->publish(convertToJointStateMsg(state));
  }

  // ===================== 状态采集线程 =====================
  void stateReadLoop()
  {
    std::uint64_t timeout_count = 0;

    while (running_.load())
    {
      const int state_hz = state_hz_.load();
      if (state_hz <= 0)
      {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "state_hz 非法=%d，必须大于 0", state_hz);
        std::this_thread::sleep_for(100ms);
        continue;
      }

      const auto period = std::chrono::microseconds(1000000 / state_hz);
      auto cycle_start = std::chrono::steady_clock::now();

      upperlimb_robot::MotorStateCache local_state;
      upperlimb_robot::SensorStateCache local_sensor_state;

      const bool do_read_status =
          (++status_read_divider_count_ >= status_read_divider_);

      if (do_read_status)
      {
        status_read_divider_count_ = 0;
      }

      for (std::size_t i = 0; i < upperlimb_robot::AXIS_NUM; ++i)
      {
        const int axis = axis_ids_[i];

        short rtn = 0;
        int32_t sts = 0;
        int32_t enc_pos = 0;
        double vel = 0.0;
        short torque_raw = 0;

        if (do_read_status)
        {
          rtn = GTN_GetSts(core_id_, axis, &sts, 1);
          if (rtn == 0)
          {
            local_state.enable[i] = (sts != 0);
          }
          else
          {
            local_state.enable[i] = axis_enabled_[i];
          }
        }
        else
        {
          local_state.enable[i] = axis_enabled_[i];
        }

        rtn = GTN_GetEcatEncPos(core_id_, axis, &enc_pos);
        if (rtn == 0)
        {
          local_state.position[i] =
              upperlimb_robot::pulseToJointDeg(axis, static_cast<double>(enc_pos));
        }
        else
        {
          local_state.position[i] = 0.0;
        }

        rtn = GTN_GetEcatAxisAtlTorque(core_id_, axis, &torque_raw);
        if (rtn == 0)
        {
          local_state.torque[i] =
              upperlimb_robot::torqueRawToNm(axis, static_cast<double>(torque_raw));
        }
        else
        {
          local_state.torque[i] = 0.0;
        }

        rtn = GTN_GetEncVel(core_id_, axis, &vel, 1);
        if (rtn == 0)
        {
          local_state.velocity[i] =
              upperlimb_robot::pulsePerMsToJointDegPerSec(axis, vel);
        }
        else
        {
          local_state.velocity[i] = 0.0;
        }
      }

      {
        std::lock_guard<std::mutex> lock(sensor_mutex_);
        local_sensor_state.bias = latest_sensor_state_.bias;
      }
      readAllForceSensors(local_sensor_state);

      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        latest_motor_state_ = local_state;
        state_ready_.store(true);
      }

      {
        std::lock_guard<std::mutex> lock(sensor_mutex_);
        latest_sensor_state_.sensor = local_sensor_state.sensor;
        latest_sensor_state_.bias = local_sensor_state.bias;
        latest_sensor_state_.valid = local_sensor_state.valid;
      }

      publish_divider_count_++;
      if (publish_divider_count_ >= publish_divider_)
      {
        publish_divider_count_ = 0;
        publishRobotState(local_state, local_sensor_state);
      }

      auto cycle_end = std::chrono::steady_clock::now();
      auto elapsed =
          std::chrono::duration_cast<std::chrono::microseconds>(cycle_end - cycle_start);

      if (elapsed < period)
      {
        std::this_thread::sleep_for(period - elapsed);
      }
      else
      {
        timeout_count++;
        if (timeout_count % 100 == 0)
        {
          double actual_hz = 1000000.0 / static_cast<double>(elapsed.count());

          RCLCPP_WARN(
              this->get_logger(),
              "状态采集循环超时，目标频率=%d Hz，实际频率=%.1f Hz，耗时=%ld us",
              state_hz_.load(),
              actual_hz,
              elapsed.count());
        }
      }
    }
  }

  // ===================== 控制线程 =====================
  void controlLoop()
  {
    std::uint64_t timeout_count = 0;

    while (running_.load())
    {
      const int control_hz = control_hz_.load();
      if (control_hz <= 0)
      {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "control_hz 非法=%d，必须大于 0", control_hz);
        std::this_thread::sleep_for(100ms);
        continue;
      }

      const auto period = std::chrono::microseconds(1000000 / control_hz);
      auto cycle_start = std::chrono::steady_clock::now();

      if (!system_ready_)
      {
        std::this_thread::sleep_for(period);
        continue;
      }

      if (!state_ready_.load())
      {
        std::this_thread::sleep_for(period);
        continue;
      }

      upperlimb_robot::MotorStateCache local_state;
      upperlimb_robot::SensorStateCache local_sensor_state;

      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        local_state = latest_motor_state_;
      }

      {
        std::lock_guard<std::mutex> lock(sensor_mutex_);
        local_sensor_state = latest_sensor_state_;
      }

      // 先更新模式状态机，再根据当前模式计算控制输出
      updateModeStateMachine();

      const int mode = applied_mode_;

      upperlimb_robot::ControlOutputCache output =
          computeControlOutput(mode, local_state, local_sensor_state);

      if (output.valid)
      {
        applyControlOutput(output);
      }
      else
      {
        clearAllTorqueCommands();
        restoreAllAxesToPositionMode();
      }

      auto cycle_end = std::chrono::steady_clock::now();
      auto elapsed =
          std::chrono::duration_cast<std::chrono::microseconds>(cycle_end - cycle_start);

      if (elapsed < period)
      {
        std::this_thread::sleep_for(period - elapsed);
      }
      else
      {
        timeout_count++;
        if (timeout_count % 100 == 0)
        {
          double actual_hz = 1000000.0 / static_cast<double>(elapsed.count());

          RCLCPP_WARN(
              this->get_logger(),
              "控制循环超时，目标频率=%d Hz，实际频率=%.1f Hz，耗时=%ld us",
              control_hz_.load(),
              actual_hz,
              elapsed.count());
        }
      }
    }
  }

  // ===================== 控制算法总入口 =====================
  upperlimb_robot::ControlOutputCache computeControlOutput(
      int mode,
      const upperlimb_robot::MotorStateCache &state,
      const upperlimb_robot::SensorStateCache &sensor_state)
  {
    upperlimb_robot::ControlOutputCache output{};

    switch (mode)
    {
    case upperlimb_robot::MODE_LEFT_PRESET_POSITION:
      upperlimb_robot::runLeftPresetPosition(state, output);
      break;

    case upperlimb_robot::MODE_RIGHT_PRESET_POSITION:
      upperlimb_robot::runRightPresetPosition(state, output);
      break;

    case upperlimb_robot::MODE_ZERO_FORCE:
      upperlimb_robot::runZeroForceMode(state, sensor_state, output);
      break;

    case upperlimb_robot::MODE_ACTIVE:
      upperlimb_robot::runActiveMode(state, sensor_state, output);
      break;

    case upperlimb_robot::MODE_IDLE:
    default:
      upperlimb_robot::runIdle(state, output);
      break;
    }

    return output;
  }

  // ===================== 执行控制输出 =====================
  //
  // 先执行位置输出，再执行扭矩输出。
  // 位置输出阶段会显式确保该轴已经处于位置模式；
  // 扭矩输出阶段会显式确保该轴已经处于扭矩模式。
  //
  void applyControlOutput(const upperlimb_robot::ControlOutputCache &output)
  {
    applyPositionOutput(output);
    applyTorqueOutput(output);
  }

  // ===================== 单轴切到位置模式 =====================
  //
  // 用于修复：
  // 上一轮轴已经切到扭矩模式，但这一轮想发位置命令时，
  // 轴仍然停留在扭矩模式导致位置命令不生效的问题。
  //
  bool ensureAxisInPositionMode(std::size_t i)
  {
    if (i >= upperlimb_robot::AXIS_NUM)
    {
      return false;
    }

    const int axis = axis_ids_[i];

    // 先把 PT 清零，避免残留扭矩
    const short rtn_zero = GTN_SetEcatAxisPT(core_id_, axis, 0);
    if (rtn_zero != 0)
    {
      RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "切回位置模式前清零 PT 失败，轴%d，错误码=%d", axis, rtn_zero);
      return false;
    }

    // 位置模式通常为 8；如果你的驱动实际不是 8，请按设备手册调整
    const short rtn_mode = GTN_SetEcatAxisMode(core_id_, axis, 8);
    if (rtn_mode != 0)
    {
      RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "GTN_SetEcatAxisMode 设置位置模式失败，轴%d，错误码=%d",
          axis, rtn_mode);
      return false;
    }

    axis_in_torque_mode_[i] = false;
    last_target_torque_[i] = 0.0;
    return true;
  }

  // ===================== 单轴切到扭矩模式 =====================
  bool ensureAxisInTorqueMode(std::size_t i)
  {
    if (i >= upperlimb_robot::AXIS_NUM)
    {
      return false;
    }

    const int axis = axis_ids_[i];

    const short rtn_mode = GTN_SetEcatAxisMode(core_id_, axis, 10);
    if (rtn_mode != 0)
    {
      RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "GTN_SetEcatAxisMode 设置扭矩模式失败，轴%d，错误码=%d",
          axis, rtn_mode);
      return false;
    }

    axis_in_torque_mode_[i] = true;
    return true;
  }

  // ===================== 全部轴恢复到位置模式 =====================
  void restoreAllAxesToPositionMode()
  {
    for (std::size_t i = 0; i < upperlimb_robot::AXIS_NUM; ++i)
    {
      if (!axis_in_torque_mode_[i])
      {
        continue;
      }

      (void)ensureAxisInPositionMode(i);
    }
    // ===================== 清除位置命令缓存 =====================
    // 下一次进入位置模式时，即使目标位置与上次一致，也会重新发送位置指令
    for (std::size_t i = 0; i < upperlimb_robot::AXIS_NUM; ++i)
    {
      command_sent_[i] = false;
    }
  }

  // ===================== 执行位置输出 =====================
  //
  // 如果某轴本轮 use_torque = true，则该轴本轮不执行位置控制。
  // 如果某轴上一轮处于扭矩模式，但这一轮要执行位置控制，
  // 则先显式切回位置模式，再发送位置命令。
  //
  void applyPositionOutput(const upperlimb_robot::ControlOutputCache &output)
  {
    long update_mask = 0;

    for (std::size_t i = 0; i < upperlimb_robot::AXIS_NUM; ++i)
    {
      const int axis = axis_ids_[i];

      if (output.use_torque[i])
      {
        continue;
      }

      if (!output.use_position[i])
      {
        command_sent_[i] = false;

        // 如果当前不再使用扭矩控制，而该轴又停留在扭矩模式，
        // 则显式恢复到位置模式，避免后续第一次位置指令不生效。
        if (axis_in_torque_mode_[i])
        {
          (void)ensureAxisInPositionMode(i);
        }
        continue;
      }

      // 如果该轴此前还停留在扭矩模式，本轮要发位置命令前必须先切回位置模式
      if (axis_in_torque_mode_[i])
      {
        if (!ensureAxisInPositionMode(i))
        {
          continue;
        }
      }

      const double target_deg = output.target_position[i];
      const double target_pulse =
          upperlimb_robot::jointDegToPulse(axis, target_deg);

      if (command_sent_[i] &&
          std::abs(target_deg - last_target_position_[i]) < position_epsilon_)
      {
        continue;
      }

      short rtn = 0;

      TTrapPrm trap{};
      trap.acc =
          upperlimb_robot::jointAccelDegPerSec2ToPulsePerMs2(
              axis, output.target_acc[i]);
      trap.dec =
          upperlimb_robot::jointAccelDegPerSec2ToPulsePerMs2(
              axis, output.target_dec[i]);
      trap.smoothTime = output.target_smooth_time[i];
      trap.velStart = 0;

      rtn = GTN_PrfTrap(core_id_, axis);
      if (rtn != 0)
      {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "GTN_PrfTrap 轴%d失败，错误码=%d", axis, rtn);
        continue;
      }

      rtn = GTN_SetTrapPrm(core_id_, axis, &trap);
      if (rtn != 0)
      {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "GTN_SetTrapPrm 轴%d失败，错误码=%d", axis, rtn);
        continue;
      }

      rtn = GTN_SetVel(
          core_id_,
          axis,
          upperlimb_robot::jointSpeedDegPerSecToPulsePerMs(
              axis, output.target_velocity[i]));
      if (rtn != 0)
      {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "GTN_SetVel 轴%d失败，错误码=%d", axis, rtn);
        continue;
      }

      rtn = GTN_SetPos(core_id_, axis, static_cast<int32_t>(target_pulse));
      if (rtn != 0)
      {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "GTN_SetPos 轴%d失败，错误码=%d", axis, rtn);
        continue;
      }

      update_mask |= (1L << (axis - 1));
      last_target_position_[i] = target_deg;
      command_sent_[i] = true;
    }

    if (update_mask != 0)
    {
      const short rtn = GTN_Update(core_id_, update_mask);
      if (rtn != 0)
      {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "GTN_Update 失败，mask=0x%lx，错误码=%d",
            update_mask, rtn);
      }
    }
  }

  // ===================== 执行扭矩输出 =====================
  //
  // 1. use_torque == true 时，确保切到扭矩模式
  // 2. use_torque == false 时，如果该轴还停留在扭矩模式，则清零并切回位置模式
  //
  void applyTorqueOutput(const upperlimb_robot::ControlOutputCache &output)
  {
    for (std::size_t i = 0; i < upperlimb_robot::AXIS_NUM; ++i)
    {
      const int axis = axis_ids_[i];

      if (!output.use_torque[i])
      {
        if (axis_in_torque_mode_[i])
        {
          const short rtn_zero = GTN_SetEcatAxisPT(core_id_, axis, 0);
          if (rtn_zero != 0)
          {
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "GTN_SetEcatAxisPT 清零失败，轴%d，错误码=%d", axis, rtn_zero);
          }

          (void)ensureAxisInPositionMode(i);
        }

        last_target_torque_[i] = 0.0;
        continue;
      }

      const double test_raw = upperlimb_robot::torqueNmToRaw(axis, 1.0);
      if (test_raw == 0.0)
      {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "轴%d 当前没有有效扭矩标定系数，跳过扭矩输出", axis);
        continue;
      }

      const double target_torque_nm = output.target_torque[i];

      if (std::abs(target_torque_nm - last_target_torque_[i]) < torque_epsilon_)
      {
        continue;
      }

      if (!axis_in_torque_mode_[i])
      {
        if (!ensureAxisInTorqueMode(i))
        {
          continue;
        }
      }

      const double torque_raw_double =
          upperlimb_robot::torqueNmToRaw(axis, target_torque_nm);

      const short torque_raw = static_cast<short>(std::lround(torque_raw_double));

      const short rtn_pt = GTN_SetEcatAxisPT(core_id_, axis, torque_raw);
      if (rtn_pt != 0)
      {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "GTN_SetEcatAxisPT 发送扭矩失败，轴%d，目标扭矩=%.4f Nm，raw=%d，错误码=%d",
            axis, target_torque_nm, torque_raw, rtn_pt);
        continue;
      }

      last_target_torque_[i] = target_torque_nm;
    }
  }

  // ===================== 清零全部扭矩命令 =====================
  void clearAllTorqueCommands()
  {
    for (std::size_t i = 0; i < upperlimb_robot::AXIS_NUM; ++i)
    {
      if (!axis_in_torque_mode_[i])
      {
        continue;
      }

      const int axis = axis_ids_[i];
      const short rtn = GTN_SetEcatAxisPT(core_id_, axis, 0);
      if (rtn != 0)
      {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "清零扭矩命令失败，轴%d，错误码=%d", axis, rtn);
      }

      last_target_torque_[i] = 0.0;
    }
  }

  // ===================== 停止全部轴 =====================
  bool stopAllAxesRaw()
  {
    if (!controller_opened_)
    {
      return true;
    }

    bool all_ok = true;
    int failed_axes = 0;

    for (std::size_t i = 0; i < upperlimb_robot::AXIS_NUM; ++i)
    {
      const int axis = axis_ids_[i];

      if (!axis_enabled_[i])
      {
        continue;
      }

      short rtn_stop =
          GTN_Stop(core_id_, static_cast<long>(1L << (axis - 1)), 0);
      if (rtn_stop != 0)
      {
        fprintf(stderr, "停止轴%d失败，错误码=%d\n", axis, rtn_stop);
        fflush(stderr);
        all_ok = false;
        failed_axes++;
      }

      short rtn_off = GTN_AxisOff(core_id_, axis);
      if (rtn_off != 0)
      {
        fprintf(stderr, "关闭轴%d使能失败，错误码=%d\n", axis, rtn_off);
        fflush(stderr);
        all_ok = false;
        failed_axes++;
      }
      else
      {
        axis_enabled_[i] = false;
      }
    }

    if (!all_ok)
    {
      fprintf(stderr, "共有 %d 个轴关闭过程出现错误。\n", failed_axes);
      fflush(stderr);
    }

    return all_ok;
  }

  // ===================== JointState 转换 =====================
  //
  // URDF 定义：
  // joint1  ~ joint10  -> 左臂
  // joint11 ~ joint20  -> 右臂
  //
  // 控制层内部状态：
  // state[10~19] -> 左臂
  // state[0~9]   -> 右臂
  //
  sensor_msgs::msg::JointState convertToJointStateMsg(
      const upperlimb_robot::MotorStateCache &state)
  {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->now();

    msg.name = {
        "joint1", "joint2", "joint3", "joint4", "joint5",
        "joint6", "joint7", "joint8", "joint9", "joint10",
        "joint11", "joint12", "joint13", "joint14", "joint15",
        "joint16", "joint17", "joint18", "joint19", "joint20"};

    msg.position.resize(20);
    msg.velocity.resize(20);
    msg.effort.resize(20);

    constexpr double deg_to_rad = M_PI / 180.0;

    // 左臂：控制层索引 10~19 -> URDF joint1~joint10 -> msg 下标 0~9
    for (std::size_t src = 10, dst = 0; src < 20; ++src, ++dst)
    {
      msg.position[dst] = state.position[src] * deg_to_rad;
      msg.velocity[dst] = state.velocity[src] * deg_to_rad;
      msg.effort[dst] = state.torque[src];
    }

    // 右臂：控制层索引 0~9 -> URDF joint11~joint20 -> msg 下标 10~19
    for (std::size_t src = 0, dst = 10; src < 10; ++src, ++dst)
    {
      msg.position[dst] = state.position[src] * deg_to_rad;
      msg.velocity[dst] = state.velocity[src] * deg_to_rad;
      msg.effort[dst] = state.torque[src];
    }

    return msg;
  }

private:
  rclcpp::Subscription<upperlimb_robot::msg::ControlCommand>::SharedPtr control_command_sub_;
  rclcpp::Publisher<upperlimb_robot::msg::SystemState>::SharedPtr system_pub_;
  rclcpp::Publisher<upperlimb_robot::msg::RobotState>::SharedPtr robot_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr state_timer_;

  bool joint_init_ = false;
  bool controller_init_ = false;
  bool system_ready_ = false;

  bool controller_opened_ = false;
  bool ecat_started_ = false;
  bool shutdown_done_ = false;

  std::atomic<bool> running_{false};
  std::atomic<bool> state_ready_{false};

  std::thread state_read_thread_;
  std::thread control_thread_;

  // 外部请求模式
  std::atomic<int> current_mode_{upperlimb_robot::MODE_IDLE};

  // 已完成进入动作的模式
  int applied_mode_{upperlimb_robot::MODE_IDLE};

  std::atomic<int> state_hz_{200};
  std::atomic<int> control_hz_{90};

  int status_read_divider_ = 20;
  int status_read_divider_count_ = 0;

  std::mutex state_mutex_;
  upperlimb_robot::MotorStateCache latest_motor_state_;

  std::mutex sensor_mutex_;
  upperlimb_robot::SensorStateCache latest_sensor_state_;

  int publish_divider_ = 10;
  int publish_divider_count_ = 0;

  int core_id_ = 1;

  // 控制层索引 0~19 -> GTN 物理轴号 1~20
  std::array<int, upperlimb_robot::AXIS_NUM> axis_ids_ = {
      1, 2, 3, 4, 5,
      6, 7, 8, 9, 10,
      11, 12, 13, 14, 15,
      16, 17, 18, 19, 20};

  std::array<int32_t, upperlimb_robot::AXIS_NUM> enc_pos_{};
  std::array<bool, upperlimb_robot::AXIS_NUM> axis_enabled_{};

  // 上一次发送的位置目标（单位 deg）
  std::array<double, upperlimb_robot::AXIS_NUM> last_target_position_{};

  // 是否已发送过位置命令
  std::array<bool, upperlimb_robot::AXIS_NUM> command_sent_{};

  // 当前轴是否处于扭矩模式
  std::array<bool, upperlimb_robot::AXIS_NUM> axis_in_torque_mode_{};

  // 上一次发送的扭矩目标（单位 Nm）
  std::array<double, upperlimb_robot::AXIS_NUM> last_target_torque_{};

  double position_epsilon_ = 0.05;
  double torque_epsilon_ = 0.01;
};

static std::shared_ptr<RobotControlNode> g_node = nullptr;

void signal_handler(int signum)
{
  fprintf(stderr, "\n收到退出信号(%d)，开始安全关闭...\n", signum);
  fflush(stderr);

  if (g_node)
  {
    g_node->shutdownHardware();
  }

  rclcpp::shutdown();
}

int main(int argc, char const *argv[])
{
  rclcpp::init(argc, argv);

  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  g_node = std::make_shared<RobotControlNode>();

  rclcpp::spin(g_node);

  if (g_node)
  {
    g_node->shutdownHardware();
    g_node.reset();
  }

  return 0;
}