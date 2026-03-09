#include "rclcpp/rclcpp.hpp"
#include "upperlimb_robot/msg/system_state.hpp"
#include "upperlimb_robot/gxn.h"

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>

using namespace std::chrono_literals;

class MotorDriverNode : public rclcpp::Node
{
public:
  MotorDriverNode() : Node("motor_driver_node")
  {
    RCLCPP_INFO(this->get_logger(), "电机驱动节点启动");

    system_pub_ =
        this->create_publisher<upperlimb_robot::msg::SystemState>("system_state", 10);

    // 定时发布系统状态，避免订阅节点错过
    state_timer_ = this->create_wall_timer(
        500ms, std::bind(&MotorDriverNode::publish_state, this));

    // 初始化控制器
    if (!init_controller())
    {
      RCLCPP_ERROR(this->get_logger(), "控制器初始化失败");
      publish_state();
      return;
    }
    controller_init_ = true;
    publish_state();

    // 初始化电机
    if (!init_motors())
    {
      RCLCPP_ERROR(this->get_logger(), "电机初始化失败");
      publish_state();
      return;
    }
    joint_init_ = true;

    system_ready_ = controller_init_ && joint_init_;
    publish_state();

    RCLCPP_INFO(this->get_logger(), "电机初始化完成，系统准备就绪");
  }

  ~MotorDriverNode() override
  {
    RCLCPP_INFO(this->get_logger(), "正在安全关闭机器人系统...");

    stop_all_axes();

    if (ecat_started_)
    {
      const short rtn = GTN_TerminateEcatComm(core_id_);
      if (rtn != 0)
      {
        RCLCPP_ERROR(this->get_logger(),
                     "EtherCAT通信关闭失败，错误码=%d", rtn);
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "EtherCAT通信已关闭");
      }
    }

    if (controller_opened_)
    {
      const short rtn = GTN_Close();
      if (rtn != 0)
      {
        RCLCPP_ERROR(this->get_logger(), "控制器关闭失败，错误码=%d", rtn);
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "控制器已关闭");
      }
    }
  }

private:
  bool init_controller()
  {
    short rtn = GTN_Open();
    if (rtn != 0)
    {
      RCLCPP_ERROR(this->get_logger(), "GTN_Open 打开控制器失败，错误码=%d", rtn);
      return false;
    }
    controller_opened_ = true;

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

  bool init_motors()
  {
    short rtn = 0;

    for (std::size_t i = 0; i < AXIS_NUM; ++i)
    {
      const int axis = axis_ids_[i];

      rtn = GTN_GetEcatEncPos(core_id_, axis, &enc_pos_[i]);
      if (rtn != 0)
      {
        RCLCPP_ERROR(this->get_logger(),
                     "读取轴%d编码器位置失败，错误码=%d",
                     axis, rtn);
        return false;
      }

      rtn = GTN_SetPrfPos(core_id_, axis, enc_pos_[i]);
      if (rtn != 0)
      {
        RCLCPP_ERROR(this->get_logger(),
                     "设置轴%d规划位置失败，错误码=%d",
                     axis, rtn);
        return false;
      }

      rtn = GTN_SetEncPos(core_id_, axis, enc_pos_[i]);
      if (rtn != 0)
      {
        RCLCPP_ERROR(this->get_logger(),
                     "设置轴%d编码器位置失败，错误码=%d",
                     axis, rtn);
        return false;
      }

      rtn = GTN_AxisOn(core_id_, axis);
      if (rtn != 0)
      {
        RCLCPP_ERROR(this->get_logger(),
                     "轴%d使能失败，错误码=%d",
                     axis, rtn);
        return false;
      }

      axis_enabled_[i] = true;

      RCLCPP_INFO(this->get_logger(),
                  "轴%d初始化完成，编码器位置=%d",
                  axis, enc_pos_[i]);
    }

    return true;
  }

  void publish_state()
  {
    upperlimb_robot::msg::SystemState msg;
    msg.joint_init = joint_init_;
    msg.controller_init = controller_init_;
    msg.system_ready = system_ready_;

    system_pub_->publish(msg);
  }

  void stop_all_axes()
  {
    if (!controller_opened_)
    {
      return;
    }

    for (std::size_t i = 0; i < AXIS_NUM; ++i)
    {
      const int axis = axis_ids_[i];

      if (!axis_enabled_[i])
      {
        continue;
      }

      const uint32_t mask = (1u << (axis - 1));

      short rtn = GTN_Stop(core_id_, static_cast<short>(mask), 0);
      if (rtn != 0)
      {
        RCLCPP_ERROR(this->get_logger(),
                     "停止轴%d失败，错误码=%d",
                     axis, rtn);
      }

      rtn = GTN_AxisOff(core_id_, axis);
      if (rtn != 0)
      {
        RCLCPP_ERROR(this->get_logger(),
                     "关闭轴%d使能失败，错误码=%d",
                     axis, rtn);
      }
      else
      {
        axis_enabled_[i] = false;
      }
    }

    RCLCPP_INFO(this->get_logger(), "所有电机已关闭使能");
  }

private:
  rclcpp::Publisher<upperlimb_robot::msg::SystemState>::SharedPtr system_pub_;
  rclcpp::TimerBase::SharedPtr state_timer_;

  // 电机关节是否初始化完成
  bool joint_init_ = false;

  // 控制器是否初始化完成
  bool controller_init_ = false;

  // 整个系统是否准备就绪（控制器 + 电机初始化成功）
  bool system_ready_ = false;

  // -控制器运行状态 

  // 控制器是否已经成功打开（GTN_Open）
  bool controller_opened_ = false;

  // EtherCAT通信是否已经启动
  bool ecat_started_ = false;

  // -控制器参数 

  // 控制器核心ID（固高控制器一般为1）
  int core_id_ = 1;

  // -机器人轴配置 

  // 机器人总轴数
  static constexpr std::size_t AXIS_NUM = 20;

  // 每个轴在控制器中的ID编号
  // 对应 EtherCAT 从站电机轴编号
  std::array<int, AXIS_NUM> axis_ids_ = {
      1, 2, 3, 4, 5,
      6, 7, 8, 9, 10,
      11, 12, 13, 14, 15,
      16, 17, 18, 19, 20};

  // -电机运行数据

  // 编码器当前位置（单位：编码器脉冲）
  // 用于初始化规划位置，使当前位置作为零运动参考
  std::array<int32_t, AXIS_NUM> enc_pos_ = {};

  // 每个轴当前是否已经使能
  // 用于关闭系统时安全停止电机
  std::array<bool, AXIS_NUM> axis_enabled_ = {};
};

int main(int argc, char const *argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MotorDriverNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}