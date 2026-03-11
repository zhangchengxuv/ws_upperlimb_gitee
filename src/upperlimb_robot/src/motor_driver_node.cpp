#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "upperlimb_robot/msg/system_state.hpp"
#include "upperlimb_robot/gxn.h"

#include <array>
#include <chrono>
#include <cstdint>
#include <csignal>
#include <cstdio>
#include <memory>
#include <string>

using namespace std::chrono_literals;

class MotorDriverNode : public rclcpp::Node
{
public:
  MotorDriverNode() : Node("motor_driver_node")
  {
    RCLCPP_INFO(this->get_logger(), "电机驱动节点启动");

    system_pub_ =
        this->create_publisher<upperlimb_robot::msg::SystemState>("system_state", 10);

    state_timer_ = this->create_wall_timer(
        500ms, std::bind(&MotorDriverNode::publish_state, this));

    if (!init_controller())
    {
      RCLCPP_ERROR(this->get_logger(), "控制器初始化失败");
      publish_state();
      return;
    }
    controller_init_ = true;
    publish_state();

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
    shutdown_hardware();
  }

  void shutdown_hardware()
  {
    if (shutdown_done_)
    {
      return;
    }
    shutdown_done_ = true;

    fprintf(stderr, "正在安全关闭机器人系统...\n");
    fflush(stderr);

    const bool axes_ok = stop_all_axes_raw();

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
  bool init_controller()
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

    RCLCPP_INFO(
        this->get_logger(),
        "准备加载 EtherCAT 配置文件: %s",
        config_path.c_str());

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

  bool init_motors()
  {
    short rtn = 0;

    for (std::size_t i = 0; i < AXIS_NUM; ++i)
    {
      const int axis = axis_ids_[i];

      rtn = GTN_GetEcatEncPos(core_id_, axis, &enc_pos_[i]);
      if (rtn != 0)
      {
        RCLCPP_ERROR(
            this->get_logger(),
            "读取轴%d编码器位置失败，错误码=%d",
            axis, rtn);
        return false;
      }

      rtn = GTN_SetPrfPos(core_id_, axis, enc_pos_[i]);
      if (rtn != 0)
      {
        RCLCPP_ERROR(
            this->get_logger(),
            "设置轴%d规划位置失败，错误码=%d",
            axis, rtn);
        return false;
      }

      rtn = GTN_SetEncPos(core_id_, axis, enc_pos_[i]);
      if (rtn != 0)
      {
        RCLCPP_ERROR(
            this->get_logger(),
            "设置轴%d编码器位置失败，错误码=%d",
            axis, rtn);
        return false;
      }

      rtn = GTN_AxisOn(core_id_, axis);
      if (rtn != 0)
      {
        RCLCPP_ERROR(
            this->get_logger(),
            "轴%d使能失败，错误码=%d",
            axis, rtn);
        return false;
      }

      axis_enabled_[i] = true;

      RCLCPP_INFO(
          this->get_logger(),
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

  bool stop_all_axes_raw()
  {
    if (!controller_opened_)
    {
      return true;
    }

    bool all_ok = true;
    int failed_axes = 0;

    for (std::size_t i = 0; i < AXIS_NUM; ++i)
    {
      const int axis = axis_ids_[i];

      if (!axis_enabled_[i])
      {
        continue;
      }

      const uint32_t mask = (1u << (axis - 1));

      short rtn_stop = GTN_Stop(core_id_, static_cast<short>(mask), 0);
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

private:
  rclcpp::Publisher<upperlimb_robot::msg::SystemState>::SharedPtr system_pub_;
  rclcpp::TimerBase::SharedPtr state_timer_;

  bool joint_init_ = false;
  bool controller_init_ = false;
  bool system_ready_ = false;

  bool controller_opened_ = false;
  bool ecat_started_ = false;
  bool shutdown_done_ = false;

  int core_id_ = 1;

  static constexpr std::size_t AXIS_NUM = 20;

  std::array<int, AXIS_NUM> axis_ids_ = {
      1, 2, 3, 4, 5,
      6, 7, 8, 9, 10,
      11, 12, 13, 14, 15,
      16, 17, 18, 19, 20};

  std::array<int32_t, AXIS_NUM> enc_pos_ = {};
  std::array<bool, AXIS_NUM> axis_enabled_ = {};
};

// ===== 全局节点指针，用于信号退出时安全关闭 =====
static std::shared_ptr<MotorDriverNode> g_node = nullptr;

void signal_handler(int signum)
{
  fprintf(stderr, ">>> signal_handler called, signal=%d\n", signum);
  fflush(stderr);

  if (g_node)
  {
    g_node->shutdown_hardware();
  }

  rclcpp::shutdown();
}

int main(int argc, char const *argv[])
{
  rclcpp::init(argc, argv);

  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  g_node = std::make_shared<MotorDriverNode>();

  rclcpp::spin(g_node);

  if (g_node)
  {
    g_node->shutdown_hardware();
    g_node.reset();
  }

  return 0;
}
