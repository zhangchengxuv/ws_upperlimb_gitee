// Copyright 2024 Zhang Chenxu
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//  * 控制算法节点，负责实现各控制模式的算法逻辑
//  * 从 motor_driver_node 订阅系统状态，确保在系统准备就绪后才允许控制算法开始运行
//  * 从 GUI 订阅控制命令，执行对应的控制算法
//  * 同时向 GUI 发布当前机器人状态（位置、速度、扭矩等）

// 1. 包含头文件
#include "rclcpp/rclcpp.hpp"
#include "upperlimb_robot/msg/control_command.hpp"
#include "upperlimb_robot/msg/robot_state.hpp"
#include "upperlimb_robot/msg/system_state.hpp"
#include "upperlimb_robot/control_algorithms.hpp"
#include "upperlimb_robot/gxn.h"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <thread>

using namespace std::chrono_literals;

// 3. 自定义节点类
class ControlAlgorithmNode : public rclcpp::Node
{
public:
    ControlAlgorithmNode() : Node("control_algorithm_node")
    {
        RCLCPP_INFO(this->get_logger(), "控制算法节点启动");

        // ===================== ROS2 通信 =====================
        control_command_sub_ =
            this->create_subscription<upperlimb_robot::msg::ControlCommand>(
                "control_command",
                10,
                std::bind(&ControlAlgorithmNode::controlCommandCallback, this, std::placeholders::_1));

        system_state_sub_ =
            this->create_subscription<upperlimb_robot::msg::SystemState>(
                "system_state",
                10,
                std::bind(&ControlAlgorithmNode::systemStateCallback, this, std::placeholders::_1));


        // ===================== 启动线程 =====================
        running_.store(true);

        control_thread_ = std::thread(&ControlAlgorithmNode::controlLoop, this);
        RCLCPP_INFO(this->get_logger(), "控制线程已启动");
    }

    ~ControlAlgorithmNode() override
    {
        RCLCPP_INFO(this->get_logger(), "控制算法节点准备退出...");

        running_.store(false);

        RCLCPP_INFO(this->get_logger(), "状态采集线程已安全退出");

        if (control_thread_.joinable())
        {
            control_thread_.join();
        }
        RCLCPP_INFO(this->get_logger(), "控制算法线程已安全退出");
    }

private:
    // ===================== 控制命令回调 =====================
    void controlCommandCallback(const upperlimb_robot::msg::ControlCommand::SharedPtr msg)
    {
        // 假设 ControlCommand.msg 中有字段：int32 command
        current_mode_.store(msg->command);

        RCLCPP_INFO(this->get_logger(), "收到控制命令：%d", msg->command);
    }

    // ===================== 系统状态回调 =====================
    void systemStateCallback(const upperlimb_robot::msg::SystemState::SharedPtr msg)
    {
        // 缓存系统状态
        joint_init_.store(msg->joint_init);
        controller_init_.store(msg->controller_init);
        system_ready_.store(msg->system_ready);

        if (msg->system_ready)
        {
            RCLCPP_INFO(this->get_logger(), "系统初始化完成，允许启动控制算法");
        }
    }

    // ===================== 高频控制线程 =====================
    void controlLoop()
    {
        // 目标控制周期：2ms（500Hz）
        constexpr auto period = std::chrono::microseconds(2000);

        while (running_.load())
        {
            auto cycle_start = std::chrono::steady_clock::now();

            // 1. 系统未初始化完成，不允许进入控制
            if (!system_ready_.load())
            {
                std::this_thread::sleep_for(period);
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "未初始化完成，控制等待中");
                continue;
            }

            // 2. 没有收到状态前不做控制
            if (!state_ready_.load())
            {
                std::this_thread::sleep_for(period);
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "未收到GUI指令，控制等待中");
                continue;
            }

            // local_state 是本次循环使用的局部状态变量
            // 用于存储当前读取到的所有电机信息（位置、速度、扭矩、使能状态）

            // MotorStateCache local_state;
            // local_state.position[i] = ...
            // local_state.velocity[i] = ...
            // local_state.torque[i]   = ...
            MotorStateCache local_state;
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                local_state = latest_motor_state_;
            }

            const int mode = current_mode_.load();

            switch (mode)
            {
            case 0:
                runIdle(local_state);
                break;

            case 1:
                runPassiveTraining(local_state);
                break;

            case 3:
                runZeroForceMode(local_state);
                break;

            case 4:
                runBilateralMode(local_state);
                break;

            case 5:
                runSlidingModeControl(local_state);
                break;

            case 7:
                runLeftPresetPosition(local_state);
                break;

            default:
                runIdle(local_state);
                break;
            }

            auto cycle_end = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(cycle_end - cycle_start);

            if (elapsed < period)
            {
                std::this_thread::sleep_for(period - elapsed);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "控制循环超时");
            }
        }
    }


private:
    // ===================== ROS2 通信 =====================
    rclcpp::Subscription<upperlimb_robot::msg::ControlCommand>::SharedPtr control_command_sub_;
    rclcpp::Subscription<upperlimb_robot::msg::SystemState>::SharedPtr system_state_sub_;

    // ===================== 线程 =====================
    std::thread control_thread_;    // 高频控制线程

    // ===================== 线程控制 =====================
    std::atomic<bool> running_{false};
    std::atomic<bool> state_ready_{false};

    // 系统初始化状态
    std::atomic<bool> joint_init_{false};
    std::atomic<bool> controller_init_{false};
    std::atomic<bool> system_ready_{false};

    // 当前模式，通过 control_command 控制
    std::atomic<int> current_mode_{0};

    // ===================== 状态缓存 =====================
    std::mutex state_mutex_;
    MotorStateCache latest_motor_state_;

    // ===================== 发布分频 =====================
    // 例如采样 500Hz，发布 50Hz，则每 10 次发布 1 次
    int publish_divider_ = 10;
    int publish_divider_count_ = 0;

    // ===================== 控制器参数 =====================
    int core_id_ = 1;

    // ===================== 轴配置 =====================
    static constexpr std::size_t AXIS_NUM = 20;
    std::array<int, AXIS_NUM> axis_ids_ = {
        1, 2, 3, 4, 5,
        6, 7, 8, 9, 10,
        11, 12, 13, 14, 15,
        16, 17, 18, 19, 20};
};

int main(int argc, char const *argv[])
{
    // 2. 初始化 ROS2 客户端
    rclcpp::init(argc, argv);

    // 4. 调用 spin 函数，并传入节点对象指针
    rclcpp::spin(std::make_shared<ControlAlgorithmNode>());

    // 5. 资源释放
    rclcpp::shutdown();
    return 0;
}