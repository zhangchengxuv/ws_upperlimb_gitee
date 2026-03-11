// 1. 包含头文件
#include "rclcpp/rclcpp.hpp"
#include "upperlimb_robot/msg/control_command.hpp"
#include "upperlimb_robot/msg/robot_state.hpp"
#include "upperlimb_robot/gxn.h"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <thread>

using namespace std::chrono_literals;

// ===================== 电机状态缓存结构体 =====================
struct MotorStateCache
{
    std::array<double, 20> position{}; // 关节位置（可换算成角度）
    std::array<double, 20> velocity{}; // 关节速度
    std::array<double, 20> torque{};   // 关节扭矩
    std::array<bool, 20> enabled{};    // 轴是否使能
};

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

        robot_state_pub_ =
            this->create_publisher<upperlimb_robot::msg::RobotState>(
                "robot_state",
                10);

        // ===================== 启动线程 =====================
        running_.store(true);

        state_read_thread_ = std::thread(&ControlAlgorithmNode::stateReadLoop, this);
        RCLCPP_INFO(this->get_logger(), "状态采集线程已启动");
        control_thread_ = std::thread(&ControlAlgorithmNode::controlLoop, this);
        RCLCPP_INFO(this->get_logger(), "控制线程已启动");
    }

    ~ControlAlgorithmNode() override
    {
        RCLCPP_INFO(this->get_logger(), "控制算法节点准备退出...");

        running_.store(false);

        if (state_read_thread_.joinable())
        {
            state_read_thread_.join();
        }

        if (control_thread_.joinable())
        {
            control_thread_.join();
        }

        RCLCPP_INFO(this->get_logger(), "控制算法节点已安全退出");
    }

private:
    // ===================== 控制命令回调 =====================
    void controlCommandCallback(const upperlimb_robot::msg::ControlCommand::SharedPtr msg)
    {
        // 假设 ControlCommand.msg 中有字段：int32 command
        // 如果你的字段名不是 command，请改成你自己的字段名
        current_mode_.store(msg->command);

        RCLCPP_INFO(this->get_logger(), "收到控制命令：%d", msg->command);
    }

    // ===================== 高频状态采集线程 =====================
    void stateReadLoop()
    {
        // 目标采样周期：2ms（500Hz）
        constexpr auto period = std::chrono::microseconds(2000);

        while (running_.load())
        {
            auto cycle_start = std::chrono::steady_clock::now();

            MotorStateCache local_state;

            // ===== 在这里读取 20 轴位置、速度、扭矩 =====

            for (std::size_t i = 0; i < AXIS_NUM; ++i)
            {
                const int axis = axis_ids_[i];

                // ---------------- 位置 ----------------
                // TODO: 读取接口

                // 这里先写成占位
                local_state.position[i] = 0.0;

                // ---------------- 速度 ----------------
                // TODO: 替换为你的真实读取接口
                // 例如：
                // double vel = 0.0;
                // short rtn = GTN_GetEncVel(core_id_, axis, &vel);
                // if (rtn == 0) { local_state.velocity[i] = vel; }

                local_state.velocity[i] = 0.0;

                // ---------------- 扭矩 ----------------
                // TODO: 替换为你的真实读取接口
                // 例如：
                // short torque_raw = 0;
                // short rtn = GTN_GetEcatAxisAtlTorque(core_id_, axis, &torque_raw);
                // if (rtn == 0) { local_state.torque[i] = static_cast<double>(torque_raw); }

                local_state.torque[i] = 0.0;

                // ---------------- 使能状态 ----------------
                // TODO: 这里后面可以接你的轴使能状态
                local_state.enabled[i] = true;
            }

            // 用互斥锁更新共享状态
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                latest_motor_state_ = local_state;
                state_ready_.store(true);
            }

            // ===== 低频向 GUI 发布 RobotState =====
            // 采样 500Hz，发布 50Hz：每 10 次发 1 次
            publish_divider_count_++;
            if (publish_divider_count_ >= publish_divider_)
            {
                publish_divider_count_ = 0;
                publishRobotState(local_state);
            }

            // 周期等待
            auto cycle_end = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(cycle_end - cycle_start);
            if (elapsed < period)
            {
                std::this_thread::sleep_for(period - elapsed);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "状态采集循环超时");
            }
        }
    }

    // ===================== 高频控制线程 =====================
    void controlLoop()
    {
        // 目标控制周期：2ms（500Hz）
        constexpr auto period = std::chrono::microseconds(2000);

        while (running_.load())
        {
            // 记录周期开始时间
            auto cycle_start = std::chrono::steady_clock::now();

            // 没有收到状态前不做控制
            if (!state_ready_.load())
            {
                std::this_thread::sleep_for(period);
                continue;
            }

            MotorStateCache local_state;
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                local_state = latest_motor_state_;
            }

            // 当前控制模式
            const int mode = current_mode_.load();

            // switch-case 模式切换
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

            // 记录本次循环结束时间
            auto cycle_end = std::chrono::steady_clock::now();
            // 如果控制算法执行时间超过周期时间，这里不等待，直接进入下一周期
            auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(cycle_end - cycle_start);
            // 如果执行时间小于周期时间，补足等待
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

    // ===================== 发布 RobotState 给 GUI =====================
    void publishRobotState(const MotorStateCache &state)
    {
        upperlimb_robot::msg::RobotState msg;

        // float64[] position
        // float64[] velocity
        // float64[] torque
        // bool[] enabled
        msg.position.assign(state.position.begin(), state.position.end());
        msg.velocity.assign(state.velocity.begin(), state.velocity.end());
        msg.torque.assign(state.torque.begin(), state.torque.end());
        msg.enabled.assign(state.enabled.begin(), state.enabled.end());

        robot_state_pub_->publish(msg);
    }

    // ===================== 各控制模式骨架 =====================

    void runIdle(const MotorStateCache &state)
    {
        (void)state;
        // TODO:
        // 待机 / 不输出控制 / 保持安全状态
    }

    void runPassiveTraining(const MotorStateCache &state)
    {
        (void)state;
        // TODO:
        // 被动训练控制算法
        // 例如：轨迹规划 + 位置控制
    }

    void runZeroForceMode(const MotorStateCache &state)
    {
        (void)state;
        // TODO:
        // 零力模式控制算法
        // 例如：导纳 / 阻抗 / 力矩控制
    }

    void runBilateralMode(const MotorStateCache &state)
    {
        (void)state;
        // TODO:
        // 双侧协同控制
    }

    void runSlidingModeControl(const MotorStateCache &state)
    {
        (void)state;
        // TODO:
        // 滑模控制
    }

    void runLeftPresetPosition(const MotorStateCache &state)
    {
        (void)state;
        // TODO:
        // 左侧回归预设位置
    }

private:
    // ===================== ROS2 通信 =====================
    rclcpp::Subscription<upperlimb_robot::msg::ControlCommand>::SharedPtr control_command_sub_;
    rclcpp::Publisher<upperlimb_robot::msg::RobotState>::SharedPtr robot_state_pub_;

    // ===================== 线程 =====================
    std::thread state_read_thread_; // 高频状态采集线程
    std::thread control_thread_;    // 高频控制线程

    // ===================== 线程控制 =====================
    std::atomic<bool> running_{false};
    std::atomic<bool> state_ready_{false};

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