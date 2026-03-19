#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

#include <Eigen/Dense>

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>

using namespace std::chrono_literals;

class IkRvizNode : public rclcpp::Node
{
public:
  IkRvizNode() : Node("ik_rviz_node")
  {
    try
    {
      // ============================================================
      // 1. 读取 URDF
      // ============================================================
      // 默认从 pinocchio_kin_demo 包中读取
      std::string package_path =
          ament_index_cpp::get_package_share_directory("pinocchio_kin_demo");
      std::string urdf_path = package_path + "/urdf/my_robot.urdf";

      // 末端 frame 名称
      // 你的模型虽然最后 3 个关节固定了，但 link10 仍然存在，
      // 因此仍然可以把 link10 作为末端坐标系使用。
      ee_frame_ = "link10";

      // 构建 Pinocchio 模型
      pinocchio::urdf::buildModel(urdf_path, model_);
      data_ = std::make_shared<pinocchio::Data>(model_);

      RCLCPP_INFO(this->get_logger(), "已成功加载机器人模型");
      RCLCPP_INFO(this->get_logger(), "模型维度：nq = %d, nv = %d", model_.nq, model_.nv);

      // 获取末端 frame 索引
      frame_id_ = model_.getFrameId(ee_frame_);
      if (frame_id_ >= model_.frames.size())
      {
        RCLCPP_ERROR(this->get_logger(), "未找到末端坐标系：%s", ee_frame_.c_str());
        throw std::runtime_error("末端 frame 不存在");
      }

      // ============================================================
      // 2. 关节名称
      // ============================================================
      // 当前 URDF 中 joint8、joint9、joint10 已经改成 fixed，
      // 因此真正可动关节只有 7 个。
      joint_names_ = {
          "joint1", "joint2", "joint3", "joint4",
          "joint5", "joint6", "joint7"};

      // 对于当前这种 revolute + fixed 的 URDF，
      // 一般 nq = nv = 7，所以这里可以直接和 nq 比较。
      if (static_cast<int>(joint_names_.size()) != model_.nq)
      {
        RCLCPP_ERROR(this->get_logger(),
                     "关节名称数量与 nq 不匹配：%zu vs %d",
                     joint_names_.size(), model_.nq);
        throw std::runtime_error("关节名称数量错误");
      }

      // ============================================================
      // 3. 创建 /joint_states 发布器
      // ============================================================
      // robot_state_publisher 会读取 /joint_states，
      // 再发布整棵机器人 TF 树到 RViz。
      joint_state_pub_ =
          this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

      // ============================================================
      // 4. 设置逆运动学初值（单位：度）
      // ============================================================
      q_deg_ = {
          0.0,
          0.0,
          -32.0,
          0.0,
          0.0,
          -22.0,
          -62.0};

      if (static_cast<int>(q_deg_.size()) != model_.nq)
      {
        RCLCPP_ERROR(this->get_logger(),
                     "输入角度数量与 nq 不匹配：%zu vs %d",
                     q_deg_.size(), model_.nq);
        throw std::runtime_error("输入角度数量错误");
      }

      // ============================================================
      // 5. 设置目标末端位置（单位：米）
      // ============================================================
      // 这是“只考虑位置，不考虑姿态”的位置型 IK。
      target_pos_ << 0.645666, -0.205327, -0.348438;

      RCLCPP_INFO(this->get_logger(),
                  "目标末端位置：x = %.4f, y = %.4f, z = %.4f",
                  target_pos_.x(), target_pos_.y(), target_pos_.z());

      // ============================================================
      // 6. 执行 Jacobian Transpose 逆运动学
      // ============================================================
      bool success = solveIK();

      if (success)
      {
        RCLCPP_INFO(this->get_logger(), "逆运动学求解成功");
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "逆运动学未完全收敛，但会发布当前结果");
      }

      // 打印最终结果
      print_final_result();

      // ============================================================
      // 7. 定时发布 joint_states，让 RViz 中姿态保持显示
      // ============================================================
      timer_ = this->create_wall_timer(
          100ms, std::bind(&IkRvizNode::publish_joint_states, this));

      RCLCPP_INFO(this->get_logger(), "正在发布 /joint_states，可在 RViz 中观察结果");
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "初始化失败：%s", e.what());
      throw;
    }
  }

private:
  // ============================================================
  // 角度与弧度转换
  // ============================================================
  double deg2rad(double deg) const
  {
    return deg * M_PI / 180.0;
  }

  double rad2deg(double rad) const
  {
    return rad * 180.0 / M_PI;
  }

  // ============================================================
  // 把当前 q_deg_ 转为 Pinocchio 需要的弧度向量 q
  // ============================================================
  Eigen::VectorXd build_q_from_deg() const
  {
    Eigen::VectorXd q(model_.nq);
    for (int i = 0; i < model_.nq; ++i)
    {
      q[i] = deg2rad(q_deg_[i]);
    }
    return q;
  }

  // ============================================================
  // 关节角限幅（单位：度）
  // ============================================================
  // 当前 URDF 中 revolute 关节限位写的是 ±pi（约 ±180°），
  // 因此这里也按 ±180° 限幅。
  double clamp_deg(double x) const
  {
    return std::max(-180.0, std::min(180.0, x));
  }

  // ============================================================
  // Jacobian Transpose 逆运动学求解
  // 只做位置 IK，不约束姿态
  // ============================================================
  bool solveIK()
  {
    // 使用当前 q_deg_ 作为初值
    Eigen::VectorXd q = build_q_from_deg();

    // 迭代参数
    const int max_iter = 500; // 最大迭代次数
    const double tol = 1e-4;  // 位置误差收敛阈值（米）
    const double alpha = 0.05; // Jacobian Transpose 步长，过大容易发散

    bool converged = false;

    for (int iter = 0; iter < max_iter; ++iter)
    {
      // --------------------------------------------------------
      // 1. 正运动学
      // --------------------------------------------------------
      pinocchio::forwardKinematics(model_, *data_, q);
      pinocchio::updateFramePlacements(model_, *data_);

      // 当前末端位姿
      const pinocchio::SE3 &pose = data_->oMf[frame_id_];
      Eigen::Vector3d p = pose.translation();

      // --------------------------------------------------------
      // 2. 位置误差
      // --------------------------------------------------------
      Eigen::Vector3d err = target_pos_ - p;
      double err_norm = err.norm();

      if (iter % 20 == 0 || err_norm < tol)
      {
        RCLCPP_INFO(this->get_logger(),
                    "第 %d 次迭代：误差范数 = %.6f",
                    iter, err_norm);
      }

      // 收敛判断
      if (err_norm < tol)
      {
        converged = true;
        break;
      }

      // --------------------------------------------------------
      // 3. 计算雅可比矩阵
      // --------------------------------------------------------
      pinocchio::computeJointJacobians(model_, *data_, q);

      Eigen::MatrixXd J(6, model_.nv);
      pinocchio::getFrameJacobian(
          model_,
          *data_,
          frame_id_,
          pinocchio::LOCAL_WORLD_ALIGNED,
          J);

      // 只取位置部分（后 3 行）
      Eigen::MatrixXd Jv = J.bottomRows(3);

      // --------------------------------------------------------
      // 4. Jacobian Transpose
      //
      // dq = alpha * J^T * err
      //
      // 这是最经典、最简单的数值逆运动学方法。
      // 优点：形式简单，容易实现。
      // 缺点：收敛速度和稳定性通常不如阻尼最小二乘。
      // --------------------------------------------------------
      Eigen::VectorXd dq = alpha * Jv.transpose() * err;

      // --------------------------------------------------------
      // 5. 限制单次更新步长
      // --------------------------------------------------------
      // 防止某次 Jacobian 更新过大，导致关节突然跳到边界
      // 或让误差发散。
      const double dq_max = 0.1; // 单次最大弧度步长
      if (dq.norm() > dq_max)
      {
        dq = dq / dq.norm() * dq_max;
      }

      // 步长更新
      q = q + dq;

      // --------------------------------------------------------
      // 6. 关节限幅
      // --------------------------------------------------------
      // 这里为了简单，直接对每个关节做角度限幅。
      for (int i = 0; i < model_.nq; ++i)
      {
        double q_deg = rad2deg(q[i]);
        q_deg = clamp_deg(q_deg);
        q[i] = deg2rad(q_deg);
      }
    }

    // 把最终结果写回 q_deg_
    for (int i = 0; i < model_.nq; ++i)
    {
      q_deg_[i] = rad2deg(q[i]);
    }

    return converged;
  }

  // ============================================================
  // 打印最终求解结果
  // ============================================================
  void print_final_result()
  {
    Eigen::VectorXd q = build_q_from_deg();

    pinocchio::forwardKinematics(model_, *data_, q);
    pinocchio::updateFramePlacements(model_, *data_);
    pinocchio::computeJointJacobians(model_, *data_, q);

    const pinocchio::SE3 &pose = data_->oMf[frame_id_];
    Eigen::Vector3d p = pose.translation();
    Eigen::Matrix3d R = pose.rotation();

    // 构造齐次变换矩阵
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = p;

    // 位置误差
    Eigen::Vector3d err = target_pos_ - p;

    // Jacobian
    Eigen::MatrixXd J(6, model_.nv);
    pinocchio::getFrameJacobian(
        model_,
        *data_,
        frame_id_,
        pinocchio::LOCAL_WORLD_ALIGNED,
        J);

    Eigen::MatrixXd Jv = J.bottomRows(3);
    Eigen::MatrixXd J_force = J.transpose();

    std::cout << "\n================ 最终关节角结果（度） ================\n";
    for (size_t i = 0; i < q_deg_.size(); ++i)
    {
      std::cout << joint_names_[i] << " : " << q_deg_[i] << " 度\n";
    }

    std::cout << "\n================ 目标末端位置 ================\n";
    std::cout << target_pos_.transpose() << "\n";

    std::cout << "\n================ 实际末端位置 ================\n";
    std::cout << p.transpose() << "\n";

    std::cout << "\n================ 最终位置误差 ================\n";
    std::cout << err.transpose() << "\n";
    std::cout << "误差范数 = " << err.norm() << "\n";

    std::cout << "\n================ 末端旋转矩阵 R ================\n";
    std::cout << R << "\n";

    std::cout << "\n================ 末端齐次变换矩阵 T ================\n";
    std::cout << T << "\n";

    std::cout << "\n================ 位置 Jacobian Jv (3 x nv) ================\n";
    std::cout << Jv << "\n";

    std::cout << "\n================ 速度 Jacobian J (6 x nv) ================\n";
    std::cout << J << "\n";

    std::cout << "\n================ 力映射矩阵 J^T (nv x 6) ================\n";
    std::cout << J_force << "\n";
  }

  // ============================================================
  // 周期发布 joint_states
  // ============================================================
  void publish_joint_states()
  {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->now();
    msg.name = joint_names_;
    msg.position.resize(q_deg_.size());

    for (size_t i = 0; i < q_deg_.size(); ++i)
    {
      msg.position[i] = deg2rad(q_deg_[i]);
    }

    // 这里只发位置即可
    msg.velocity.clear();
    msg.effort.clear();

    joint_state_pub_->publish(msg);
  }

private:
  // Pinocchio 模型与数据
  pinocchio::Model model_;
  std::shared_ptr<pinocchio::Data> data_;

  // 末端 frame
  std::string ee_frame_;
  pinocchio::FrameIndex frame_id_;

  // 可动关节名称与当前角度（单位：度）
  std::vector<std::string> joint_names_;
  std::vector<double> q_deg_;

  // 目标末端位置
  Eigen::Vector3d target_pos_;

  // ROS 发布器与定时器
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<IkRvizNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}