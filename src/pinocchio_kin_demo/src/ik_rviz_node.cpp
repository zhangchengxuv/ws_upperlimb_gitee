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
      // =========================================
      // 1. 读取 URDF
      // =========================================
      std::string package_path =
        ament_index_cpp::get_package_share_directory("pinocchio_kin_demo");
      std::string urdf_path = package_path + "/urdf/my_robot.urdf";

      ee_frame_ = "link10";

      pinocchio::urdf::buildModel(urdf_path, model_);
      data_ = std::make_shared<pinocchio::Data>(model_);

      RCLCPP_INFO(this->get_logger(), "已成功加载机器人模型");
      RCLCPP_INFO(this->get_logger(), "模型维度：nq = %d, nv = %d", model_.nq, model_.nv);

      frame_id_ = model_.getFrameId(ee_frame_);
      if (frame_id_ >= model_.frames.size())
      {
        RCLCPP_ERROR(this->get_logger(), "未找到末端坐标系：%s", ee_frame_.c_str());
        throw std::runtime_error("末端 frame 不存在");
      }

      // =========================================
      // 2. 关节名称
      // 必须与 URDF 中可动关节一致
      // =========================================
      joint_names_ = {
        "joint1", "joint2", "joint3", "joint4", "joint5",
        "joint6", "joint7", "joint8", "joint9", "joint10"
      };

      if (static_cast<int>(joint_names_.size()) != model_.nq)
      {
        RCLCPP_ERROR(this->get_logger(),
                     "关节名称数量与 nq 不匹配：%zu vs %d",
                     joint_names_.size(), model_.nq);
        throw std::runtime_error("关节名称数量错误");
      }

      // =========================================
      // 3. 初始化 joint_states 发布器
      // =========================================
      joint_state_pub_ =
        this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

      // =========================================
      // 4. 初始角度（单位：度）
      // 这里作为 IK 初值
      // =========================================
      q_deg_ = {
        0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0
      };

      // =========================================
      // 5. 目标末端位置（单位：米）
      // =========================================
      target_pos_ << 0.05, -0.10, -0.55;

      RCLCPP_INFO(this->get_logger(),
                  "目标末端位置：x = %.4f, y = %.4f, z = %.4f",
                  target_pos_.x(), target_pos_.y(), target_pos_.z());

      // =========================================
      // 6. 执行最简单数值逆运动学
      // =========================================
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

      // =========================================
      // 7. 定时发布到 RViz
      // =========================================
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
  double deg2rad(double deg) const
  {
    return deg * M_PI / 180.0;
  }

  double rad2deg(double rad) const
  {
    return rad * 180.0 / M_PI;
  }

  Eigen::VectorXd build_q_from_deg() const
  {
    Eigen::VectorXd q(model_.nq);
    for (int i = 0; i < model_.nq; ++i)
    {
      q[i] = deg2rad(q_deg_[i]);
    }
    return q;
  }

  double clamp_deg(double x) const
  {
    return std::max(-90.0, std::min(90.0, x));
  }

  bool solveIK()
  {
    // 用当前角度作为初值
    Eigen::VectorXd q = build_q_from_deg();

    // 迭代参数
    const int max_iter = 300;
    const double alpha = 0.8;       // 步长
    const double tol = 1e-4;        // 误差阈值（米）

    bool converged = false;

    for (int iter = 0; iter < max_iter; ++iter)
    {
      // 1. 正运动学
      pinocchio::forwardKinematics(model_, *data_, q);
      pinocchio::updateFramePlacements(model_, *data_);

      const pinocchio::SE3 &pose = data_->oMf[frame_id_];
      Eigen::Vector3d p = pose.translation();

      // 2. 位置误差
      Eigen::Vector3d err = target_pos_ - p;
      double err_norm = err.norm();

      if (iter % 20 == 0 || err_norm < tol)
      {
        RCLCPP_INFO(this->get_logger(),
                    "第 %d 次迭代：误差范数 = %.6f",
                    iter, err_norm);
      }

      // 3. 收敛判断
      if (err_norm < tol)
      {
        converged = true;
        break;
      }

      // 4. 计算 Jacobian
      pinocchio::computeJointJacobians(model_, *data_, q);

      Eigen::MatrixXd J(6, model_.nv);
      pinocchio::getFrameJacobian(
        model_,
        *data_,
        frame_id_,
        pinocchio::LOCAL_WORLD_ALIGNED,
        J);

      // 5. 只取线速度 Jacobian：后 3 行
      Eigen::MatrixXd Jv = J.bottomRows(3);

      // 6. Jacobian Transpose 更新
      // dq = alpha * Jv^T * err
      Eigen::VectorXd dq = alpha * Jv.transpose() * err;

      // 7. 更新 q
      q = q + dq;

      // 8.  URDF ±90 度
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

  void print_final_result()
  {
    Eigen::VectorXd q = build_q_from_deg();

    pinocchio::forwardKinematics(model_, *data_, q);
    pinocchio::updateFramePlacements(model_, *data_);
    pinocchio::computeJointJacobians(model_, *data_, q);

    const pinocchio::SE3 &pose = data_->oMf[frame_id_];
    Eigen::Vector3d p = pose.translation();
    Eigen::Matrix3d R = pose.rotation();

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = p;

    Eigen::Vector3d err = target_pos_ - p;

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

    joint_state_pub_->publish(msg);
  }

private:
  pinocchio::Model model_;
  std::shared_ptr<pinocchio::Data> data_;

  std::string ee_frame_;
  pinocchio::FrameIndex frame_id_;

  std::vector<std::string> joint_names_;
  std::vector<double> q_deg_;

  Eigen::Vector3d target_pos_;

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