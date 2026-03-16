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

using namespace std::chrono_literals;

class FkRvizNode : public rclcpp::Node
{
public:
  FkRvizNode() : Node("fk_rviz_node")
  {
    try
    {
      // ================================
      // 1. 通过包路径获取 URDF 文件
      // ================================
      std::string package_path =
        ament_index_cpp::get_package_share_directory("pinocchio_kin_demo");
      std::string urdf_path = package_path + "/urdf/my_robot.urdf";

      ee_frame_ = "link10";

      // ================================
      // 2. 构建 Pinocchio 模型
      // ================================
      pinocchio::urdf::buildModel(urdf_path, model_);
      data_ = std::make_shared<pinocchio::Data>(model_);

      RCLCPP_INFO(this->get_logger(), "已成功加载机器人模型");
      RCLCPP_INFO(this->get_logger(), "模型配置维度 nq = %d，速度维度 nv = %d", model_.nq, model_.nv);

      frame_id_ = model_.getFrameId(ee_frame_);
      if (frame_id_ >= model_.frames.size())
      {
        RCLCPP_ERROR(this->get_logger(), "未找到末端坐标系：%s", ee_frame_.c_str());
        throw std::runtime_error("末端 frame 不存在");
      }

      // ================================
      // 3. 关节名称
      // 必须和 URDF 中的可动关节顺序一致
      // ================================
      joint_names_ = {
        "joint1", "joint2", "joint3", "joint4", "joint5",
        "joint6", "joint7", "joint8", "joint9", "joint10"
      };

      if (static_cast<int>(joint_names_.size()) != model_.nq)
      {
        RCLCPP_ERROR(this->get_logger(),
                     "关节名称数量与模型 nq 不匹配：joint_names=%zu, nq=%d",
                     joint_names_.size(), model_.nq);
        throw std::runtime_error("关节名称数量错误");
      }

      // ================================
      // 4. 输入关节角（单位：度）
      // 你以后只改这里就行
      // ================================
      q_deg_ = {
        0.0,   // joint1
        10.0,  // joint2
        -20.0, // joint3
        15.0,  // joint4
        30.0,  // joint5
        -10.0, // joint6
        25.0,  // joint7
        -5.0,  // joint8
        12.0,  // joint9
        0.0    // joint10
      };

      if (static_cast<int>(q_deg_.size()) != model_.nq)
      {
        RCLCPP_ERROR(this->get_logger(),
                     "输入角度数量与模型 nq 不匹配：q_deg=%zu, nq=%d",
                     q_deg_.size(), model_.nq);
        throw std::runtime_error("输入角度数量错误");
      }

      // ================================
      // 5. 创建 JointState 发布器
      // robot_state_publisher 会读取 /joint_states
      // 然后发布整棵机器人 TF 树到 RViz
      // ================================
      joint_state_pub_ =
        this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

      // ================================
      // 6. 先计算一次 Pinocchio 结果并打印
      // ================================
      compute_and_print_results();

      // ================================
      // 7. 定时发布 joint_states，让 RViz 姿态保持显示
      // ================================
      timer_ = this->create_wall_timer(
        100ms, std::bind(&FkRvizNode::publish_joint_states, this));

      RCLCPP_INFO(this->get_logger(), "节点已启动：正在发布 /joint_states， RViz 中查看机器人姿态");
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

  Eigen::VectorXd build_q_rad_from_deg() const
  {
    Eigen::VectorXd q(model_.nq);
    for (int i = 0; i < model_.nq; ++i)
    {
      q[i] = deg2rad(q_deg_[i]);
    }
    return q;
  }

  void compute_and_print_results()
  {
    // ================================
    // 1. 角度转弧度
    // ================================
    Eigen::VectorXd q = build_q_rad_from_deg();

    // 限位检查
    for (int i = 0; i < model_.nq; ++i)
    {
      if (q_deg_[i] < -90.0 || q_deg_[i] > 90.0)
      {
        RCLCPP_WARN(this->get_logger(),
                    "第 %d 个关节输入为 %.2f 度，超出了 ±90 度限制",
                    i + 1, q_deg_[i]);
      }
    }

    std::cout << "\n================ 输入关节角 ================\n";
    for (int i = 0; i < model_.nq; ++i)
    {
      std::cout << joint_names_[i] << " : "
                << q_deg_[i] << " 度, "
                << q[i] << " 弧度\n";
    }

    // ================================
    // 2. 正运动学
    // ================================
    pinocchio::forwardKinematics(model_, *data_, q);
    pinocchio::updateFramePlacements(model_, *data_);

    // 末端位姿
    const pinocchio::SE3 &pose = data_->oMf[frame_id_];
    Eigen::Vector3d t = pose.translation();
    Eigen::Matrix3d R = pose.rotation();

    // ================================
    // 3. 齐次变换矩阵
    // T = [R t; 0 1]
    // ================================
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = t;

    RCLCPP_INFO(this->get_logger(), "末端坐标系：%s", ee_frame_.c_str());
    RCLCPP_INFO(this->get_logger(),
                "末端位置：x = %.6f, y = %.6f, z = %.6f",
                t.x(), t.y(), t.z());

    std::cout << "\n================ 末端旋转矩阵 R ================\n";
    std::cout << R << "\n";

    std::cout << "\n================ 末端齐次变换矩阵 T ================\n";
    std::cout << T << "\n";

    std::cout << "\n================ 旋转矩阵检查 ================\n";
    std::cout << "det(R) = " << R.determinant() << "\n";

    // ================================
    // 4. 计算速度雅可比矩阵
    // 先算 joint Jacobian，再取 frame Jacobian
    // Pinocchio 中：
    // 上 3 行 = 角速度部分
    // 下 3 行 = 线速度部分
    // ================================
    pinocchio::computeJointJacobians(model_, *data_, q);

    Eigen::MatrixXd J(6, model_.nv);
    pinocchio::getFrameJacobian(
      model_,
      *data_,
      frame_id_,
      pinocchio::LOCAL_WORLD_ALIGNED,
      J);

    std::cout << "\n================ 速度雅可比矩阵 J (6 x nv) ================\n";
    std::cout << J << "\n";

    std::cout << "\n说明：\n";
    std::cout << "J 的前 3 行：角速度部分\n";
    std::cout << "J 的后 3 行：线速度部分\n";

    // 只看位置雅可比（做位置型 IK 常用）
    Eigen::MatrixXd Jv = J.bottomRows(3);
    std::cout << "\n================ 线速度雅可比矩阵 Jv (3 x nv) ================\n";
    std::cout << Jv << "\n";

    // ================================
    // 5. 给一个示例关节速度，计算末端速度
    // v = J * qdot
    // ================================
    Eigen::VectorXd qdot(model_.nv);
    qdot << 0.1, 0.2, -0.1, 0.15, 0.05, -0.2, 0.1, 0.05, -0.1, 0.2;

    Eigen::VectorXd v_ee = J * qdot;

    std::cout << "\n================ 示例关节角速度 qdot ================\n";
    std::cout << qdot.transpose() << "\n";

    std::cout << "\n================ 末端空间速度 v = J * qdot ================\n";
    std::cout << v_ee << "\n";

    std::cout << "\n角速度部分：\n" << v_ee.head(3) << "\n";
    std::cout << "\n线速度部分：\n" << v_ee.tail(3) << "\n";

    // ================================
    // 6. 力映射矩阵
    // tau = J^T * F
    // 力雅可比矩阵
    // 本质就是 J^T
    // ================================
    Eigen::MatrixXd J_force = J.transpose();

    std::cout << "\n================ 力映射矩阵 J^T (nv x 6) ================\n";
    std::cout << J_force << "\n";

    // 示例末端 wrench
    Eigen::Matrix<double, 6, 1> wrench;
    wrench << 0.0, 0.0, 1.0,   // 力矩 mx my mz
              5.0, 0.0, 0.0;   // 力 fx fy fz

    Eigen::VectorXd tau = J.transpose() * wrench;

    std::cout << "\n================ 示例末端力/力矩 F ================\n";
    std::cout << wrench << "\n";

    std::cout << "\n================ 关节力矩 tau = J^T * F ================\n";
    std::cout << tau << "\n";
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

    // 可以发速度、力矩
    msg.velocity.clear();
    msg.effort.clear();

    joint_state_pub_->publish(msg);
  }

private:
  pinocchio::Model model_;
  std::shared_ptr<pinocchio::Data> data_;

  std::string ee_frame_;
  pinocchio::FrameIndex frame_id_;

  std::vector<std::string> joint_names_;
  std::vector<double> q_deg_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FkRvizNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}