// 1.包含头文件
#include "rclcpp/rclcpp.hpp"
#include "upperlimb_robot/msg/control_command.hpp"
#include "upperlimb_robot/gxn.h"


// 3.自定义节点类；
class ControlAlgorithmNode : public rclcpp::Node
{
public:
    ControlAlgorithmNode() : Node("control_algorithm_node")
    {
    }
};

int main(int argc, char const *argv[])
{
    // 2.初始化ROS2客户端；
    rclcpp::init(argc, argv);
    // 4.调用spain函数，并传入节点对象指针；
    rclcpp::spin(std::make_shared<ControlAlgorithmNode>());
    // 5.资源释放
    rclcpp::shutdown();
    return 0;
}