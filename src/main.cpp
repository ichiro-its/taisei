#include "main.hpp"

int main(int argc, char ** argv){

    if (argc < 2) {
        std::cerr << "Usage: ros2 run taisei robot_wrapper" << " path_to_urdf" << std::endl;
        return 1;
    }
    std::string model_path = argv[1];

    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("RobotWrapperNode");
    auto robot_wrapper_node = std::make_shared<RobotWrapperNode>(node, model_path);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 