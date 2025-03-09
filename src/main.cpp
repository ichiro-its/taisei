#include "taisei/taisei.hpp"

int main(int argc, char ** argv){

    auto args = rclcpp::init_and_remove_ros_arguments(argc, argv);

    if (args.size() < 3) {
        std::cerr << "Usage: ros2 run taisei robot_wrapper <urdf_path> <frame_config_path>" << std::endl;
        return 0;
    }

    const std::string & model_path = args[1];
    const std::string & config_path = args[2];

    auto node = rclcpp::Node::make_shared("RobotWrapperNode");
    auto robot_wrapper_node = std::make_shared<taisei::RobotWrapperNode>(node, model_path, config_path);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 

//ros2 run taisei robot_wrapper "/home/riyan/pinocchio_ws/src/taisei/src/taisei/robot_wrapper/umaru.urdf" "/home/riyan/pinocchio_ws/src/taisei/src/taisei/robot_wrapper/"