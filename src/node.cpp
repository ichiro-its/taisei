#include "robot_wrapper/robot_wrapper.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include <chrono>
#include <memory>
#include <string>

using namespace std::chrono_literals;

class RobotWrapperNode : public rclcpp::Node 
{
    public:
    RobotWrapperNode(const std::string &model_directory) : Node("robot_wrapper_node"){
        robot_wrapper = std::make_shared<RobotWrapper>(model_directory);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        node_timer = this->create_wall_timer(8ms, [this]() { this->broadcast_tf_frames();});
        joint_subscription = this->create_subscription<tachimawari_interfaces::msg::CurrentJoints>("topic", 10, 
        [this](tachimawari_interfaces::msg::CurrentJoints::SharedPtr msg) -> void {
            for(const auto& joint : msg->joints){
                robot_wrapper->update_joint_positions(joint.id, joint.position);
            }
        });
    }

    private:
    void broadcast_tf_frames(){
        tf_frames = robot_wrapper->get_tf_frames();
        for(auto &tf_frame : tf_frames){
            tf_frame.header.stamp = this->now();
            tf_broadcaster_->sendTransform(tf_frame);
        }
    }

    std::shared_ptr<RobotWrapper> robot_wrapper;
    std::vector<geometry_msgs::msg::TransformStamped> tf_frames;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<tachimawari_interfaces::msg::CurrentJoints>::SharedPtr joint_subscription;
    rclcpp::TimerBase::SharedPtr node_timer;
};

int main(int argc, char ** argv){

    if (argc < 2) {
        std::cerr << "Usage: ros2 run robot_wrapper robot_wrapper" << " path_to_urdf" << std::endl;
        return 1;
    }
    std::string model_path = argv[1];

    rclcpp::init(argc, argv);

    auto node = std::make_shared<RobotWrapperNode>(model_path);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 