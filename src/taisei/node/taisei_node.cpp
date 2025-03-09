#include "taisei/node/taisei_node.hpp"

using namespace std::chrono_literals;

namespace taisei
{


RobotWrapperNode::RobotWrapperNode(const rclcpp::Node::SharedPtr& node, const std::string & model_directory, const std::string & config_path) : node(node)
{ 
    robot_wrapper = std::make_shared<RobotWrapper>(model_directory, config_path);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);

    joint_subscriber = node->create_subscription<tachimawari_interfaces::msg::CurrentJoints>("topic", 10, 
    [this](tachimawari_interfaces::msg::CurrentJoints::SharedPtr msg) -> void {
        for(const auto& joint : msg->joints){
            robot_wrapper->update_joint_positions(joint.id, joint.position);
        }
    });

    node_timer = node->create_wall_timer(8ms, [this]() { this->broadcast_tf_frames();});
}

void RobotWrapperNode::broadcast_tf_frames(){
    tf_frames = robot_wrapper->get_tf_frames();
    for(auto &tf_frame : tf_frames){
        tf_frame.header.stamp = node->now();
        tf_broadcaster_->sendTransform(tf_frame);
    }
}

} //namespace taisei

