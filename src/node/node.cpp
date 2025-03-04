#include "node/node.hpp"

using namespace std::chrono_literals;

RobotWrapperNode::RobotWrapperNode(const rclcpp::Node::SharedPtr& node, const std::string &model_directory) : node(node)
{ 
    robot_wrapper = std::make_shared<RobotWrapper>(model_directory);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);

    node_timer = node->create_wall_timer(8ms, [this]() { this->broadcast_tf_frames();});

    joint_subscriber = node->create_subscription<tachimawari_interfaces::msg::CurrentJoints>("topic", 10, 
    [this](tachimawari_interfaces::msg::CurrentJoints::SharedPtr msg) -> void {
        for(const auto& joint : msg->joints){
            robot_wrapper->update_joint_positions(joint.id, joint.position);
        }
    });
}

void RobotWrapperNode::broadcast_tf_frames(){
    tf_frames = robot_wrapper->get_tf_frames();
    for(auto &tf_frame : tf_frames){
        tf_frame.header.stamp = node->now();
        tf_broadcaster_->sendTransform(tf_frame);
    }
}

