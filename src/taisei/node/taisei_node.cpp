// MIT License

// Copyright (c) 2025 ICHIRO ITS

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "taisei/node/taisei_node.hpp"

using namespace std::chrono_literals;

namespace taisei
{


RobotWrapperNode::RobotWrapperNode(const rclcpp::Node::SharedPtr & node, const std::string & model_directory, const std::string & config_path) : node(node)
{   
    base_footprint = std::make_shared<BaseFootprint>();
    robot_wrapper = std::make_shared<RobotWrapper>(model_directory, config_path, base_footprint);
    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

    joint_subscriber = node->create_subscription<tachimawari_interfaces::msg::CurrentJoints>("/joint/current_joints", 10, 
    [this](tachimawari_interfaces::msg::CurrentJoints::SharedPtr msg) -> void {
        for(const auto& joint : msg->joints){
            robot_wrapper->update_joint_positions(joint.id, joint.position);
        }
    });

    orientation_subscriber = node->create_subscription<kansei_interfaces::msg::Status>("/measurement/status", 10,
    [this](kansei_interfaces::msg::Status::SharedPtr msg) -> void {
        auto roll = keisan::make_degree(msg->orientation.roll);
        auto pitch = keisan::make_degree(msg->orientation.pitch);
        auto yaw = keisan::make_degree(msg->orientation.yaw);
        robot_wrapper->update_orientation(roll, pitch, yaw);
    });

    node_timer = node->create_wall_timer(8ms, [this]() { this->broadcast_tf_frames();});
}

void RobotWrapperNode::broadcast_tf_frames(){
    tf_frames = robot_wrapper->get_tf_frames();
    for(auto &tf_frame : tf_frames){
        tf_frame.header.stamp = node->now();
        tf_broadcaster->sendTransform(tf_frame);
    }
}

} //namespace taisei

