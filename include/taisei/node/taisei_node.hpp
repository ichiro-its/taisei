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

#ifndef TAISE__NODE__TAISEI_NODE_HPP
#define TAISEI__NODE__TAISEI_NODE_HPP    

#include "taisei/robot_wrapper/robot_wrapper.hpp"



namespace taisei{

class RobotWrapperNode
{
public:
    using TransformStamped = geometry_msgs::msg::TransformStamped;

    RobotWrapperNode(const rclcpp::Node::SharedPtr& node, const std::string & model_directory, const std::string & config_path); 

    void broadcast_tf_frames();

private:
    
    std::shared_ptr<RobotWrapper> robot_wrapper;
    std::vector<TransformStamped> tf_frames;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Node::SharedPtr node;
    rclcpp::Subscription<tachimawari_interfaces::msg::CurrentJoints>::SharedPtr joint_subscriber;
    rclcpp::TimerBase::SharedPtr node_timer;
};

} //namespace taisei

#endif //TAISEI__NODE__TAISEI_NODE_HPP    
