#ifndef NODE_HPP
#define NODE_HPP    

#include "robot_wrapper/robot_wrapper.hpp"

class RobotWrapperNode
{
public:

    RobotWrapperNode(const rclcpp::Node::SharedPtr& node, const std::string &model_directory); 

    void broadcast_tf_frames();

private:
    
    std::shared_ptr<RobotWrapper> robot_wrapper;
    std::vector<geometry_msgs::msg::TransformStamped> tf_frames;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Node::SharedPtr node;
    rclcpp::Subscription<tachimawari_interfaces::msg::CurrentJoints>::SharedPtr joint_subscriber;
    rclcpp::TimerBase::SharedPtr node_timer;
};

#endif //NODE_HPP