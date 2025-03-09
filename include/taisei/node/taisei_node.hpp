#ifndef TAISE__NODE__TAISEI_NODE_HPP
#define TAISEI__NODE__TAISEI_NODE_HPP    

#include "taisei/robot_wrapper/robot_wrapper.hpp"

using TransformStamped = geometry_msgs::msg::TransformStamped;

namespace taisei{

class RobotWrapperNode
{
public:

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
#endif //TAISEI_NODE_HPP
