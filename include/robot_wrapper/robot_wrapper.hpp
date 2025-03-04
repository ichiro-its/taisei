#ifndef ROBOT_WRAPPER_HPP
#define ROBOT_WRAPPER_HPP


#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "Eigen/Geometry"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tachimawari_interfaces/msg/current_joints.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include <chrono>
#include <vector>
#include <memory>
#include <string>
#include <cmath>
#include <iostream>

extern std::vector<std::pair<std::string, int>> links; 
extern std::map<u_int8_t, std::string> joint_dictionary;

class RobotWrapper{
    public:
        RobotWrapper(std::string model_directory);
        void build_urdf();
        void update_kinematics();
        void get_frame_indexes();
        void publish_tf_tree();
        void update_joint_positions(u_int8_t joint_id, double position);
        std::vector<geometry_msgs::msg::TransformStamped> get_tf_frames();
        
        std::vector<std::pair<pinocchio::FrameIndex, pinocchio::FrameIndex>> frame_indexes;
        pinocchio::Model model;
        pinocchio::Data* data;
        std::string model_directory_;
        Eigen::VectorXd q;
};

#endif //ROBOT_WRAPPER_HPP