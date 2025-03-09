#ifndef TAISEI__ROBOT_WRAPPER__ROBOT_WRAPPER_HPP_
#define TAISEI__ROBOT_WRAPPER__ROBOT_WRAPPER_HPP_


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

extern std::map<u_int8_t, std::string> joint_dictionary;

namespace taisei
{

struct Link{
    std::string name;
    int parent_id;
};

class RobotWrapper{
    
private:

    pinocchio::Model model;
    pinocchio::Data* data;
    std::string model_directory_;
    std::string path_;
    Eigen::VectorXd q;
    std::vector<std::pair<pinocchio::FrameIndex, pinocchio::FrameIndex>> frame_indexes;
    std::vector<Link> links;

public:

    RobotWrapper(const std::string & model_directory, const std::string & config_path);
    void build_urdf();
    void update_kinematics();
    void get_frame_indexes();
    void update_joint_positions(u_int8_t joint_id, double position);
    void get_config();
    std::vector<geometry_msgs::msg::TransformStamped> get_tf_frames();

};

} //namespace taisei


#endif //ROBOT_WRAPPER_HPP
