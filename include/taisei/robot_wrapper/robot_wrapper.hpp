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

#include "taisei/base_footprint/base_footprint.hpp"

#include <chrono>
#include <vector>
#include <memory>
#include <string>



namespace taisei
{

struct Link{
    std::string name;
    int parent_id;
};

class RobotWrapper
{
public:

    RobotWrapper(const std::string & model_directory, const std::string & config_path, const std::shared_ptr<BaseFootprint> & base_footprint);
    void build_urdf();
    void update_kinematics();
    void get_frame_indexes();
    void update_joint_positions(u_int8_t joint_id, double position);
    void get_joint_dictionary();
    void get_config();
    std::vector<geometry_msgs::msg::TransformStamped> get_tf_frames();
    const pinocchio::SE3 & get_left_foot_frame();
    const pinocchio::SE3 & get_right_foot_frame();

private:

    pinocchio::Model model;
    pinocchio::Data* data;
    std::string model_directory_;
    std::string path_;
    Eigen::VectorXd q;
    std::vector<std::pair<pinocchio::FrameIndex, pinocchio::FrameIndex>> frame_indexes;
    std::map<uint8_t, std::string> joint_dictionary;
    std::vector<Link> links;
    std::shared_ptr<BaseFootprint> base_footprint_;

};

} //namespace taisei


#endif //TAISEI__ROBOT_WRAPPER__ROBOT_WRAPPER_HPP_
