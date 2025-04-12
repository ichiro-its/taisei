#include "taisei/base_footprint/base_footprint.hpp"
#include <iostream>

namespace taisei{

BaseFootprint::BaseFootprint(double initial_yaw) 
  : yaw_(initial_yaw) {}

void BaseFootprint::get_odom_orientation(double yaw){
    yaw_ = M_PI/180.0 * yaw;
}

pinocchio::SE3 BaseFootprint::compute_base_footprint(const pinocchio::SE3& r_foot_frame, const pinocchio::SE3& l_foot_frame){
    if (r_foot_frame.translation().z() < l_foot_frame.translation().z()) {
        pivot_foot = r_foot_frame;
        swing_foot = l_foot_frame;
    } else {
        pivot_foot = l_foot_frame;
        swing_foot = r_foot_frame;
    }
       
    Eigen::Vector3d translation;
    Eigen::Matrix3d rotation = Eigen::AngleAxisd(yaw_, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    pinocchio::SE3 swing_foot_in_pivot_frame = pivot_foot.inverse() * swing_foot;
    
    translation.z() = 0.0;
    translation.x() = swing_foot_in_pivot_frame.translation().x()/2.0;
    translation.y() = swing_foot_in_pivot_frame.translation().y()/2.0;

    base_footprint_ = pivot_foot * pinocchio::SE3(Eigen::Matrix3d::Identity(), translation);
    base_footprint_.translation().z() = 0.0;
    
    base_footprint_.rotation() = rotation;
    return base_footprint_;
}

} //namespace taisei