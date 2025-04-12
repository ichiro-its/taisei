#ifndef TAISEI__BASE_FOOTPRINT__BASE_FOOTPRINT_HPP_
#define TAISEI__BASE_FOOTPRINT__BASE_FOOTPRINT_HPP_

#include "Eigen/Geometry"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "kansei_interfaces/msg/status.hpp"
#include "math.h"

namespace taisei{

class BaseFootprint
{
public:

    BaseFootprint(double initial_yaw = 0.0);
    pinocchio::SE3 compute_base_footprint(const pinocchio::SE3& r_foot_frame, const pinocchio::SE3& l_foot_frame);
    void get_odom_orientation(double yaw);

private:

    pinocchio::SE3 base_footprint_;
    pinocchio::SE3 pivot_foot;
    pinocchio::SE3 swing_foot;
    double yaw_;
 
};

} //namespace taisei

#endif //TAISEI__BASE_FOOTPRINT__BASE_FOOTPRINT_HPP_