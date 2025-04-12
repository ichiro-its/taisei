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