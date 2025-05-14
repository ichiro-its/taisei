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

#include "taisei/base_footprint/base_footprint.hpp"

namespace taisei{

BaseFootprint::BaseFootprint(){
    rotation = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}


const pinocchio::SE3 & BaseFootprint::compute_base_footprint(const pinocchio::SE3 & r_foot_frame, const pinocchio::SE3 & l_foot_frame, 
    const keisan::Angle<double> & yaw) {
    if (r_foot_frame.translation().z() < l_foot_frame.translation().z()) {
        pivot_foot = r_foot_frame;
        swing_foot = l_foot_frame;
    } else {
        pivot_foot = l_foot_frame;
        swing_foot = r_foot_frame;
    }
    
    translation.x() = (pivot_foot.translation().x() + swing_foot.translation().x())/2;
    translation.y() = (pivot_foot.translation().y() + swing_foot.translation().y())/2;
    translation.z() = pivot_foot.translation().z();

    rotation = Eigen::AngleAxisd(yaw.radian(), Eigen::Vector3d::UnitZ()).toRotationMatrix();

    base_footprint.translation() = translation;
    base_footprint.rotation() = rotation;

    return base_footprint;
}

} //namespace taisei
