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

#include "taisei/robot_wrapper/robot_wrapper.hpp"
#include "tachimawari/joint/model/joint_id.hpp"
#include <jitsuyo/config.hpp>
#include <stdexcept>
#include <cmath>

namespace taisei {

RobotWrapper::RobotWrapper(const std::string & model_directory) : model_directory_(model_directory){
    build_urdf();
    update_kinematics();
    get_q_indexes();
    get_joint_dictionary();

    if (model.names.size() > 1 && !model.names[1].empty()){
        floating_base_name = model.names[1];
    } else {
        floating_base_name = "base_link";
    }

    body_quaterniond.setIdentity();
}


void RobotWrapper::build_urdf() {
    pinocchio::urdf::buildModel(model_directory_, pinocchio::JointModelFreeFlyer(), model);
    data = std::make_unique<pinocchio::Data>(model);
    q = pinocchio::neutral(model);
}

void RobotWrapper::update_kinematics() {
    pinocchio::forwardKinematics(model, *data, q);
    pinocchio::updateGlobalPlacements(model, *data);
    pinocchio::updateFramePlacements(model, *data);
}

//get correct quartenion index of each joints
void RobotWrapper::get_q_indexes() {
    q_index_map.clear();
    for(pinocchio::JointIndex jid = 0; jid < model.njoints; ++jid){
        const std::string& name = model.names[jid];
        if (name.empty() ) continue;

        q_index_map[name] = model.joints[jid].idx_q();
    }
 
}

//update joint position based on tachimawari's current joint
void RobotWrapper::update_joint_positions(u_int8_t joint_id, double position_deg){

    const auto it_name = joint_dictionary.find(joint_id);
    if (it_name == joint_dictionary.end()) return;

    const std::string& joint_name = it_name->second;
    double position = position_deg * M_PI/180.0;

    auto it = q_index_map.find(joint_name);
    if (it == q_index_map.end()) return;

    const int qidx = it->second;

    if(qidx < 0 || qidx >= (int)q.size()) return;
    q[qidx] = position;
}


void RobotWrapper::update_orientation(
    const keisan::Angle<double>& roll,
    const keisan::Angle<double>& pitch,
    const keisan::Angle<double>& yaw)
{
    Eigen::Quaterniond imu_q =
        Eigen::AngleAxisd(yaw.radian(),   Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch.radian(), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll.radian(),  Eigen::Vector3d::UnitX());

    imu_q.normalize();

    body_quaterniond = imu_q;

    if (q.size() >= 7) {
        q[3] = imu_q.x();
        q[4] = imu_q.y();
        q[5] = imu_q.z();
        q[6] = imu_q.w();
    }
}


const pinocchio::SE3 RobotWrapper::get_frame_by_name(const std::string& name){
    if(!model.existFrame(name))
        throw std::runtime_error("Frame not found: " + name);

    const auto frame_id = model.getFrameId(name);
    return data->oMf[frame_id];
}


// compute base footprint in world 
pinocchio::SE3 RobotWrapper::compute_base_footprint_world() {
    if (!model.existFrame("left_foot_frame") ||
        !model.existFrame("right_foot_frame")) {
        return pinocchio::SE3::Identity();
    }

    const auto& T_L = data->oMf[model.getFrameId("left_foot")];
    const auto& T_R = data->oMf[model.getFrameId("right_foot")];

    Eigen::Vector3d pL = T_L.translation();
    Eigen::Vector3d pR = T_R.translation();

    Eigen::Vector3d mid = 0.5 * (pL + pR);

    // Use lowest foot for stable ground contact
    mid.z() = std::min(pL.z(), pR.z());

    double yaw = get_yaw_from_quaternion(body_quaterniond);

    Eigen::Matrix3d R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    return pinocchio::SE3(R, mid);
}

std::vector<geometry_msgs::msg::TransformStamped> RobotWrapper::get_all_transforms(const rclcpp::Time& stamp) {
    update_kinematics();
    
    pinocchio::SE3 T_w_bf = compute_base_footprint_world();
    
    std::vector<geometry_msgs::msg::TransformStamped> tf_list;

    for (size_t i = 1; i < model.frames.size(); ++i) {
        const auto& frame = model.frames[i];
        auto parent_idx = frame.parentFrame;

        //broadcast only BODY type frames
        if (frame.type != pinocchio::FrameType::BODY)
            continue;

        while (parent_idx != 0 &&
            model.frames[parent_idx].type != pinocchio::FrameType::BODY)
        {
            parent_idx = model.frames[parent_idx].parentFrame;
        }

        geometry_msgs::msg::TransformStamped ts;
        ts.header.stamp = stamp;
        ts.child_frame_id = frame.name;

        pinocchio::SE3 T_relative;
        if (parent_idx == 0) {
            ts.header.frame_id = "base_footprint";
        
            pinocchio::SE3 T_w_link = data->oMf[i];
            T_relative = T_w_bf.inverse() * T_w_link;
        } 
        else {
            ts.header.frame_id = model.frames[parent_idx].name;
            
            pinocchio::SE3 T_w_p = data->oMf[parent_idx];
            pinocchio::SE3 T_w_c = data->oMf[i];
            T_relative = T_w_p.inverse() * T_w_c;
        }

        ts.transform.translation.x = T_relative.translation().x();
        ts.transform.translation.y = T_relative.translation().y();
        ts.transform.translation.z = T_relative.translation().z();

        Eigen::Quaterniond q(T_relative.rotation());
        ts.transform.rotation.x = q.x();
        ts.transform.rotation.y = q.y();
        ts.transform.rotation.z = q.z();
        ts.transform.rotation.w = q.w();

        tf_list.push_back(ts);
    }

    return tf_list;
}

double RobotWrapper::get_yaw_from_quaternion(const Eigen::Quaterniond& q)
{
    const double w = q.w();
    const double x = q.x();
    const double y = q.y();
    const double z = q.z();

    const double siny_cosp = 2.0 * (w*z + x*y);
    const double cosy_cosp = 1.0 - 2.0 * (y*y + z*z);

    return std::atan2(siny_cosp, cosy_cosp);
}

void RobotWrapper::get_joint_dictionary(){
    for (const auto &pair : tachimawari::joint::JointId::by_name) {
        joint_dictionary[pair.second] = pair.first;
    }
}


} //namespace taisei
