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

namespace {
    constexpr double FOOT_HEIGHT_EPS = 1e-3;
}

RobotWrapper::RobotWrapper(const std::string & model_directory) : model_directory_(model_directory){
    build_urdf();
    update_kinematics();
    get_q_indexes();
    get_joint_dictionary();
    get_feet_id();

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

void RobotWrapper::get_feet_id(){
    const bool has_left_frame  = model.existFrame("left_foot_frame");
    const bool has_right_frame = model.existFrame("right_foot_frame");
    const bool has_left_link   = model.existFrame("left_foot");
    const bool has_right_link  = model.existFrame("right_foot");   

    if (has_left_frame && has_right_frame){
        left_foot_id = model.getFrameId("left_foot_frame");
        right_foot_id = model.getFrameId("right_foot_frame");
    } else if (has_left_link && has_right_link){
        left_foot_id = model.getFrameId("left_foot");
        right_foot_id = model.getFrameId("right_foot");
    } else {
        throw std::runtime_error("URDF has no feet frames");
    }
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

    const auto joint = joint_dictionary.find(joint_id);
    if (joint == joint_dictionary.end()) return;

    const std::string& joint_name = joint->second;
    double position = position_deg * M_PI/180.0;

    auto q_joint = q_index_map.find(joint_name);
    if (q_joint == q_index_map.end()) return;

    const int qidx = q_joint->second;

    if(qidx < 0 || qidx >= (int)q.size()) return;
    q[qidx] = position;
}


// Update robot orientation using gravity (roll & pitch) and yaw
void RobotWrapper::update_orientation(const keisan::Point3& gravity, const keisan::Angle<double>& yaw)
{
    Eigen::Vector3d g_body{
        -gravity.y,
        gravity.x,
        gravity.z
    };

    if (g_body.norm() < 1e-6)
        return;

    g_body.normalize();

    Eigen::Quaterniond level_q;
    level_q.setFromTwoVectors(g_body, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond yaw_q(
        Eigen::AngleAxisd(yaw.radian(), Eigen::Vector3d::UnitZ())
    );

    Eigen::Quaterniond final_q = yaw_q * level_q;
    final_q.normalize();

    body_quaterniond = final_q;

    if (q.size() >= 7) {
        q[3] = final_q.x();
        q[4] = final_q.y();
        q[5] = final_q.z();
        q[6] = final_q.w();
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
    const pinocchio::SE3& T_L = data->oMf[left_foot_id];
    const pinocchio::SE3& T_R = data->oMf[right_foot_id];


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
    base_footprint_world = compute_base_footprint_world();
    
    std::vector<geometry_msgs::msg::TransformStamped> tf_list;

    for (size_t i = 1; i < model.frames.size(); ++i) {
        const auto& frame = model.frames[i];
        auto parent_idx = frame.parentFrame;

        //broadcast only BODY type frames
        if (frame.type != pinocchio::FrameType::BODY)
            continue;

        //construct correct tf tree hierarchy
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
            T_relative = base_footprint_world.inverse() * T_w_link;
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

aruku_interfaces::msg::WalkPhase RobotWrapper::get_walk_phase(){
    Eigen::Vector3d pL = base_footprint_world.inverse().act(data->oMf[left_foot_id].translation());
    Eigen::Vector3d pR = base_footprint_world.inverse().act(data->oMf[right_foot_id].translation());

    aruku_interfaces::msg::WalkPhase walk_phase;

    bool left_contact  = std::abs(pL.z()) < FOOT_HEIGHT_EPS;
    bool right_contact = std::abs(pR.z()) < FOOT_HEIGHT_EPS;

    if (left_contact && right_contact) {
        walk_phase.current = aruku_interfaces::msg::WalkPhase::DOUBLE_SUPPORT;
    } else if (left_contact) {
        walk_phase.current = aruku_interfaces::msg::WalkPhase::LEFT_SUPPORT;
    } else if (right_contact) {
        walk_phase.current = aruku_interfaces::msg::WalkPhase::RIGHT_SUPPORT;
    }

    return walk_phase;
}

} //namespace taisei
