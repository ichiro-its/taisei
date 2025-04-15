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
#include <nlohmann/json.hpp>
#include <fstream>

namespace taisei {

RobotWrapper::RobotWrapper(const std::string & model_directory, const std::string & config_path, const std::shared_ptr<BaseFootprint>& base_footprint) : model_directory_(model_directory), path_(config_path), base_footprint_(base_footprint){
    build_urdf();
    update_kinematics();
    get_frame_indexes();
    get_joint_dictionary();
}

void RobotWrapper::build_urdf(){
    pinocchio::urdf::buildModel(model_directory_, model);
    data = new pinocchio::Data(model);
    q = pinocchio::neutral(model);
}

void RobotWrapper::update_kinematics(){
    pinocchio::forwardKinematics(model, *data, q);
    pinocchio::updateGlobalPlacements(model, *data);
    pinocchio::updateFramePlacements(model, *data);
}

void RobotWrapper::get_frame_indexes(){
    get_config();

    for(const auto& link : links){

        if(link.name == "base_link") continue;
        
        const auto &frame_id = model.getFrameId(link.name);
        const std::string& parent_link_name = links[link.parent_id].name;
        const auto &frame_parent_id = model.getFrameId(parent_link_name);

        frame_indexes.push_back(std::make_pair(frame_id, frame_parent_id));
    }
 
}

void RobotWrapper::update_joint_positions(u_int8_t joint_id, double position){
    const auto joint_name = joint_dictionary[joint_id];
    auto idx = model.getJointId(joint_name);
    position = position * M_PI/180.0;
    q[idx-1] = position;
    update_kinematics();
}

const pinocchio::SE3& RobotWrapper::get_left_foot_frame(){
    const auto &frame_id = model.getFrameId("left_foot_frame");
    return data->oMf[frame_id];
}

const pinocchio::SE3& RobotWrapper::get_right_foot_frame(){
    const auto &frame_id = model.getFrameId("right_foot_frame");
    return data->oMf[frame_id];
}

std::vector<geometry_msgs::msg::TransformStamped> RobotWrapper::get_tf_frames() {
    std::vector<geometry_msgs::msg::TransformStamped> tf_frames;
    const pinocchio::SE3 computed_base_footprint = base_footprint_->compute_base_footprint(
        get_right_foot_frame(), 
        get_left_foot_frame()
    );
    bool added_base_footprint = false;

    // Helper function to create TransformStamped from SE3
    auto create_transform = [](const std::string& parent_frame, const std::string& child_frame, const pinocchio::SE3& transform) {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.frame_id = parent_frame;
        tf.child_frame_id = child_frame;

        const Eigen::Vector3d translation = transform.translation();
        tf.transform.translation.x = translation.x();
        tf.transform.translation.y = translation.y();
        tf.transform.translation.z = translation.z();

        Eigen::Quaterniond quat(transform.rotation());
        quat.normalize();
        tf.transform.rotation.x = quat.x();
        tf.transform.rotation.y = quat.y();
        tf.transform.rotation.z = quat.z();
        tf.transform.rotation.w = quat.w();

        return tf;
    };

    for (const auto& [frame_idx, parent_idx] : frame_indexes) {
        const auto& frame = model.frames[frame_idx];
        const auto& frame_parent = model.frames[parent_idx];
        const auto& parent_transform = data->oMf[parent_idx];

        if (frame_parent.name == "base_link") {
            if (!added_base_footprint) {
                tf_frames.push_back(create_transform(
                    "base_link", "base_footprint", computed_base_footprint
                ));
                added_base_footprint = true;
            }

            const pinocchio::SE3 adjusted_transform = 
                computed_base_footprint.inverse() * data->oMf[frame_idx];
            
            tf_frames.push_back(create_transform(
                "base_footprint", frame.name, adjusted_transform
            ));
        } else {
            const pinocchio::SE3 relative_transform = 
                parent_transform.inverse() * data->oMf[frame_idx];
            
            tf_frames.push_back(create_transform(
                frame_parent.name, frame.name, relative_transform
            ));
        }
    }
    return tf_frames;
}


void RobotWrapper::get_config(){

    nlohmann::ordered_json link_names;
    if(!jitsuyo::load_config(path_, "frame_names.json", link_names)){
        std::cerr << "Failed to load config" << std::endl;
        return;
    }

    for (const auto & item : link_names.items()){
        Link link;
        const std::string& key = item.key();
        bool success;

        success = jitsuyo::assign_val(link_names, key, link.parent_id);

        link.name = key;
        links.push_back(link);
    }
}

void RobotWrapper::get_joint_dictionary(){
    for (const auto &pair : tachimawari::joint::JointId::by_name) {
        joint_dictionary[pair.second] = pair.first;
    }
}



} //namespace taisei

