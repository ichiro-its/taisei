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

RobotWrapper::RobotWrapper(const std::string & model_directory, const std::string & config_path) : model_directory_(model_directory), path_(config_path){
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

std::vector<geometry_msgs::msg::TransformStamped> RobotWrapper::get_tf_frames(){
    std::vector<geometry_msgs::msg::TransformStamped> tf_frames;
    for(size_t i = 0; i<frame_indexes.size(); i++){
        const auto &frame = model.frames[frame_indexes[i].first];
        const auto &frame_parent = model.frames[frame_indexes[i].second];
        const auto &parent_transform = data->oMf[frame_indexes[i].second];
        
        pinocchio::SE3 relative_transform = parent_transform.inverse() * data->oMf[frame_indexes[i].first];
        Eigen::Vector3d translation = relative_transform.translation();

        geometry_msgs::msg::TransformStamped tf_frame;
        tf_frame.header.frame_id = frame_parent.name;
        tf_frame.child_frame_id = frame.name;

        tf_frame.transform.translation.x = translation.x();
        tf_frame.transform.translation.y = translation.y();
        tf_frame.transform.translation.z = translation.z();

        Eigen::Quaterniond quat(relative_transform.rotation());
        quat.normalize();

        tf_frame.transform.rotation.x = quat.x();
        tf_frame.transform.rotation.y = quat.y();
        tf_frame.transform.rotation.z = quat.z();
        tf_frame.transform.rotation.w = quat.w();

        tf_frames.push_back(tf_frame);
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

