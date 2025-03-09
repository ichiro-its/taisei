#include "taisei/robot_wrapper/robot_wrapper.hpp"
#include <jitsuyo/config.hpp>
#include <nlohmann/json.hpp>
#include <fstream>

namespace taisei {

std::map<u_int8_t, std::string> joint_dictionary = {
    {1, "right_shoulder_pitch"},
    {2, "left_shoulder_pitch"},
    {3, "right_shoulder_roll"},
    {4, "left_shoulder_roll"},
    {5, "right_elbow"},
    {6, "left_elbow"},
    {7, "right_hip_yaw"},
    {8, "left_hip_yaw"},
    {9, "right_hip_roll"},
    {10, "left_hip_roll"},
    {11, "right_hip_pitch"},
    {12, "left_hip_pitch"},
    {13, "right_knee"},
    {14, "left_knee"},
    {15, "right_ankle_pitch"},
    {16, "left_ankle_pitch"},
    {17, "right_ankle_roll"},
    {18, "left_ankle_roll"},
    {19, "neck_yaw"},
    {20, "neck_pitch"},
  };

RobotWrapper::RobotWrapper(const std::string & model_directory, const std::string & config_path) : model_directory_(model_directory), path_(config_path){
    build_urdf();
    update_kinematics();
    get_frame_indexes();
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
    for(size_t i = 1; i<links.size(); i++){

        const std::string& link_name = links[i].name;
        const auto &frame_id = model.getFrameId(link_name);

        const int link_parent_id = links[i].parent_id;
        
        const std::string& parent_link_name = links[link_parent_id].name;

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

    nlohmann::json link_names;
    if(!jitsuyo::load_config(path_, "frame_names.json", link_names)){
        std::cerr << "Failed to load config" << std::endl;
        return;
    }

    for (const auto& entry : link_names["links"]) {
        Link link;
        bool success = true;

        for(const auto& [name, parent_id_json] : entry.items()){
            link.name = name;
            success = jitsuyo::assign_val(entry, name, link.parent_id);
        }

        links.push_back(link);
    };
}

} //namespace taisei