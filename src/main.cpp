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

#include "taisei/taisei.hpp"

int main(int argc, char ** argv){

    auto args = rclcpp::init_and_remove_ros_arguments(argc, argv);

    if (args.size() < 3) {
        std::cerr << "Usage: ros2 run taisei robot_wrapper <urdf_path> <frame_config_path>" << std::endl;
        return 0;
    }

    const std::string & model_path = args[1];
    const std::string & config_path = args[2];

    auto node = rclcpp::Node::make_shared("RobotWrapperNode");
    auto robot_wrapper_node = std::make_shared<taisei::RobotWrapperNode>(node, model_path, config_path);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 
