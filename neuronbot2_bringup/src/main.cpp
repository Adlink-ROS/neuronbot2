//  Copyright 2020 ADLINK Technology, Inc.
//  Developer: YU-WEN, CHEN (real.yuwen@gmail.com)
// 
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
// 
//      http://www.apache.org/licenses/LICENSE-2.0
// 
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#include <rclcpp/rclcpp.hpp>

#include "neuronbot2_bringup/neuron_serial.hpp"

int main(int argc, char *argv[])
{
    //setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    // Create an executor
    rclcpp::executors::SingleThreadedExecutor executor;

    // Create a node and add into the executor
    auto neuron_serial = std::make_shared<neuronbot2::NeuronSerial>();
    executor.add_node(neuron_serial);

    // Spin the executor in a single thread
    executor.spin();

    rclcpp::shutdown();
    return 0;    
}
