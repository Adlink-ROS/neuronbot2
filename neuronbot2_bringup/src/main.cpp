#include <rclcpp/rclcpp.hpp>

#include "neuronbot2_bringup/neuron_serial.hpp"
#include "neuronbot2_bringup/neuron_base.hpp"

int main(int argc, char *argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    // Create an executor
    rclcpp::executors::SingleThreadedExecutor executor;

    // Create a node and add into the executor
    auto neuron_base = std::make_shared<neuronbot2::NeuronBase>();
    executor.add_node(neuron_base);

    // Create a node and add into the executor
    auto neuron_serial = std::make_shared<neuronbot2::NeuronSerial>();
    executor.add_node(neuron_serial);

    // Spin the executor in a single thread
    executor.spin();

    rclcpp::shutdown();
    return 0;    
}
