#include <rclcpp/rclcpp.hpp>
#include <neuronbot2_bringup/neuron_serial.hpp>
#include <neuronbot2_bringup/neuron_base.hpp>

int main(int argc, char *argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor executor;

    auto neuron_base = std::make_shared<NeuronBase>();
    executor.add_node(neuron_base);

    // auto neuron_serial = std::make_shared<NeuronSerial>();
    // executor.add_node(neuron_serial);


    executor.spin();

    rclcpp::shutdown();
    return 0;    
}
