#include <rclcpp/rclcpp.hpp>
#include <neuronbot2_bringup/base_driver.hpp>
#include <neuronbot2_bringup/neuron_serial.hpp>

int main(int argc, char *argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor executor;

    // auto neuronbot2 = std::make_shared<BaseDriver>();
    // executor.add_node(neuronbot2);

    auto neuron_serial = std::make_shared<NeuronSerial>();
    executor.add_node(neuron_serial);


    executor.spin();

    rclcpp::shutdown();
    return 0;    
}
