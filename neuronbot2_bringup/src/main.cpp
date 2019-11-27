#include <rclcpp/rclcpp.hpp>
#include <neuronbot2_bringup/base_driver.hpp>

int main(int argc, char *argv[])
{
#if 0
    ros::init(argc, argv, "neuronbot2_driver");
#endif

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor executor;

    auto neuronbot2 = std::make_shared<BaseDriver>();
    executor.add_node(neuronbot2);

    executor.spin();

    rclcpp::shutdown();
    return 0;
    
#if 0
    // BaseDriver::Instance()->work_loop();
#endif
}
