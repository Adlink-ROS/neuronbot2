#include "nav2_client.hpp"
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>

using namespace BT;

int main(int argc, char **argv) {

  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  auto nh = rclcpp::Node::make_shared("neuronbt");
  nh->declare_parameter("bt_xml", rclcpp::ParameterValue(std::string("/home/iven/iven_ws/neuronbot2_ws/src/neuronbt/config/neuronbt.xml")));
  std::string bt_xml;
  nh->get_parameter("bt_xml", bt_xml);
  RCLCPP_INFO(nh->get_logger(), "Loading XML : %s", bt_xml.c_str());

  // We use the BehaviorTreeFactory to register our custom nodes
  BehaviorTreeFactory factory;

  factory.registerNodeType<Nav2Client>("Nav2Client");

  // Trees are created at deployment-time (i.e. at run-time, but only once at
  // the beginning). The currently supported format is XML. IMPORTANT: when the
  // object "tree" goes out of scope, all the TreeNodes are destroyed
  auto tree = factory.createTreeFromFile(bt_xml);

  // Create a logger
  StdCoutLogger logger_cout(tree);

  NodeStatus status = NodeStatus::RUNNING;
  // Keep on ticking until you get either a SUCCESS or FAILURE state
  while (rclcpp::ok() && status == NodeStatus::RUNNING) {
    status = tree.root_node->executeTick();
    // Sleep 100 milliseconds
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return 0;
}
