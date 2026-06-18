#include "rclcpp/rclcpp.hpp"
#include "tm_msgs/srv/ask_sta.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <iostream>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("demo_ask_sta");
  rclcpp::Client<tm_msgs::srv::AskSta>::SharedPtr client =
    node->create_client<tm_msgs::srv::AskSta>("ask_sta");

  auto request = std::make_shared<tm_msgs::srv::AskSta::Request>();

  // Refer to Spec [ex: Queries whether to enter the Listen Node(external script control mode) or not]
  request->subcmd = "00";  // SubCmd 00: In external script control mode or not
  request->subdata = "";  // Subdata is not required and should be left empty for subcmd 00

  /* 
     Refer to Spec [ex: Queries whether the specific QueueTag number '1' has been executed completely or not]
     request->subcmd = "01";  // SubCmd 01: Complete the configured QueueTag numbering or not.
     request->subdata = "1";  // The specific QueueTag number (1-15), e.g., '1'
  */

  request->wait_time = 1.0;

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 1;
    }
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto getResult = result.get();
    if (getResult->ok) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "OK");
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), getResult->subcmd);
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), getResult->subdata);
    }
    else
    {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "not OK");
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Failed to call service");
  }

  rclcpp::shutdown();
  return 0;
}
