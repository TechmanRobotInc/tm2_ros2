#include "tm_driver/tm_ros2_svr.h"
#include "tm_driver/tm_ros2_moveit_sct.h"
#include "rclcpp/rclcpp.hpp"

#include <sstream>

void debug_function_print(char* msg) {
  printf("%s[TM_DEBUG] %s\n%s", PRINT_CYAN.c_str(), msg, PRINT_RESET.c_str());
}

void info_function_print(char* msg) {
  printf("[TM_INFO] %s\n", msg);
}

void warn_function_print(char* msg) {
  printf("%s[TM_WARN] %s\n%s", PRINT_YELLOW.c_str(), msg, PRINT_RESET.c_str());
}

void error_function_print(char* msg) {
  printf("%s[TM_ERROR] %s\n%s", PRINT_RED.c_str(), msg, PRINT_RESET.c_str());
}

void fatal_function_print(char* msg) {
  printf("%s[TM_FATAL] %s\n%s", PRINT_GREEN.c_str(), msg, PRINT_RESET.c_str());
}

void ros_debug_print(char* msg) {
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("tm_driver"), msg);
}

void ros_info_print(char* msg) {
  RCLCPP_INFO_STREAM(rclcpp::get_logger("tm_driver"), msg);
}

void ros_warn_function_print(char* msg) {
  RCLCPP_WARN_STREAM(rclcpp::get_logger("tm_driver"), msg);
}

void ros_error_print(char* msg) {
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("tm_driver"), msg);
}

void ros_fatal_print(char* msg) {
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("tm_driver"), "[TM_FATAL] " << msg);
}

void ros_once_print(char* msg) {
  RCLCPP_INFO_STREAM_ONCE(rclcpp::get_logger("tm_driver"), msg);
}

void set_up_print_fuction() {
  set_up_print_debug_function(debug_function_print);
  set_up_print_info_function(info_function_print);
  set_up_print_warn_function(warn_function_print);
  set_up_print_error_function(error_function_print);
  set_up_print_fatal_function(fatal_function_print);
  set_up_print_once_function(default_print_once_function_print);
}

void set_up_ros_print_fuction() {
  set_up_print_debug_function(ros_debug_print);
  set_up_print_info_function(ros_info_print);
  set_up_print_warn_function(ros_warn_function_print);
  set_up_print_error_function(ros_error_print);
  set_up_print_fatal_function(ros_fatal_print);
  set_up_print_once_function(ros_once_print);
}

int main(int argc, char *argv[])
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  set_up_ros_print_fuction();

  rclcpp::init(argc, argv);

  bool is_fake = true;
  bool is_rc = false;
  std::string host;

  if (argc > 1) {
    host = argv[1];
    if (host.find("robot_ip:=") != std::string::npos) {
      host.replace(host.begin(), host.begin() + 10, "");
      is_fake = false;
    } else if (host.find("ip:=") != std::string::npos) {
      host.replace(host.begin(), host.begin() + 4, "");
      is_fake = false;
    } else if (host.find("rc:=") != std::string::npos) {
      host.replace(host.begin(), host.begin() + 4, "");
      is_fake = false;
      is_rc = true;
    } else {
      std::stringstream ss;
      ss << "Only the keywords 'ip', 'robot_ip' or 'rc' are supported, while you entered '" << host << "' in argv[1]";
      print_warn(ss.str().c_str());
    }
  }
  else
  {
    print_fatal("Incorrect argument, connection to TM Cobot failed.");
    rclcpp::shutdown();
    return 1;
  }

  if (is_fake) {
    print_info("!!! The <robot_ip_address> is not found, enter the fake experimental node. You can press CTRL+C to about.");
  }

  if (is_rc) {
    print_info("!!! Please check if remote control mode is activated in the TMflow configuration. Otherwise, it will not work.");
  }

  if (argc == 3) {
    bool isSetNoLogPrint;
    std::istringstream(argv[2]) >> std::boolalpha >> isSetNoLogPrint;
    if (isSetNoLogPrint) {
      set_up_print_fuction();
    }
  }

  TmDriver iface(host, nullptr, nullptr);

  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("tm_driver_node");

  auto tm_svr = std::make_shared<TmSvrRos2>(node, iface, is_fake);
  auto tm_sct = std::make_shared<TmRos2SctMoveit>(node, iface, is_fake, is_rc);

  iface.set_tag(iface.tag, 0);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
