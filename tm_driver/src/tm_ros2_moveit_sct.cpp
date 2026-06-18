#include "tm_driver/tm_ros2_moveit_sct.h"

void TmRos2SctMoveit::intial_action()
{
  as_ = rclcpp_action::create_server<control_msgs::action::FollowJointTrajectory>(
      node->get_node_base_interface(),
      node->get_node_clock_interface(),
      node->get_node_logging_interface(),
      node->get_node_waitables_interface(),
      "tmr_arm_controller/follow_joint_trajectory",
      std::bind(&TmRos2SctMoveit::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&TmRos2SctMoveit::handle_cancel, this, std::placeholders::_1),
      std::bind(&TmRos2SctMoveit::handle_accepted, this, std::placeholders::_1));

  goal_thread_open_ = false;
}

rclcpp_action::GoalResponse TmRos2SctMoveit::handle_goal(const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal)
{
  std::unique_lock<std::mutex> lck(goal_mtx_);

  auto goal_id = rclcpp_action::to_string(uuid);
  print_info("[ACTION] Received new action goal %s", goal_id.c_str());

  if (!is_fake_) {
    if (!svr_.is_connected()) {
      print_info("[ACTION] -----GOAL REJECT-----");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!sct_.is_connected()) {
      print_info("[ACTION] -----GOAL REJECT-----");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (state_.has_error()) {
      print_info("[ACTION] -----GOAL REJECT-----");
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  if (!has_points(goal->trajectory)) {
    print_info("[ACTION] -----GOAL REJECT-----");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (!goals_queue_.empty()) {
    print_info("[ACTION] -----GOAL ACCEPT_AND_DEFER-----");
    return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
  }

  print_info("[ACTION] -----GOAL ACCEPT-----");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void TmRos2SctMoveit::handle_accepted(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle)
{
  {
    std::unique_lock<std::mutex> lck(goal_mtx_);
    goals_queue_.push_back(goal_handle);
  }
  print_info("[ACTION] goals_queue_ size: %d", goals_queue_.size());

  if (!goal_thread_open_) {
    print_debug("Open Goal Thread");
    if (goal_thread_.joinable()) {
      goal_thread_.join();
    }
    goal_thread_ = std::thread(&TmRos2SctMoveit::execute_goal_traj, this);
  }
}

rclcpp_action::CancelResponse TmRos2SctMoveit::handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle)
{
  auto goal_id = rclcpp_action::to_string(goal_handle->get_goal_id());
  print_info("[ACTION CANCEL] Cancel request received for goal %s", goal_id.c_str());

  // if the goal is current
  if (goal_handle->is_executing()) {
    iface_.stop_pvt_traj();
    print_info("[ACTION CANCEL] stop_pvt_traj()");
    print_info("[ACTION CANCEL] Goal %s execution canceled", goal_id.c_str());
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // remove the goal from queue
  auto it = std::find(goals_queue_.begin(), goals_queue_.end(), goal_handle);
  if (it != goals_queue_.end()) {
    goals_queue_.erase(it);
    print_info("[ACTION CANCEL] Deferd Goal canceled %s", goal_id.c_str());
    print_info("deferred_goal_handles queue size: %d", goals_queue_.size());
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // the goal not exist
  print_warn("[ACTION CANCEL] Cancel request received for goal %s, but it is neither active nor deferred.", goal_id.c_str());
  return rclcpp_action::CancelResponse::REJECT;
}

void TmRos2SctMoveit::execute_goal_traj()
{
  {
    std::unique_lock<std::mutex> lck(goal_mtx_);
    goal_thread_open_ = true;
  }
  print_info("[ACTION] -----goal execution thread begin-----");
  auto result = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();

  while (!goals_queue_.empty()) {
    auto goal_handle = goals_queue_.front();

    try {
      auto &traj_points = goal_handle->get_goal()->trajectory.points;
      bool is_match_start_pose = is_positions_match(traj_points.front(), 0.01);
      if (!is_match_start_pose) {
        print_warn("[ACTION] Start point doesn't match current pose");
      }

      auto pvts = get_pvt_traj(traj_points, 0.025);
      print_info("[ACTION] traj. total time:= %d", (int)pvts->total_time);
      if (!goal_handle->is_executing()) {
        goal_handle->execute();
        print_info("[ACTION] traj executed");
      }

      if (!is_fake_) {
        iface_.run_pvt_traj(*pvts);  // run traj and wait for done
      }
      else {
        iface_.fake_run_pvt_traj(*pvts);
      }
      // ERROR, and clear goal queue
      if (iface_.is_sct_error) {
        result->error_code = result->INVALID_JOINTS;
        result->error_string = "TM_SCT get Error";
        print_error(result->error_string.c_str());
        iface_.is_sct_error = false;
        goal_handle->abort(result);
        goals_queue_.clear();
        {
          std::unique_lock<std::mutex> lck(goal_mtx_);
          goal_thread_open_ = false;
        }
        print_info("[ACTION] ERROR, clean all goals, goal execution thread end");
        return;
      }
      // CANCEL, and start next goal
      if (goal_handle->is_canceling()) {
        result->error_code = result->INVALID_GOAL;
        result->error_string = "Goal canceled";
        print_info(result->error_string.c_str());
        goal_handle->canceled(result);
        print_info("[ACTION] Current traj execution thread end");
      }
      // NORMAL, and start next goal
      else if (goal_handle->is_active()) {
        bool is_match_final_pose = is_positions_match(traj_points.back(), 0.01);
        if (!is_match_final_pose) {
          result->error_code = result->GOAL_TOLERANCE_VIOLATED;
          result->error_string = "Current pose doesn't match Goal point";
          print_warn(result->error_string.c_str());
          goal_handle->abort(result);
        }
        else
        {
          result->error_code = result->SUCCESSFUL;
          result->error_string = "Goal reached, success!";
          print_info(result->error_string.c_str());
          goal_handle->succeed(result);
        }
      }
    }
    catch (...) {
      result->error_code = result->INVALID_GOAL;
      result->error_string = "Goal fail !";
      goal_handle->abort(result);
    }
    goals_queue_.pop_front();
  }

  {
    std::unique_lock<std::mutex> lck(goal_mtx_);
    goal_thread_open_ = false;
  }
  print_info("[ACTION] -----goal execution thread end-----");
}

void TmRos2SctMoveit::reorder_traj_joints(
    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> &new_traj_points,
    const trajectory_msgs::msg::JointTrajectory& traj)
{
  /* Reorders trajectory - destructive */
  std::vector<size_t> mapping;
  mapping.resize(jns_.size(), jns_.size());
  for (size_t i = 0; i < traj.joint_names.size(); ++i) {
    for (size_t j = 0; j < jns_.size(); ++j) {
      if (traj.joint_names[i] == jns_[j])
        mapping[j] = i;
    }
  }
  new_traj_points.clear();
  for (unsigned int i = 0; i < traj.points.size(); ++i) {
    trajectory_msgs::msg::JointTrajectoryPoint new_point;
    for (unsigned int j = 0; j < traj.points[i].positions.size(); ++j) {
      new_point.positions.push_back(traj.points[i].positions[mapping[j]]);
      new_point.velocities.push_back(traj.points[i].velocities[mapping[j]]);
      if (traj.points[i].accelerations.size() != 0)
        new_point.accelerations.push_back(traj.points[i].accelerations[mapping[j]]);
    }
    new_point.time_from_start = traj.points[i].time_from_start;
    new_traj_points.push_back(new_point);
  }
}

bool TmRos2SctMoveit::has_points(const trajectory_msgs::msg::JointTrajectory &traj)
{
  if (traj.points.size() == 0) return false;
  for (auto &point : traj.points) {
    if (point.positions.size() != traj.joint_names.size() ||
      point.velocities.size() != traj.joint_names.size()) return false;
  }
  return true;
}

bool TmRos2SctMoveit::is_traj_finite(const trajectory_msgs::msg::JointTrajectory &traj)
{
  for (auto &point : traj.points) {
    for (auto &p : point.positions) {
      if (!std::isfinite(p)) return false;
    }
    for (auto &v : point.velocities) {
      if (!std::isfinite(v)) return false;
    }
  }
  return true;
}

bool TmRos2SctMoveit::is_positions_match(
    const trajectory_msgs::msg::JointTrajectoryPoint &point, double eps)
{
  auto q_act = state_.joint_angle();
  for (size_t i = 0; i < point.positions.size(); ++i) {
    if (fabs(point.positions[i] - q_act[i]) > eps)
      return false;
  }
  return true;
}

void TmRos2SctMoveit::set_pvt_traj(
    TmPvtTraj &pvts, const std::vector<trajectory_msgs::msg::JointTrajectoryPoint> &traj_points, double Tmin)
{
  size_t i = 0, i_1 = 0, i_2 = 0;
  int skip_count = 0;
  TmPvtPoint point;

  if (traj_points.size() == 0) return;

  pvts.mode = TmPvtMode::Joint;

  auto sec = [](const builtin_interfaces::msg::Duration& t) {
    return static_cast<double>(t.sec) + 1e-9 * static_cast<double>(t.nanosec);
  };

  // first point
  if (sec(traj_points[i].time_from_start) != 0.0) {
    print_warn("[ACTION] Traj.: first point should be the current position, with time_from_start set to 0.0");
    point.time = sec(traj_points[i].time_from_start);
    point.positions = traj_points[i].positions;
    point.velocities = traj_points[i].velocities;
    pvts.points.push_back(point);
  }
  for (i = 1; i < traj_points.size() - 1; ++i) {
    point.time = sec(traj_points[i].time_from_start) - sec(traj_points[i_1].time_from_start);
    if (point.time >= Tmin) {
      i_2 = i_1;
      i_1 = i;
      point.positions = traj_points[i].positions;
      point.velocities = traj_points[i].velocities;
      pvts.points.push_back(point);
    }
    else
    {
      ++skip_count;
    }
  }
  if (skip_count > 0) {
    print_warn("[ACTION] Traj.: skip %d points", (int)skip_count);
    // RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"),"TM_ROS: Traj.: skip " << (int)skip_count << " points");
  }
  // last point
  if (traj_points.size() > 1) {
    i = traj_points.size() - 1;
    point.time = sec(traj_points[i].time_from_start) - sec(traj_points[i_1].time_from_start);
    point.positions = traj_points[i].positions;
    point.velocities = traj_points[i].velocities;
    if (point.time >= Tmin) {
      pvts.points.push_back(point);
    }
    else
    {
      point.time = sec(traj_points[i].time_from_start) - sec(traj_points[i_2].time_from_start);
      pvts.points.back() = point;
      ++skip_count;
      print_warn("[ACTION] Traj.: skip 1 more last point");
    }
  }
  pvts.total_time = sec(traj_points.back().time_from_start);
}

std::shared_ptr<TmPvtTraj> TmRos2SctMoveit::get_pvt_traj(
    const std::vector<trajectory_msgs::msg::JointTrajectoryPoint> &traj_points, double Tmin)
{
  std::shared_ptr<TmPvtTraj> pvts = std::make_shared<TmPvtTraj>();
  set_pvt_traj(*pvts, traj_points, Tmin);
  return pvts;
}