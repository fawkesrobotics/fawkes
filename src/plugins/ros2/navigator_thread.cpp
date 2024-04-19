
/***************************************************************************
 *  navigator_thread.cpp - Robotino ROS2 Navigator Thread
 *
 *  Created: Sat June 09 15:13:27 2012
 *  Copyright  2012  Sebastian Reuter
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include "navigator_thread.h"

using namespace fawkes;

/** @class ROS2NavigatorThread "navigator_thread.h"
 * Send Fawkes locomotion commands off to ROS.
 * @author Sebastian Reuter
 */

/** Constructor.
 * @param cfg_prefix configuration prefix specific for the ros/navigator
 */
ROS2NavigatorThread::ROS2NavigatorThread(std::string &cfg_prefix)
: Thread("ROS2NavigatorThread", Thread::OPMODE_CONTINUOUS),
  //  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT),
  cfg_prefix_(cfg_prefix)
{
}

void
ROS2NavigatorThread::init()
{
	// navigator interface to give feedback of navigation task (writer)
	try {
		nav_if_ = blackboard->open_for_writing<NavigatorInterface>("Navigator");
	} catch (Exception &e) {
		e.append("%s initialization failed, could not open navigator "
		         "interface for writing",
		         name());
		logger->log_error(name(), e);
		throw;
	}
	try {
		pose_if_ = blackboard->open_for_reading<Position3DInterface>("Pose");
	} catch (Exception &e) {
		e.append("%s initialization failed, could not open pose "
		         "interface for reading",
		         name());
		logger->log_error(name(), e);
		throw;
	}

	//tell the action client that we want to spin a thread by default
	ac_ = rclcpp_action::create_client<NavigateToPose>(node_handle, "navigate_to_pose");

	connected_history_ = false;
	//nav_if_->set_final(false);
	//nav_if_->write();
	load_config();

	ac_init_checktime_ = new fawkes::Time(clock);

	//	node_handle->declare_parameter(cfg_dynreconf_path_ + "/" + cfg_dynreconf_trans_vel_name_);
	//	node_handle->declare_parameter(cfg_dynreconf_path_ + "/" + cfg_dynreconf_rot_vel_name_);
}

void
ROS2NavigatorThread::finalize()
{
	// close interfaces
	try {
		blackboard->close(nav_if_);
	} catch (Exception &e) {
		logger->log_error(name(), "Closing interface failed!");
		logger->log_error(name(), e);
	}
	//	delete ac_;
	delete ac_init_checktime_;
}

void
ROS2NavigatorThread::send_goal()
{
	// reset interface
	nav_if_->set_final(false);
	nav_if_->set_error_code(0);
	nav_if_->write();
	logger->log_info(name(), "Send a new goal!");
	//  stop_goals();
	auto goal_ = NavigateToPose::Goal();
	result_    = rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult();
	//get goal from Navigation interface
	goal_.pose.header.frame_id = nav_if_->target_frame();
	goal_.pose.header.stamp    = node_handle->get_clock()->now();
	goal_.pose.pose.position.x = nav_if_->dest_x();
	goal_.pose.pose.position.y = nav_if_->dest_y();
	//move_base discards goals with an invalid quaternion
	fawkes::tf::Quaternion q(std::isfinite(nav_if_->dest_ori()) ? nav_if_->dest_ori() : 0.0, 0, 0);
	goal_.pose.pose.orientation.x = q.x();
	goal_.pose.pose.orientation.y = q.y();
	goal_.pose.pose.orientation.z = q.z();
	goal_.pose.pose.orientation.w = q.w();

	auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
	send_goal_options.result_callback =
	  [this](const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult result) {
		  logger->log_info(name(), "Goal finished and we got the callback we wanted");
		  if (!future_goal_handle_.valid()) {
			  logger->log_error(name(), "But the Goal Handle is invalid");
			  nav_if_->set_final(true);
			  nav_if_->set_error_code(NavigatorInterface::ERROR_UNKNOWN_PLACE);
			  nav_if_->write();
			  return;
		  }
		  auto goal_handle = future_goal_handle_.get();
		  for (std::size_t i = 0; i < UUID_SIZE; i++) {
			  if (result.goal_id[i] != goal_handle->get_goal_id()[i]) {
				  logger->log_error(name(), "Goal ID missmatch!!!");
				  nav_if_->set_final(true);
				  auto new_goal = NavigateToPose::Goal();
				  new_goal.pose.header.frame_id = "map";
				  new_goal.pose.header.stamp    = node_handle->get_clock()->now();
				  new_goal.pose.pose.position.x = nav_if_->dest_x();
				  new_goal.pose.pose.position.y = nav_if_->dest_y();
				  auto empty_opt = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
				  nav_if_->set_error_code(NavigatorInterface::ERROR_UNKNOWN_PLACE);
			          ac_->async_send_goal(new_goal, empty_opt);
				  nav_if_->write();
				  return;
			  }
		  }
		  switch (result.code) {
		  case rclcpp_action::ResultCode::SUCCEEDED: {
			  if (!goal_handle) {
				  throw std::runtime_error("Goal was rejected by the action server");
			  }
			  pose_if_->read();

			  //  		logger->log_info(name(), "Finished successfully");

			  // Check if we reached the goal
			  fawkes::tf::Quaternion q_base_rotation;
			  q_base_rotation.setX(pose_if_->rotation(0));
			  q_base_rotation.setY(pose_if_->rotation(1));
			  q_base_rotation.setZ(pose_if_->rotation(2));
			  q_base_rotation.setW(pose_if_->rotation(3));

			  double base_position_x   = pose_if_->translation(0);
			  double base_position_y   = pose_if_->translation(1);
			  double base_position_yaw = fawkes::tf::get_yaw(q_base_rotation);

			  double diff_x   = fabs(base_position_x - goal_position_x);
			  double diff_y   = fabs(base_position_y - goal_position_y);
			  double diff_yaw = normalize_mirror_rad(base_position_yaw - goal_position_yaw);

			  // Check if we reached the goal
			  //fawkes::tf::Quaternion q_base_rotation;
			  //q_base_rotation.setX(base_position.pose.orientation.x);
			  //q_base_rotation.setY(base_position.pose.orientation.y);
			  //q_base_rotation.setZ(base_position.pose.orientation.z);
			  //q_base_rotation.setW(base_position.pose.orientation.w);

			  //double base_position_x   = base_position.pose.position.x;
			  //double base_position_y   = base_position.pose.position.y;
			  //double base_position_yaw = fawkes::tf::get_yaw(q_base_rotation);

			  //double diff_x   = fabs(base_position_x - goal_position_x);
			  //double diff_y   = fabs(base_position_y - goal_position_y);
			  //double diff_yaw = normalize_mirror_rad(base_position_yaw - goal_position_yaw);
			  //if (diff_x >= goal_tolerance_trans || diff_y >= goal_tolerance_trans
			  //    || diff_yaw >= goal_tolerance_yaw) {
			  //} else {
			  nav_if_->set_final(true);
			  nav_if_->set_error_code(NavigatorInterface::ERROR_NONE);
			  nav_if_->write();
			  //}
			  std::stringstream ss;
			  for (std::size_t i = 0; i < 16; i++) {
				  if (i != 0) {
					  ss << ", ";
				  }
				  ss << "0x" << std::hex << static_cast<int>(result.goal_id[i]);
			  }
			  logger->log_warn(name(), ("Succeeded goal with id: " + ss.str()).c_str());

			  auto val = result.result;
		  } break;

		  case rclcpp_action::ResultCode::ABORTED: {
			  nav_if_->set_final(true);
			  nav_if_->set_error_code(NavigatorInterface::ERROR_PATH_GEN_FAIL);
			  nav_if_->write();
			  std::stringstream ss;
			  for (std::size_t i = 0; i < 16; i++) {
				  if (i != 0) {
					  ss << ", ";
				  }
				  ss << "0x" << std::hex << static_cast<int>(result.goal_id[i]);
			  }
			  logger->log_warn(name(), ("Aborted goal with id: " + ss.str()).c_str());

			  auto val = result.result;

		  } break;

		  case rclcpp_action::ResultCode::CANCELED: {
			  nav_if_->set_final(true);
			  nav_if_->set_error_code(NavigatorInterface::ERROR_UNKNOWN_PLACE);
			  nav_if_->write();
			  std::stringstream ss;
			  for (std::size_t i = 0; i < 16; i++) {
				  if (i != 0) {
					  ss << ", ";
				  }
				  ss << "0x" << std::hex << static_cast<int>(result.goal_id[i]);
			  }
			  logger->log_warn(name(), ("Canceled goal with id: " + ss.str()).c_str());

			  auto val = result.result;
		  } break;
		  default: {
			  std::stringstream ss;
			  for (std::size_t i = 0; i < 16; i++) {
				  if (i != 0) {
					  ss << ", ";
				  }
				  ss << "0x" << std::hex << static_cast<int>(result.goal_id[i]);
			  }
			  logger->log_error(name(), "Unknown result code received");
			  nav_if_->set_final(true);
			  nav_if_->set_error_code(NavigatorInterface::ERROR_UNKNOWN_PLACE);
			  nav_if_->write();
		  } break;
		  }
		  //logger->log_info(name(), "Goal DONE");
	  };
	send_goal_options.goal_response_callback =
	  [this](
	    std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>> handle) {
		  if (!handle) {
			  logger->log_error(name(), "goal handle rejected by server");
			  return;
		  } else {
			  logger->log_info(name(), "goal handle accepted by server");
		  }
	  };
	send_goal_options.feedback_callback =
	  [this](rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr,
	         const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
		  base_position = feedback->current_pose;
	  };
	future_goal_handle_ = ac_->async_send_goal(goal_, send_goal_options);
}

void
ROS2NavigatorThread::stop_goals()
{
	//cancel all existing goals and send Goal={0/0/0}
	logger->log_warn(name(), "stop goals called");
	auto future_canceled = ac_->async_cancel_all_goals();
	future_canceled.wait();
	nav_if_->set_final(true);
	nav_if_->write();
	logger->log_warn(name(), "stop goals finished");
}

void
ROS2NavigatorThread::loop()
{
	if (!ac_->action_server_is_ready()) {
		logger->log_warn(name(), "Action server not ready");
		fawkes::Time now(clock);
		if (now - ac_init_checktime_ >= 5.0) {
			// action client never connected, yet. Re-create to avoid stale client.
			//			delete ac_;
			// TODO: check for actual name of "nav2" actionserver
			ac_ = rclcpp_action::create_client<NavigateToPose>(node_handle, "navigate_to_pose");
			//			ac_ = new Nav2Client(node_handle, "nav2");
			ac_init_checktime_->stamp();
		}
		if (!nav_if_->msgq_empty()) {
			logger->log_warn(name(),
			                 "Command received while ROS2 ActionClient "
			                 "not reachable, ignoring");
			nav_if_->set_error_code(NavigatorInterface::ERROR_PATH_GEN_FAIL);
			nav_if_->write();
			nav_if_->msgq_flush();
		}

		if (connected_history_) {
			//			delete ac_;
			ac_ = rclcpp_action::create_client<NavigateToPose>(node_handle, "navigate_to_pose");
			//			ac_                = new Nav2Client(node_handle, "nav2");
			//			ac_                = new MoveBaseClient("move_base", false);
			logger->log_warn(name(), "History disconnectd");
			connected_history_ = false;
		}

	} else {
		connected_history_ = true;
		// process incoming messages from fawkes
		while (!nav_if_->msgq_empty()) {
			// stop
			if (NavigatorInterface::StopMessage *msg = nav_if_->msgq_first_safe(msg)) {
				if (msg->msgid() == 0 || msg->msgid() == nav_if_->msgid()) {
					logger->log_warn(name(), "Stop message received");
					nav_if_->set_dest_x(0);
					nav_if_->set_dest_y(0);
					nav_if_->set_dest_ori(0);

					nav_if_->set_msgid(msg->id());
					nav_if_->write();

					stop_goals();
				} else {
					logger->log_warn(name(),
					                 "Received stop message for non-active command "
					                 "(got %u, running %u)",
					                 msg->msgid(),
					                 nav_if_->msgid());
				}
			}

			// cartesian goto
			else if (NavigatorInterface::CartesianGotoMessage *msg = nav_if_->msgq_first_safe(msg)) {
				logger->log_info(name(),
				                 "Cartesian goto message received (x,y,ori) = (%f,%f,%f)",
				                 msg->x(),
				                 msg->y(),
				                 std::isfinite(msg->orientation()) ? msg->orientation() : 0.0);
				nav_if_->set_dest_x(msg->x());
				nav_if_->set_dest_y(msg->y());
				nav_if_->set_dest_ori(msg->orientation());
				nav_if_->set_target_frame("base_link");

				nav_if_->set_msgid(msg->id());

				nav_if_->write();

				goal_position_x   = nav_if_->dest_x();
				goal_position_y   = nav_if_->dest_y();
				goal_position_yaw = nav_if_->dest_ori();

				goal_tolerance_trans = cfg_trans_tolerance_;
				goal_tolerance_yaw   = cfg_ori_tolerance_;

				// Transform the desired goal position into the fixed frame
				// so we can check whether we reached the goal or not
				if (strcmp(cfg_fixed_frame_.c_str(), nav_if_->target_frame()) != 0) {
					transform_to_fixed_frame();
				}

				send_goal();
			}

			// cartesian goto
			else if (NavigatorInterface::CartesianGotoWithFrameMessage *msg =
			           nav_if_->msgq_first_safe(msg)) {
				logger->log_info(name(),
				                 "Cartesian goto message received (x,y,ori) = (%f,%f,%f)",
				                 msg->x(),
				                 msg->y(),
				                 std::isfinite(msg->orientation()) ? msg->orientation() : 0.0);
				nav_if_->set_dest_x(msg->x());
				nav_if_->set_dest_y(msg->y());
				nav_if_->set_dest_ori(msg->orientation());
				nav_if_->set_target_frame(msg->target_frame());

				nav_if_->set_msgid(msg->id());

				nav_if_->write();

				goal_position_x   = nav_if_->dest_x();
				goal_position_y   = nav_if_->dest_y();
				goal_position_yaw = nav_if_->dest_ori();

				goal_tolerance_trans = cfg_trans_tolerance_;
				goal_tolerance_yaw   = cfg_ori_tolerance_;

				// Transform the desired goal position into the fixed frame
				// so we can check whether we reached the goal or not
				if (strcmp(cfg_fixed_frame_.c_str(), nav_if_->target_frame()) != 0) {
					transform_to_fixed_frame();
				}

				send_goal();
			}

			// cartesian goto with tolerance
			else if (NavigatorInterface::CartesianGotoWithToleranceMessage *msg =
			           nav_if_->msgq_first_safe(msg)) {
				logger->log_info(name(),
				                 "Cartesian goto with tolerance message received "
				                 "(x,y,ori,trans_tolerance,ori_tolerance) = (%f,%f,%f,%f,%f)",
				                 msg->x(),
				                 msg->y(),
				                 std::isfinite(msg->orientation()) ? msg->orientation() : 0.0,
				                 msg->translation_tolerance(),
				                 msg->orientation_tolerance());
				nav_if_->set_dest_x(msg->x());
				nav_if_->set_dest_y(msg->y());
				nav_if_->set_dest_ori(msg->orientation());
				nav_if_->set_target_frame("base_link");

				nav_if_->set_msgid(msg->id());

				nav_if_->write();

				goal_position_x   = nav_if_->dest_x();
				goal_position_y   = nav_if_->dest_y();
				goal_position_yaw = nav_if_->dest_ori();

				goal_tolerance_trans = msg->translation_tolerance();
				goal_tolerance_yaw   = msg->orientation_tolerance();

				// Transform the desired goal position into the fixed frame
				// so we can check whether we reached the goal or not
				if (strcmp(cfg_fixed_frame_.c_str(), nav_if_->target_frame()) != 0) {
					transform_to_fixed_frame();
				}

				send_goal();
			}

			// cartesian goto with frame and tolerance
			else if (NavigatorInterface::CartesianGotoWithFrameWithToleranceMessage *msg =
			           nav_if_->msgq_first_safe(msg)) {
				logger->log_info(name(),
				                 "Cartesian goto with tolerance message received "
				                 "(x,y,ori,trans_tolerance,ori_tolerance) = (%f,%f,%f,%f,%f)",
				                 msg->x(),
				                 msg->y(),
				                 std::isfinite(msg->orientation()) ? msg->orientation() : 0.0,
				                 msg->translation_tolerance(),
				                 msg->orientation_tolerance());
				nav_if_->set_dest_x(msg->x());
				nav_if_->set_dest_y(msg->y());
				nav_if_->set_dest_ori(msg->orientation());
				nav_if_->set_target_frame(msg->target_frame());

				nav_if_->set_msgid(msg->id());

				nav_if_->write();

				goal_position_x   = nav_if_->dest_x();
				goal_position_y   = nav_if_->dest_y();
				goal_position_yaw = nav_if_->dest_ori();

				goal_tolerance_trans = msg->translation_tolerance();
				goal_tolerance_yaw   = msg->orientation_tolerance();

				// Transform the desired goal position into the fixed frame
				// so we can check whether we reached the goal or not
				if (strcmp(cfg_fixed_frame_.c_str(), nav_if_->target_frame()) != 0) {
					transform_to_fixed_frame();
				}

				send_goal();
			}

			// polar goto
			else if (NavigatorInterface::PolarGotoMessage *msg = nav_if_->msgq_first_safe(msg)) {
				logger->log_info(name(),
				                 "Polar goto message received (phi,dist) = (%f,%f)",
				                 msg->phi(),
				                 msg->dist());
				nav_if_->set_dest_x(msg->dist() * cos(msg->phi()));
				nav_if_->set_dest_y(msg->dist() * cos(msg->phi()));
				nav_if_->set_dest_ori(msg->phi());
				nav_if_->set_msgid(msg->id());
				nav_if_->write();

				goal_tolerance_trans = cfg_trans_tolerance_;
				goal_tolerance_yaw   = cfg_ori_tolerance_;

				send_goal();
			}

			// max velocity
			else if (NavigatorInterface::SetMaxVelocityMessage *msg = nav_if_->msgq_first_safe(msg)) {
				logger->log_info(name(), "velocity message received %f", msg->max_velocity());
				nav_if_->set_max_velocity(msg->max_velocity());
				nav_if_->set_msgid(msg->id());
				nav_if_->write();

				//				set_dynreconf_value(cfg_dynreconf_trans_vel_name_, msg->max_velocity());

				send_goal();
			}

			// max rotation velocity
			else if (NavigatorInterface::SetMaxRotationMessage *msg = nav_if_->msgq_first_safe(msg)) {
				logger->log_info(name(), "rotation message received %f", msg->max_rotation());
				nav_if_->set_max_rotation(msg->max_rotation());
				nav_if_->set_msgid(msg->id());
				nav_if_->write();

				//				set_dynreconf_value(cfg_dynreconf_rot_vel_name_, msg->max_rotation());

				send_goal();
			}

			else if (NavigatorInterface::SetSecurityDistanceMessage *msg =
			           nav_if_->msgq_first_safe(msg)) {
				logger->log_info(name(), "velocity message received %f", msg->security_distance());
				nav_if_->set_security_distance(msg->security_distance());
				nav_if_->set_msgid(msg->id());
				nav_if_->write();

				send_goal();
			}

			nav_if_->msgq_pop();
		} // while
	}
	usleep(100000);
}

void
ROS2NavigatorThread::load_config()
{
	try {
		//		cfg_dynreconf_path_           = config->get_string(cfg_prefix_ + "/dynreconf/path");
		//		cfg_dynreconf_trans_vel_name_ = config->get_string(cfg_prefix_ + "/dynreconf/trans_vel_name");
		//		cfg_dynreconf_rot_vel_name_   = config->get_string(cfg_prefix_ + "/dynreconf/rot_vel_name");
		cfg_fixed_frame_     = config->get_string("/frames/fixed");
		cfg_ori_tolerance_   = config->get_float(cfg_prefix_ + "/ori_tolerance");
		cfg_trans_tolerance_ = config->get_float(cfg_prefix_ + "/trans_tolerance");

		logger->log_info(name(), "fixed frame: %s", cfg_fixed_frame_.c_str());

	} catch (Exception &e) {
		logger->log_error(name(), "Error in loading config: %s", e.what());
	}
}

void
ROS2NavigatorThread::transform_to_fixed_frame()
{
	fawkes::tf::Quaternion goal_q = fawkes::tf::create_quaternion_from_yaw(nav_if_->dest_ori());
	fawkes::tf::Point      goal_p(nav_if_->dest_x(), nav_if_->dest_y(), 0.);
	fawkes::tf::Pose       goal_pos(goal_q, goal_p);
	fawkes::tf::Stamped<fawkes::tf::Pose> goal_pos_stamped(goal_pos,
	                                                       fawkes::Time(0, 0),
	                                                       nav_if_->target_frame());
	fawkes::tf::Stamped<fawkes::tf::Pose> goal_pos_stamped_transformed;

	try {
		tf_listener->transform_pose(cfg_fixed_frame_, goal_pos_stamped, goal_pos_stamped_transformed);
	} catch (fawkes::Exception &e) {
	}

	goal_position_x   = goal_pos_stamped_transformed.getOrigin().getX();
	goal_position_y   = goal_pos_stamped_transformed.getOrigin().getY();
	goal_position_yaw = fawkes::tf::get_yaw(goal_pos_stamped_transformed.getRotation());
}
