
/***************************************************************************
 *  navigator_thread.cpp - Robotino ROS Navigator Thread
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

/** @class RosNavigatorThread "navigator_thread.h"
 * Send Fawkes locomotion commands off to ROS.
 * @author Sebastian Reuter
 */

/** Constructor.
 * @param cfg_prefix configuration prefix specific for the ros/navigator
 */
RosNavigatorThread::RosNavigatorThread(std::string &cfg_prefix)
  : Thread("RosNavigatorThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT),
    cfg_prefix_(cfg_prefix)
{
}

void
RosNavigatorThread::init()
{
  // navigator interface to give feedback of navigation task (writer)
  try {
    nav_if_ = blackboard->open_for_writing<NavigatorInterface>("Navigator");
  } catch (Exception& e) {
    e.append("%s initialization failed, could not open navigator "
             "interface for writing", name());
    logger->log_error(name(), e);
    throw;
  }

  //tell the action client that we want to spin a thread by default
  ac_ = new MoveBaseClient("move_base", false);
  

  cmd_sent_ = false;
  connected_history_ = false;
  nav_if_->set_final(true);
  nav_if_->write();
  load_config();

  ac_init_checktime_ = new fawkes::Time(clock);
}

void
RosNavigatorThread::finalize()
{
  // close interfaces
  try {
    blackboard->close(nav_if_);
  } catch (Exception& e) {
    logger->log_error(name(), "Closing interface failed!");
    logger->log_error(name(), e);
  }
  delete ac_;
  delete ac_init_checktime_;
}

void
RosNavigatorThread::check_status()
{
  bool write = false;
  if (rosnode->hasParam(cfg_dynreconf_path_ + "/" + cfg_dynreconf_x_vel_name_)) {
      rosnode->getParam(cfg_dynreconf_path_ + "/" + cfg_dynreconf_x_vel_name_, param_max_vel);
      nav_if_->set_max_velocity(param_max_vel);
      write = true;
  }
  if (rosnode->hasParam(cfg_dynreconf_path_ + "/" + cfg_dynreconf_rot_vel_name_)) {
      rosnode->getParam(cfg_dynreconf_path_ + "/" + cfg_dynreconf_rot_vel_name_, param_max_rot);
      nav_if_->set_max_rotation(param_max_rot);
      write = true;
  }
  
  if (cmd_sent_){

    if (ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED || 
        ac_->getState() == actionlib::SimpleClientGoalState::PREEMPTED) {
      
      nav_if_->set_final(true);
      
      // Check if we reached the goal
      fawkes::tf::Quaternion q_base_rotation;
      q_base_rotation.setX(base_position.pose.orientation.x);
      q_base_rotation.setY(base_position.pose.orientation.y);
      q_base_rotation.setZ(base_position.pose.orientation.z);
      q_base_rotation.setW(base_position.pose.orientation.w);

      double base_position_x = base_position.pose.position.x;
      double base_position_y = base_position.pose.position.y;
      double base_position_yaw = fawkes::tf::get_yaw(q_base_rotation);

      double diff_x = fabs(base_position_x - goal_position_x);
      double diff_y = fabs(base_position_y - goal_position_y);
      double diff_yaw = normalize_mirror_rad(base_position_yaw - goal_position_yaw);

      if (diff_x >= cfg_trans_tolerance_ ||
          diff_y >= cfg_trans_tolerance_ ||
          diff_yaw >= cfg_ori_tolerance_) {
          nav_if_->set_error_code(NavigatorInterface::ERROR_OBSTRUCTION);
      } else {
          nav_if_->set_error_code(NavigatorInterface::ERROR_NONE);
      }
      nav_if_->write();
    }
    else if (
             ac_->getState() == actionlib::SimpleClientGoalState::LOST)
    {
      nav_if_->set_final(true);
      nav_if_->set_error_code(NavigatorInterface::ERROR_UNKNOWN_PLACE);
    }
    else if (ac_->getState() == actionlib::SimpleClientGoalState::ABORTED ||
             ac_->getState() == actionlib::SimpleClientGoalState::REJECTED)
    {
      nav_if_->set_final(true);
      nav_if_->set_error_code(NavigatorInterface::ERROR_PATH_GEN_FAIL);
    }
    else {
      nav_if_->set_final(false);
      nav_if_->set_error_code(0);
    }
    write = true;
  }
  if (write) nav_if_->write();
}

void
RosNavigatorThread::send_goal()
{
  //get goal from Navigation interface
  goal_.target_pose.header.frame_id = nav_if_->target_frame();
  goal_.target_pose.header.stamp = ros::Time::now();
  goal_.target_pose.pose.position.x = nav_if_->dest_x();
  goal_.target_pose.pose.position.y = nav_if_->dest_y();
  //move_base discards goals with an invalid quaternion
  fawkes::tf::Quaternion q(std::isfinite(nav_if_->dest_ori()) ?
                           nav_if_->dest_ori() : 0.0, 0, 0);
  goal_.target_pose.pose.orientation.x = q.x();
  goal_.target_pose.pose.orientation.y = q.y();
  goal_.target_pose.pose.orientation.z = q.z();
  goal_.target_pose.pose.orientation.w = q.w();

  ac_->sendGoal(goal_,
                boost::bind(&RosNavigatorThread::doneCb, this, _1, _2),
                boost::bind(&RosNavigatorThread::activeCb, this),
                boost::bind(&RosNavigatorThread::feedbackCb, this, _1));
      
  cmd_sent_ = true;
}

// Called once when the goal becomes active
void
RosNavigatorThread::activeCb()
{
}

// Called every time feedback is received for the goal
void
RosNavigatorThread::feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
  base_position = feedback->base_position;
}

void
RosNavigatorThread::doneCb(const actionlib::SimpleClientGoalState& state,
            const move_base_msgs::MoveBaseResultConstPtr& result)
{
  logger->log_info(name(), "Finished in state [%s]", state.toString().c_str());
}

bool
RosNavigatorThread::set_dynreconf_value(const std::string& path, const float value)
{
  dynreconf_double_param.name = path;
  dynreconf_double_param.value = value;
  dynreconf_conf.doubles.push_back(dynreconf_double_param);
  dynreconf_srv_req.config = dynreconf_conf;

  if (! ros::service::call(cfg_dynreconf_path_ + "/set_parameters", dynreconf_srv_req, dynreconf_srv_resp)) {
      logger->log_error(name(), "Error in setting dynreconf value %s to %f in path %s", path.c_str(), value, cfg_dynreconf_path_.c_str());
      dynreconf_conf.doubles.clear();
      return false;
  } else {
      logger->log_info(name(), "Setting %s to %f", path.c_str(), value);
      dynreconf_conf.doubles.clear();
      return true;
  }
}

void
RosNavigatorThread::stop_goals()
{
  //cancel all existing goals and send Goal={0/0/0}
  ac_->cancelAllGoals();
}

void
RosNavigatorThread::loop()
{
  if (! ac_->isServerConnected()) {
	  fawkes::Time now(clock);
	  if (now - ac_init_checktime_ >= 5.0) {
		  // action client never connected, yet. Re-create to avoid stale client.
		  delete ac_;
		  ac_ = new MoveBaseClient("move_base", false);
		  ac_init_checktime_->stamp();
	  }
    if (! nav_if_->msgq_empty()) {
      logger->log_warn(name(), "Command received while ROS ActionClient "
		       "not reachable, ignoring");
      nav_if_->msgq_flush();
    }

    if (connected_history_){
      delete ac_;
      ac_ = new MoveBaseClient("move_base", false);
      connected_history_ = false;
    }

  } else {
    connected_history_ = true;
    // process incoming messages from fawkes
    while (! nav_if_->msgq_empty()) {

      // stop
      if (NavigatorInterface::StopMessage *msg = nav_if_->msgq_first_safe(msg)) {
	      if (msg->msgid() == 0 || msg->msgid() == nav_if_->msgid()) {
		      logger->log_info(name(), "Stop message received");
		      nav_if_->set_dest_x(0);
		      nav_if_->set_dest_y(0);
		      nav_if_->set_dest_ori(0);

		      nav_if_->set_msgid(msg->id());
		      nav_if_->write();

		      stop_goals();
	      } else {
		      logger->log_warn(name(), "Received stop message for non-active command "
		                       "(got %u, running %u)", msg->msgid(), nav_if_->msgid());
	      }
      }

      // cartesian goto
      else if (NavigatorInterface::CartesianGotoMessage *msg = nav_if_->msgq_first_safe(msg)) {
        logger->log_info(name(), "Cartesian goto message received (x,y,ori) = (%f,%f,%f)",
                         msg->x(), msg->y(), std::isfinite(msg->orientation()) ?
                           msg->orientation() : 0.0);
        nav_if_->set_dest_x(msg->x());
        nav_if_->set_dest_y(msg->y());
        nav_if_->set_dest_ori(msg->orientation());
        nav_if_->set_target_frame("base_link");

        nav_if_->set_msgid(msg->id());

        nav_if_->write();

        goal_position_x = nav_if_->dest_x();
        goal_position_y = nav_if_->dest_y();
        goal_position_yaw = nav_if_->dest_ori();

        // Transform the desired goal position into the fixed frame
        // so we can check whether we reached the goal or not
        if (strcmp (cfg_fixed_frame_.c_str(),nav_if_->target_frame()) != 0) {
            transform_to_fixed_frame();
        }

        send_goal();
      }

      // cartesian goto
      else if (NavigatorInterface::CartesianGotoWithFrameMessage *msg = nav_if_->msgq_first_safe(msg)) {
        logger->log_info(name(), "Cartesian goto message received (x,y,ori) = (%f,%f,%f)",
                         msg->x(), msg->y(), std::isfinite(msg->orientation()) ?
                           msg->orientation() : 0.0);
        nav_if_->set_dest_x(msg->x());
        nav_if_->set_dest_y(msg->y());
        nav_if_->set_dest_ori(msg->orientation());
        nav_if_->set_target_frame(msg->target_frame());

        nav_if_->set_msgid(msg->id());

        nav_if_->write();

        goal_position_x = nav_if_->dest_x();
        goal_position_y = nav_if_->dest_y();
        goal_position_yaw = nav_if_->dest_ori();

        // Transform the desired goal position into the fixed frame
        // so we can check whether we reached the goal or not
        if (strcmp (cfg_fixed_frame_.c_str(),nav_if_->target_frame()) != 0) {
            transform_to_fixed_frame();
        }

        send_goal();
      }

      // polar goto
      else if (NavigatorInterface::PolarGotoMessage *msg = nav_if_->msgq_first_safe(msg)) {
        logger->log_info(name(), "Polar goto message received (phi,dist) = (%f,%f)",
                         msg->phi(), msg->dist());
        nav_if_->set_dest_x(msg->dist() * cos(msg->phi()));
        nav_if_->set_dest_y(msg->dist() * cos(msg->phi()));
        nav_if_->set_dest_ori(msg->phi());
        nav_if_->set_msgid(msg->id());
        nav_if_->write();

        send_goal();
      }

      // max velocity
      else if (NavigatorInterface::SetMaxVelocityMessage *msg = nav_if_->msgq_first_safe(msg)) {
        logger->log_info(name(),"velocity message received %f",msg->max_velocity());
        nav_if_->set_max_velocity(msg->max_velocity());
        nav_if_->set_msgid(msg->id());
        nav_if_->write();

        set_dynreconf_value(cfg_dynreconf_x_vel_name_, msg->max_velocity());
        set_dynreconf_value(cfg_dynreconf_y_vel_name_, msg->max_velocity());

        send_goal();
      }

      // max rotation velocity
      else if (NavigatorInterface::SetMaxRotationMessage *msg = nav_if_->msgq_first_safe(msg)) {
        logger->log_info(name(),"rotation message received %f",msg->max_rotation());
        nav_if_->set_max_rotation(msg->max_rotation());
        nav_if_->set_msgid(msg->id());
        nav_if_->write();

        set_dynreconf_value(cfg_dynreconf_rot_vel_name_, msg->max_rotation());

        send_goal();
      }

      else if (NavigatorInterface::SetSecurityDistanceMessage *msg =
	         nav_if_->msgq_first_safe(msg))
      {
        logger->log_info(name(),"velocity message received %f",msg->security_distance ());
        nav_if_->set_security_distance (msg->security_distance ());
        nav_if_->set_msgid(msg->id());
        nav_if_->write();

        send_goal();
      }

      nav_if_->msgq_pop();
    } // while

    check_status();
  }
}

void
RosNavigatorThread::load_config()
{
    try {
        cfg_dynreconf_path_ = config->get_string(cfg_prefix_ + "/dynreconf/path");
        cfg_dynreconf_x_vel_name_ =
                config->get_string(cfg_prefix_ + "/dynreconf/x_vel_name");
        cfg_dynreconf_y_vel_name_ =
                config->get_string(cfg_prefix_ + "/dynreconf/y_vel_name");
        cfg_dynreconf_rot_vel_name_ =
                config->get_string(cfg_prefix_ + "/dynreconf/rot_vel_name");
        cfg_fixed_frame_ =
                config->get_string("/frames/fixed");
        cfg_ori_tolerance_ =
                config->get_float(cfg_prefix_ + "/ori_tolerance");
        cfg_trans_tolerance_ =
                config->get_float(cfg_prefix_ + "/trans_tolerance");

        logger->log_info(name(), "fixed frame: %s", cfg_fixed_frame_.c_str());

    } catch (Exception &e) {
        logger->log_error(name(), "Error in loading config: %s", e.what());
    }
}

void
RosNavigatorThread::transform_to_fixed_frame()
{
    fawkes::tf::Quaternion goal_q = fawkes::tf::create_quaternion_from_yaw(nav_if_->dest_ori());
    fawkes::tf::Point goal_p(nav_if_->dest_x(), nav_if_->dest_y(), 0.);
    fawkes::tf::Pose goal_pos(goal_q, goal_p);
    fawkes::tf::Stamped<fawkes::tf::Pose> goal_pos_stamped(goal_pos, fawkes::Time(0,0), nav_if_->target_frame());
    fawkes::tf::Stamped<fawkes::tf::Pose> goal_pos_stamped_transformed;

    try {
        tf_listener->transform_pose(cfg_fixed_frame_, goal_pos_stamped, goal_pos_stamped_transformed);
    } catch (fawkes::Exception &e) {
    }

    goal_position_x = goal_pos_stamped_transformed.getOrigin().getX();
    goal_position_y = goal_pos_stamped_transformed.getOrigin().getY();
    goal_position_yaw = fawkes::tf::get_yaw(goal_pos_stamped_transformed.getRotation());
}
