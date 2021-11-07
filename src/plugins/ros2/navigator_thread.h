
/***************************************************************************
 *  navigator_thread.h - Robotino ROS Navigator Thread
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

#ifndef _ROS_NAVIGATOR_THREAD_H_
#define _ROS_NAVIGATOR_THREAD_H_

#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/tf.h>
#include <core/threading/thread.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <interfaces/NavigatorInterface.h>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <plugins/ros2/aspect/ros2.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2/types.h>
#include <utils/math/angle.h>

#include <math.h>

namespace fawkes {
class NavigatorInterface;
}

class ROS2NavigatorThread : public fawkes::Thread,
                           public fawkes::ClockAspect,
                           public fawkes::BlockedTimingAspect,
                           public fawkes::LoggingAspect,
                           public fawkes::BlackBoardAspect,
                           public fawkes::ConfigurableAspect,
                           public fawkes::ROS2Aspect,
                           public fawkes::TransformAspect
{
public:
	ROS2NavigatorThread(std::string &cfg_prefix);

	virtual void init();
	virtual void finalize();
	virtual void loop();

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
protected:
	virtual void
	run()
	{
		Thread::run();
	}

private:
	void check_status();
	void send_goal();
	void stop_goals();
	void load_config();
	bool set_dynreconf_value(const std::string &path, const float value);

private:


	void responseCb(std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>::SharedPtr> future);
	void feedbackCb(rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>::SharedPtr,
				const std::shared_ptr<const nav2_msgs::action::ComputePathToPose::Feedback> feedback);
	void resultCb(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>::WrappedResult &result);

	void transform_to_fixed_frame();

	fawkes::NavigatorInterface * nav_if_;
	rclcpp_action::Client<nav2_msgs::action::ComputePathToPose>::SharedPtr ac_;
	rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>::SharedPtr cgh_;
	bool                         cmd_sent_;
	bool                         connected_history_;

	fawkes::Time *ac_init_checktime_;

	std::string cfg_prefix_;

	// ROS dynamic reconfigure parts
	dynamic_reconfigure::ReconfigureRequest  dynreconf_srv_req;
	dynamic_reconfigure::ReconfigureResponse dynreconf_srv_resp;
	dynamic_reconfigure::DoubleParameter     dynreconf_double_param;
	dynamic_reconfigure::Config              dynreconf_conf;

	std::string cfg_dynreconf_path_;
	std::string cfg_dynreconf_trans_vel_name_;
	std::string cfg_dynreconf_rot_vel_name_;

	std::string cfg_fixed_frame_;
	float       cfg_ori_tolerance_;
	float       cfg_trans_tolerance_;

	rclcpp::Parameter param_max_vel;
	rclcpp::Parameter param_max_rot;

	geometry_msgs::msg::PoseStamped base_position;
	float                      goal_position_x;
	float                      goal_position_y;
	float                      goal_position_yaw;

	float goal_tolerance_trans;
	float goal_tolerance_yaw;
};

#endif /* ROS_NAVIGATOR_THREAD_H__ */
