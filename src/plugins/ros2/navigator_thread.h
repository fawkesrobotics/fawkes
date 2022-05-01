
/***************************************************************************
 *  navigator_thread.h - Robotino ROS2 Navigator Thread
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

#ifndef _ROS2_NAVIGATOR_THREAD_H_
#define _ROS2_NAVIGATOR_THREAD_H_

#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/tf.h>
#include <core/threading/thread.h>
#include <interfaces/NavigatorInterface.h>
#include <interfaces/Position3DInterface.h>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <plugins/ros2/aspect/ros2.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
//#include <tf2/types.h>
#include <tf/types.h>
#include <utils/math/angle.h>
#include <unistd.h>

#include <math.h>

namespace fawkes {
class NavigatorInterface;
}

class ROS2NavigatorThread : public fawkes::Thread,
                           public fawkes::ClockAspect,
//                           public fawkes::BlockedTimingAspect,
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
//	bool set_dynreconf_value(const std::string &path, const float value);

private:
	using NavigateToPose = nav2_msgs::action::NavigateToPose;
	using GoalHandleNav2 = rclcpp_action::ClientGoalHandle<NavigateToPose>;
	typedef rclcpp_action::Client<NavigateToPose>::SharedPtr Nav2Client;

//	rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;

	void responseCb(std::shared_future<rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr> future);
	void feedbackCb(rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr,
				const std::shared_ptr<const NavigateToPose::Feedback> feedback);
	void resultCb(const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult &result);

	void transform_to_fixed_frame();

	fawkes::NavigatorInterface *	nav_if_;
	fawkes::Position3DInterface *	pose_if_;
	Nav2Client			ac_;
	NavigateToPose::Goal		goal_;
	GoalHandleNav2::SharedPtr	goal_handle_;
	bool				cmd_sent_;
	bool				connected_history_;
	rclcpp::Clock			ros_clock_;

	fawkes::Time *ac_init_checktime_;

	std::string cfg_prefix_;

//	// ROS2 dynamic reconfigure parts
//	dynamic_reconfigure::ReconfigureRequest  dynreconf_srv_req;
//	dynamic_reconfigure::ReconfigureResponse dynreconf_srv_resp;
//	dynamic_reconfigure::DoubleParameter     dynreconf_double_param;
//	dynamic_reconfigure::Config              dynreconf_conf;

//	std::string cfg_dynreconf_path_;
//	std::string cfg_dynreconf_trans_vel_name_;
//	std::string cfg_dynreconf_rot_vel_name_;

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

#endif /* ROS2_NAVIGATOR_THREAD_H__ */
