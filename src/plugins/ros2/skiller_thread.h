
/***************************************************************************
 *  skiller_thread.h - ROS Action Server to receive skiller commands from ROS
 *
 *  Created: Fri Jun 27 12:02:42 2014
 *  Copyright  2014  Till Hofmann
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

#ifndef _ROS2_SKILLER_THREAD_H_
#define _ROS2_SKILLER_THREAD_H_

#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <core/threading/thread.h>
#include <fawkes_msgs/action/exec_skill.hpp>
#include <fawkes_msgs/msg/skill_status.hpp>
#include <interfaces/SkillerInterface.h>
#include <plugins/ros2/aspect/ros2.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp_action/rclcpp_action.hpp>


#include <string>

namespace fawkes {
class SkillerInterface;
}

class ROS2SkillerThread : public fawkes::Thread,
                         public fawkes::BlockedTimingAspect,
                         public fawkes::LoggingAspect,
                         public fawkes::BlackBoardAspect,
                         public fawkes::ConfigurableAspect,
                         public fawkes::ROS2Aspect
{
public:
	ROS2SkillerThread();

	/* thread */
	virtual void init();
	virtual void finalize();
	virtual void once();
	virtual void loop();

private:	
	rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const fawkes_msgs::action::ExecSkill::Goal> goal);
	rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<fawkes_msgs::action::ExecSkill>> goal_handle);
	void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<fawkes_msgs::action::ExecSkill>> goal_handle);

	void message_cb(const std_msgs::msg::String::SharedPtr goal);

	void                           stop();
	std::shared_ptr<fawkes_msgs::action::ExecSkill::Result>   create_result(const std::string &errmsg);
	std::shared_ptr<fawkes_msgs::action::ExecSkill::Feedback> create_feedback();

	bool assure_control();
	void release_control();

private:
	fawkes::SkillerInterface *skiller_if_;

	rclcpp_action::Server<fawkes_msgs::action::ExecSkill>::SharedPtr server_;
	std::shared_ptr<rclcpp_action::ServerGoalHandle<fawkes_msgs::action::ExecSkill>> as_goal_;

	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_cmd_;
	rclcpp::Publisher<fawkes_msgs::msg::SkillStatus>::SharedPtr  pub_status_;

	


	std::string               goal_;

	bool         exec_as_;
	bool         exec_request_;
	bool         exec_running_;
	unsigned int exec_msgid_;
	std::string  exec_skill_string_;
	unsigned int loops_waited_;
};

#endif /* ROS_SKILLER_THREAD_H__ */
