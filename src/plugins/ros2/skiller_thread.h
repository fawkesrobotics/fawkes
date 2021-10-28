
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

#ifndef _ROS_SKILLER_THREAD_H_
#define _ROS_SKILLER_THREAD_H_

#include <actionlib/server/simple_action_server.h>
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <core/threading/thread.h>
#include <fawkes_msgs/ExecSkillAction.h>
#include <fawkes_msgs/ExecSkillActionGoal.h>
#include <fawkes_msgs/ExecSkillGoal.h>
#include <interfaces/SkillerInterface.h>
#include <plugins/ros/aspect/ros.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <string>

namespace fawkes {
class SkillerInterface;
}

class RosSkillerThread : public fawkes::Thread,
                         public fawkes::BlockedTimingAspect,
                         public fawkes::LoggingAspect,
                         public fawkes::BlackBoardAspect,
                         public fawkes::ConfigurableAspect,
                         public fawkes::ROSAspect
{
public:
	RosSkillerThread();

	/* thread */
	virtual void init();
	virtual void finalize();
	virtual void once();
	virtual void loop();

private:
	typedef actionlib::ActionServer<fawkes_msgs::ExecSkillAction> SkillerServer;

	void action_goal_cb(SkillerServer::GoalHandle goal);
	void action_cancel_cb(SkillerServer::GoalHandle goal);
	void message_cb(const std_msgs::String::ConstPtr &goal);

	void                           stop();
	fawkes_msgs::ExecSkillResult   create_result(const std::string &errmsg);
	fawkes_msgs::ExecSkillFeedback create_feedback();

	bool assure_control();
	void release_control();

private:
	fawkes::SkillerInterface *skiller_if_;

	SkillerServer * server_;
	ros::Subscriber sub_cmd_;
	ros::Publisher  pub_status_;

	SkillerServer::GoalHandle as_goal_;
	std::string               goal_;

	bool         exec_as_;
	bool         exec_request_;
	bool         exec_running_;
	unsigned int exec_msgid_;
	std::string  exec_skill_string_;
	unsigned int loops_waited_;
};

#endif /* ROS_SKILLER_THREAD_H__ */
