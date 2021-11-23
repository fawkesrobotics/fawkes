
/***************************************************************************
 *  skiller_thread.cpp - ROS Action Server to receive skiller commands from ROS
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

#include "skiller_thread.h"

#include <core/threading/mutex_locker.h>
#include <utils/time/time.h>

using namespace fawkes;

/** @class ROS2SkillerThread "skiller_thread.h"
 * Accept skiller commands from ROS.
 * @author Till Hofmann
 */

/** Contructor. */
ROS2SkillerThread::ROS2SkillerThread()
: Thread("ROS2SkillerThread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT)
{
}

void
ROS2SkillerThread::init()
{
	exec_request_ = false;
	exec_running_ = false;
	exec_as_      = false;

	try {
		skiller_if_ = blackboard->open_for_reading<SkillerInterface>("Skiller");
	} catch (const Exception &e) {
		logger->log_error(name(), "Initialization failed, could not open Skiller interface");
		throw;
	}
	using namespace std::placeholders;
	server_ = rclcpp_action::create_server<fawkes_msgs::action::ExecSkill>(
	                            node_handle,
								"skiller_server",
								std::bind(&ROS2SkillerThread::handle_goal, this, _1, _2),
								std::bind(&ROS2SkillerThread::handle_cancel, this, _1),
								std::bind(&ROS2SkillerThread::handle_accepted, this, _1));

	sub_cmd_ = node_handle->create_subscription<std_msgs::msg::String>("skiller",
	                                       1,
	                                       std::bind(&ROS2SkillerThread::message_cb, this, _1));

	pub_status_ = node_handle->create_publisher<fawkes_msgs::msg::SkillStatus>("skiller_status", true);
}

void
ROS2SkillerThread::finalize()
{
	try {
		blackboard->close(skiller_if_);
	} catch (Exception &e) {
		logger->log_error(name(), "Closing interface failed!");
	}
	//delete server_;
}

void
ROS2SkillerThread::once()
{
	//server_->start();
}

void
ROS2SkillerThread::stop()
{
	if (skiller_if_->exclusive_controller() != skiller_if_->serial().get_string()) {
		logger->log_warn(name(), "Skill abortion requested, but currently not in control");
		return;
	}

	if (skiller_if_->has_writer())
		skiller_if_->msgq_enqueue(new SkillerInterface::StopExecMessage());
	if (exec_as_) {
		std::string error_msg = "Abort on request";
		as_goal_->abort(create_result(error_msg));
	}
	skiller_if_->msgq_enqueue(new SkillerInterface::ReleaseControlMessage());
	exec_running_ = false;
}

rclcpp_action::GoalResponse 
ROS2SkillerThread::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const fawkes_msgs::action::ExecSkill::Goal> goal) 
{
	MutexLocker lock(loop_mutex);
	if (exec_running_ && exec_as_) {
		std::string error_msg = "Replaced by new goal";
		as_goal_->abort(create_result(error_msg));
		return rclcpp_action::GoalResponse::REJECT;
	}
	goal_         = goal->skillstring;
	exec_request_ = true;
	exec_as_      = true;

	return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}


rclcpp_action::CancelResponse
ROS2SkillerThread::handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<fawkes_msgs::action::ExecSkill>> goal_handle)
{
	MutexLocker lock(loop_mutex);
	stop();
	std::string error_msg = "Abort on request";
	goal_handle->canceled(create_result(error_msg));
	return rclcpp_action::CancelResponse::ACCEPT;
}

void 
ROS2SkillerThread::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<fawkes_msgs::action::ExecSkill>> goal_handle) 
{
	using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    as_goal_ = goal_handle;
}

void
ROS2SkillerThread::message_cb(const std_msgs::msg::String::SharedPtr goal)
{
	MutexLocker lock(loop_mutex);
	logger->log_info(name(), "Received new goal: '%s'", goal->data.c_str());
	goal_         = goal->data;
	exec_request_ = true;
	exec_as_      = false;
}

std::shared_ptr<fawkes_msgs::action::ExecSkill::Result> 
ROS2SkillerThread::create_result(const std::string &errmsg)
{
	std::shared_ptr<fawkes_msgs::action::ExecSkill::Result> result;
	result->errmsg = errmsg;
	return result;
}

std::shared_ptr<fawkes_msgs::action::ExecSkill::Feedback>
ROS2SkillerThread::create_feedback()
{
	return std::shared_ptr<fawkes_msgs::action::ExecSkill::Feedback>();
}

void
ROS2SkillerThread::loop()
{
	skiller_if_->read();

	// currently idle, release skiller control
	if (!exec_running_ && !exec_request_
	    && skiller_if_->exclusive_controller() == skiller_if_->serial().get_string()) {
		logger->log_debug(name(), "No skill running and no skill requested, releasing control");
		skiller_if_->msgq_enqueue(new SkillerInterface::ReleaseControlMessage());
		return;
	}

	if (exec_request_) {
		if (!skiller_if_->has_writer()) {
			logger->log_warn(name(), "no writer for skiller, cannot execute skill");
			stop();
			return;
		}

		if (skiller_if_->exclusive_controller() != skiller_if_->serial().get_string()) {
			// we need the skiller control, acquire it first
			logger->log_debug(name(), "Skill execution requested, but currently not in control");
			skiller_if_->msgq_enqueue(new SkillerInterface::AcquireControlMessage());
			return;
		}
		exec_request_ = false;

		SkillerInterface::ExecSkillMessage *msg = new SkillerInterface::ExecSkillMessage(goal_.c_str());
		msg->ref();

		logger->log_debug(name(), "Creating goal '%s'", goal_.c_str());

		try {
			skiller_if_->msgq_enqueue(msg);
			exec_running_      = true;
			exec_msgid_        = msg->id();
			exec_skill_string_ = msg->skill_string();
			loops_waited_      = 0;
		} catch (Exception &e) {
			logger->log_warn(name(), "Failed to execute skill, exception follows");
			logger->log_warn(name(), e);
		}
		msg->unref();

	} else if (exec_running_) {
		if (exec_as_)
			as_goal_->publish_feedback(create_feedback());

		if (skiller_if_->status() == SkillerInterface::S_INACTIVE
		    || skiller_if_->msgid() != exec_msgid_) {
			// wait three loops, maybe the skiller will start
			logger->log_debug(name(), "Should be executing skill, but skiller is inactive");
			++loops_waited_;
			if (loops_waited_ >= 3) {
				// give up and abort
				logger->log_warn(name(), "Skiller doesn't start, aborting");
				std::string error_msg = "Skiller doesn't start";
				if (exec_as_)
					as_goal_->abort(create_result(error_msg));
				exec_running_ = false;
			}
		} else if (skiller_if_->status() != SkillerInterface::S_RUNNING) {
			exec_running_ = false;
			if (exec_as_ && exec_skill_string_ == skiller_if_->skill_string()) {
				if (skiller_if_->status() == SkillerInterface::S_FINAL) {
					std::string error_msg = "Skill executed";
					as_goal_->succeed(create_result(error_msg));
				} else if (skiller_if_->status() == SkillerInterface::S_FAILED) {
					std::string error_msg = "Failed to execute skill";
					char *      tmp;
					if (asprintf(&tmp, "Failed to execute skill, error: %s", skiller_if_->error()) != -1) {
						error_msg = tmp;
						free(tmp);
					}
					as_goal_->canceled(create_result(""));
				}
			}
		}
	}

	if (skiller_if_->refreshed()) {
		fawkes_msgs::msg::SkillStatus msg;
		const Time *             time = skiller_if_->timestamp();
		msg.stamp                     = rclcpp::Time(time->get_sec(), time->get_nsec());
		msg.skill_string              = skiller_if_->skill_string();
		msg.error                     = skiller_if_->error();
		msg.status                    = skiller_if_->status();
		pub_status_->publish(msg);
	}
}
