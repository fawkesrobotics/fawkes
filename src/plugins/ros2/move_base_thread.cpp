
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

#include "move_base_thread.h"

#include <core/threading/mutex_locker.h>

using namespace fawkes;

/** @class ROS2MoveBaseThread "move_base_thread.h"
 * Accept locomotion commands from ROS (emulate move_base).
 * @author Sebastian Reuter
 */

/** Contructor. */
ROS2MoveBaseThread::ROS2MoveBaseThread()
: Thread("ROS2MoveBaseThread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT),
  TransformAspect(TransformAspect::ONLY_LISTENER)
{
}

void
ROS2MoveBaseThread::init()
{
	exec_request_ = false;
	exec_running_ = false;
	exec_as_      = false;
	exec_type_    = NAVGRAPH;

	cfg_base_frame_  = config->get_string("/frames/base");
	cfg_fixed_frame_ = config->get_string("/frames/fixed");

	// basic sanity check, test if a node named move_base has been launched
	// of course it might have a different name, but try at least the obvious
	ros::V_string nodes;
	if (ros::master::getNodes(nodes)) {
		for (size_t i = 0; i < nodes.size(); ++i) {
			if (nodes[i] == "/move_base" || nodes[i] == "/move_base_node") {
				throw Exception("move_base appears to be already running"
				                " (node %s detected)",
				                nodes[i].c_str());
			}
		}
	}

	try {
		nav_navgraph_if_ = blackboard->open_for_reading<NavigatorInterface>("Pathplan");
		nav_colli_if_    = blackboard->open_for_reading<NavigatorInterface>("Navigator");
	} catch (Exception &e) {
		e.append("%s initialization failed, could not open navigator "
		         "interface for reading",
		         name());
		logger->log_error(name(), e);
		throw;
	}

	as_colli_ = new MoveBaseServer(**rosnode,
	                               "move_base_colli",
	                               boost::bind(&ROS2MoveBaseThread::action_goal_cb, this, _1, COLLI),
	                               boost::bind(&ROS2MoveBaseThread::action_cancel_cb, this, _1),
	                               /* auto_start */ false);

	as_navgraph_ =
	  new MoveBaseServer(**rosnode,
	                     "move_base",
	                     boost::bind(&ROS2MoveBaseThread::action_goal_cb, this, _1, NAVGRAPH),
	                     boost::bind(&ROS2MoveBaseThread::action_cancel_cb, this, _1),
	                     /* auto_start */ false);

	sub_colli_ = rosnode->subscribe<geometry_msgs::PoseStamped>(
	  "move_base_simple/goal_colli", 1, boost::bind(&ROS2MoveBaseThread::message_cb, this, _1, COLLI));
	sub_navgraph_ = rosnode->subscribe<geometry_msgs::PoseStamped>(
	  "move_base_simple/goal", 1, boost::bind(&ROS2MoveBaseThread::message_cb, this, _1, NAVGRAPH));
}

void
ROS2MoveBaseThread::finalize()
{
	try {
		blackboard->close(nav_navgraph_if_);
		blackboard->close(nav_colli_if_);
	} catch (Exception &e) {
		logger->log_error(name(), "Closing interface failed!");
		logger->log_error(name(), e);
	}
	delete as_navgraph_;
	delete as_colli_;
}

void
ROS2MoveBaseThread::once()
{
	as_navgraph_->start();
	as_colli_->start();
}

void
ROS2MoveBaseThread::stop()
{
	NavigatorInterface::StopMessage *msg = new NavigatorInterface::StopMessage();
	if (exec_type_ == NAVGRAPH) {
		if (nav_navgraph_if_->has_writer())
			nav_navgraph_if_->msgq_enqueue(msg);
	} else {
		if (nav_colli_if_->has_writer())
			nav_colli_if_->msgq_enqueue(msg);
	}
	if (exec_as_) {
		as_goal_.setAborted(create_result());
	}
	exec_running_ = false;
}

void
ROS2MoveBaseThread::action_goal_cb(MoveBaseServer::GoalHandle goal, ExecType ext)
{
	MutexLocker lock(loop_mutex);
	if (exec_running_ && exec_as_) {
		as_goal_.setAborted(create_result(), "Replaced by new goal");
	}
	as_goal_      = goal;
	goal_pose_    = goal.getGoal()->target_pose;
	exec_type_    = ext;
	exec_request_ = true;
	exec_as_      = true;

	goal.setAccepted();
}

void
ROS2MoveBaseThread::action_cancel_cb(MoveBaseServer::GoalHandle goal)
{
	MutexLocker lock(loop_mutex);
	stop();
	goal.setCanceled(create_result(), "Abort on request");
}

void
ROS2MoveBaseThread::message_cb(geometry_msgs::PoseStampedConstPtr goal_pose, ExecType ext)
{
	MutexLocker lock(loop_mutex);
	goal_pose_    = *goal_pose;
	exec_type_    = ext;
	exec_request_ = true;
	exec_as_      = false;
}

move_base_msgs::MoveBaseResult
ROS2MoveBaseThread::create_result()
{
	return move_base_msgs::MoveBaseResult();
}

move_base_msgs::MoveBaseFeedback
ROS2MoveBaseThread::create_feedback()
{
	move_base_msgs::MoveBaseFeedback feedback;

	// origin in base frame
	tf::Stamped<tf::Pose> transform_pose_in;
	transform_pose_in.frame_id = cfg_base_frame_;
	tf::Stamped<tf::Pose> transform_pose_out;

	try {
		tf_listener->transform_pose(cfg_fixed_frame_, transform_pose_in, transform_pose_out);

		feedback.base_position.header.stamp =
		  ros::Time(transform_pose_out.stamp.get_sec(), transform_pose_out.stamp.get_nsec());
		feedback.base_position.header.frame_id    = cfg_fixed_frame_;
		feedback.base_position.pose.orientation.x = transform_pose_out.getRotation().x();
		feedback.base_position.pose.orientation.y = transform_pose_out.getRotation().y();
		feedback.base_position.pose.orientation.z = transform_pose_out.getRotation().z();
		feedback.base_position.pose.orientation.w = transform_pose_out.getRotation().w();
		feedback.base_position.pose.position.x    = transform_pose_out.getOrigin().x();
		feedback.base_position.pose.position.y    = transform_pose_out.getOrigin().y();
		feedback.base_position.pose.position.z    = transform_pose_out.getOrigin().z();
	} catch (Exception &e) {
		logger->log_warn(name(), "Failed to determine fixed frame pose");
	}

	return feedback;
}

void
ROS2MoveBaseThread::loop()
{
	if (exec_request_) {
		exec_request_ = false;

		if (exec_type_ == NAVGRAPH && !nav_navgraph_if_->has_writer()) {
			logger->log_warn(name(), "No writer for navgraph, cannot move to goal");
			stop();
			return;
		} else if (exec_type_ == COLLI && !nav_colli_if_->has_writer()) {
			logger->log_warn(name(), "No writer for navigator, cannot move to goal");
			stop();
			return;
		}

		std::string target_frame;
		if (exec_type_ == COLLI) {
			logger->log_info(name(), "Running with colli");
			target_frame = cfg_base_frame_;
			nav_if_      = nav_colli_if_;
		} else if (exec_type_ == NAVGRAPH) {
			logger->log_info(name(), "Running with navgraph");
			target_frame = cfg_fixed_frame_;
			nav_if_      = nav_navgraph_if_;
		} else {
			// This should really never happen, if it does, someone fucked up our memory
			logger->log_warn(name(), "Internal error, invalid execution type");
			return;
		}

		fawkes::Time goal_time(0, 0);
		// transform pose to target frame
		tf::Stamped<tf::Pose> target_pose_in(
		  tf::Transform(tf::Quaternion(goal_pose_.pose.orientation.x,
		                               goal_pose_.pose.orientation.y,
		                               goal_pose_.pose.orientation.z,
		                               goal_pose_.pose.orientation.w),
		                tf::Vector3(goal_pose_.pose.position.x,
		                            goal_pose_.pose.position.y,
		                            goal_pose_.pose.position.z)),
		  goal_time,
		  goal_pose_.header.frame_id);

		tf::Stamped<tf::Pose> target_pose_out;

		try {
			tf_listener->transform_pose(target_frame, target_pose_in, target_pose_out);
		} catch (Exception &e) {
			logger->log_warn(name(),
			                 "Failed to transform target pose (%s -> %s), cannot move",
			                 target_pose_in.frame_id.c_str(),
			                 target_frame.c_str());
			logger->log_warn(name(), e);
			stop();
			return;
		}

		tf::Vector3 &pos = target_pose_out.getOrigin();

		NavigatorInterface::CartesianGotoMessage *msg =
		  new NavigatorInterface::CartesianGotoMessage(/* target_frame.c_str(), */ pos.x(),
		                                               pos.y(),
		                                               tf::get_yaw(target_pose_out.getRotation()));

		logger->log_info(name(),
		                 "Goto (%f,%f,%f) in %s",
		                 msg->x(),
		                 msg->y(),
		                 msg->orientation(),
		                 target_frame.c_str());

		try {
			nav_if_->msgq_enqueue(msg);
			exec_running_ = true;
			exec_msgid_   = msg->id();
		} catch (Exception &e) {
			logger->log_warn(name(), "Failed to enqueue cartesian goto, exception follows");
			logger->log_warn(name(), e);
			stop();
		}
	}

	if (exec_running_) {
		nav_if_->read();

		if (exec_as_)
			as_goal_.publishFeedback(create_feedback());

		if (nav_if_->msgid() == exec_msgid_ && nav_if_->is_final()) {
			exec_running_ = false;
			if (exec_as_) {
				if (nav_if_->error_code() == 0) {
					logger->log_info(name(), "Target reached");
					as_goal_.setSucceeded(create_result(), "Target reached");
				} else {
					logger->log_info(name(), "Failed to reach target");
					as_goal_.setAborted(create_result(), "Failed to reach target");
				}
			}
		}
	}
}
