
/***************************************************************************
 *  navgraph_breakout_thread.cpp - Provide navgraph-like API through ROS
 *
 *  Created: Fri Jan 27 11:35:39 2017
 *  Copyright  2017  Tim Niemueller
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

#include "navgraph_breakout_thread.h"

#include <interfaces/NavigatorInterface.h>

using namespace fawkes;

/** @class RosNavgraphBreakoutThread "navigator_thread.h"
 * Provide navgraph-like API through ROS.
 * @author Tim Niemueller
 */

/** Contructor. */
RosNavgraphBreakoutThread::RosNavgraphBreakoutThread()
: Thread("RosNavgraphBreakoutThread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT)
{
}

void
RosNavgraphBreakoutThread::init()
{
	goal_active_   = false;
	was_connected_ = false;

	try {
		pp_nav_if_ = blackboard->open_for_writing<NavigatorInterface>("Pathplan");
	} catch (Exception &e) {
		e.append("%s initialization failed, could not open navigator "
		         "interface for writing",
		         name());
		logger->log_error(name(), e);
		throw;
	}

	cfg_action_topic_ = config->get_string("/ros/navgraph-breakout/action-topic");

	ac_ = new NavGraphGotoClient(cfg_action_topic_, false);
}

void
RosNavgraphBreakoutThread::finalize()
{
	try {
		blackboard->close(pp_nav_if_);
	} catch (Exception &e) {
		logger->log_error(name(), "Closing interface failed!");
		logger->log_error(name(), e);
	}
	delete ac_;
}

void
RosNavgraphBreakoutThread::loop()
{
	if (!ac_->isServerConnected()) {
		if (!pp_nav_if_->msgq_empty()) {
			logger->log_warn(name(),
			                 "Command received while action provider "
			                 "not available, ignoring");
			pp_nav_if_->msgq_flush();
		}

		if (was_connected_) {
			delete ac_;
			ac_            = new NavGraphGotoClient(cfg_action_topic_, false);
			was_connected_ = false;
		}

	} else {
		was_connected_ = true;

		// Check for new incoming commands and process them
		while (!pp_nav_if_->msgq_empty()) {
			if (NavigatorInterface::StopMessage *msg = pp_nav_if_->msgq_first_safe(msg)) {
				if (goal_active_) {
					ac_->cancelAllGoals();
					goal_active_ = false;
				}
			} else if (NavigatorInterface::PlaceGotoMessage *msg = pp_nav_if_->msgq_first_safe(msg)) {
				logger->log_info(name(), "Relaying place goto for %s", msg->place());

				pp_nav_if_->set_msgid(msg->id());

				goal_.place       = msg->place();
				goal_.orientation = std::numeric_limits<float>::quiet_NaN();
				if (goal_active_) {
					ac_->cancelAllGoals();
				}
				ac_->sendGoal(goal_);

				pp_nav_if_->set_final(false);
				pp_nav_if_->set_error_code(0);
				pp_nav_if_->write();
				goal_active_ = true;
			}

			pp_nav_if_->msgq_pop();
		}

		// If there is an active goal, check for its completion
		if (goal_active_) {
			if (ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
				fawkes_msgs::NavGraphGotoResult result = *(ac_->getResult());
				if (result.errcode != fawkes_msgs::NavGraphGotoResult::ERROR_NONE) {
					if (result.errmsg.empty()) {
						logger->log_warn(name(),
						                 "Remote navgraph goto failed without error message (code %u)",
						                 result.errcode);
					} else {
						logger->log_warn(name(), "Remote navgraph goto failed: %s", result.errmsg.c_str());
					}
				}
				pp_nav_if_->set_final(true);
				pp_nav_if_->set_error_code(result.errcode);
				pp_nav_if_->write();
				goal_active_ = false;
			} else if (ac_->getState() == actionlib::SimpleClientGoalState::ABORTED
			           || ac_->getState() == actionlib::SimpleClientGoalState::REJECTED) {
				pp_nav_if_->set_final(true);
				pp_nav_if_->set_error_code(NavigatorInterface::ERROR_OBSTRUCTION);
				pp_nav_if_->write();
				goal_active_ = false;
			}
		}
	}
}
