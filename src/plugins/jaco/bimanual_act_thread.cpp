
/***************************************************************************
 *  bimanual_act_thread.cpp - Jaco plugin act-thread for coordinated bimanual manipulation
 *
 *  Created: Mon Sep 29 03:13:20 2014
 *  Copyright  2014  Bahram Maleki-Fard
 *
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

#include "bimanual_act_thread.h"

#include "bimanual_goto_thread.h"
#include "bimanual_openrave_thread.h"

#include <interfaces/JacoBimanualInterface.h>

using namespace fawkes;

/** @class JacoBimanualActThread "bimanual_act_thread.h"
 * Jaco Arm act-thread for coordinate bimanual manipulation.
 *
 * @author Bahram Maleki-Fard
 */

/** Constructor.
 * @param arms pointer to jaco_dual_arm_t struct, to be used in this thread
 */
JacoBimanualActThread::JacoBimanualActThread(jaco_dual_arm_t *arms)
: Thread("JacoBimanualActThread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT),
  arms_(arms)
{
}

/** Destructor. */
JacoBimanualActThread::~JacoBimanualActThread()
{
}

/** Initialize. */
void
JacoBimanualActThread::init()
{
	// open interface for writing
	arms_->iface = blackboard->open_for_writing<JacoBimanualInterface>("JacoArm Bimanual");
}

/** Finalize. */
void
JacoBimanualActThread::finalize()
{
	arms_->goto_thread     = NULL;
	arms_->openrave_thread = NULL;

	try {
		blackboard->close(arms_->iface);
	} catch (fawkes::Exception &e) {
		logger->log_warn(name(),
		                 "Could not close JacoBimanualInterface interface. Er:%s",
		                 e.what_no_backtrace());
	}
}

/** Main loop. */
void
JacoBimanualActThread::loop()
{
	if (arms_->openrave_thread == NULL || arms_->goto_thread == NULL)
		return;

	while (!arms_->iface->msgq_empty()) {
		Message *m = arms_->iface->msgq_first(m);
		arms_->iface->set_msgid(m->id());
		arms_->iface->set_final(false);
		arms_->iface->set_error_code(JacoBimanualInterface::ERROR_NONE);
		//~ arms_->iface->write();

		if (arms_->iface->msgq_first_is<JacoBimanualInterface::SetPlannerParamsMessage>()) {
			JacoBimanualInterface::SetPlannerParamsMessage *msg = arms_->iface->msgq_first(msg);
			logger->log_debug(name(), "SetPlannerParamsMessage rcvd. params:%s", msg->params());

#ifdef HAVE_OPENRAVE
			arms_->openrave_thread->set_plannerparams(msg->params());
#endif

		} else if (arms_->iface->msgq_first_is<JacoBimanualInterface::SetConstrainedMessage>()) {
			JacoBimanualInterface::SetConstrainedMessage *msg = arms_->iface->msgq_first(msg);
			logger->log_debug(name(), "SetConstrainedMessage rcvd. Enabled:%u", msg->is_constrained());

#ifdef HAVE_OPENRAVE
			arms_->openrave_thread->set_constrained(msg->is_constrained());
#endif

		} else if (arms_->iface->msgq_first_is<JacoBimanualInterface::CartesianGotoMessage>()) {
			JacoBimanualInterface::CartesianGotoMessage *msg = arms_->iface->msgq_first(msg);
			logger->log_debug(name(),
			                  "CartesianGotoMessage rcvd. left: x:%f  y:%f  z:%f  e1:%f  e2:%f  e3:%f",
			                  msg->l_x(),
			                  msg->l_y(),
			                  msg->l_z(),
			                  msg->l_e1(),
			                  msg->l_e2(),
			                  msg->l_e3());
			logger->log_debug(name(),
			                  "CartesianGotoMessage      right: x:%f  y:%f  z:%f  e1:%f  e2:%f  e3:%f",
			                  msg->r_x(),
			                  msg->r_y(),
			                  msg->r_z(),
			                  msg->r_e1(),
			                  msg->r_e2(),
			                  msg->r_e3());
#ifdef HAVE_OPENRAVE
			logger->log_debug(name(),
			                  "CartesianGotoMessage is being passed to openrave",
			                  arms_->iface->id());
			// add target to OpenRAVE queue for planning
			bool s = arms_->openrave_thread->add_target(msg->l_x(),
			                                            msg->l_y(),
			                                            msg->l_z(),
			                                            msg->l_e1(),
			                                            msg->l_e2(),
			                                            msg->l_e3(),
			                                            msg->r_x(),
			                                            msg->r_y(),
			                                            msg->r_z(),
			                                            msg->r_e1(),
			                                            msg->r_e2(),
			                                            msg->r_e3());
			if (!s) {
				arms_->iface->set_error_code(JacoBimanualInterface::ERROR_NO_IK);
				logger->log_warn(name(),
				                 "Failed executing CartesianGotoMessage, could not find IK solution");
			}
#else
			logger->log_warn(name(),
			                 "OpenRAVE not found. Cannot plan coordinated trajectories. Skipping!");
#endif

		} else if (arms_->iface->msgq_first_is<JacoBimanualInterface::MoveGripperMessage>()) {
			JacoBimanualInterface::MoveGripperMessage *msg = arms_->iface->msgq_first(msg);
			logger->log_debug(name(),
			                  "MoveGripperMessage rcvd. left: f1:%f  f2:%f  f3:%f",
			                  msg->l_finger1(),
			                  msg->l_finger2(),
			                  msg->l_finger3());
			logger->log_debug(name(),
			                  "MoveGripperMessage      right: f1:%f  f2:%f  f3:%f",
			                  msg->r_finger1(),
			                  msg->r_finger2(),
			                  msg->r_finger3());

			arms_->goto_thread->move_gripper(msg->l_finger1(),
			                                 msg->l_finger2(),
			                                 msg->l_finger3(),
			                                 msg->r_finger2(),
			                                 msg->r_finger2(),
			                                 msg->r_finger3());

		} else {
			logger->log_warn(name(), "Unknown message received. Skipping");
		}

		arms_->iface->msgq_pop();
	}

	arms_->iface->set_final(arms_->goto_thread->final());
	arms_->iface->write();
}
