
/***************************************************************************
 *  act_thread.cpp - Kinova Jaco plugin act-thread
 *
 *  Created: Tue Jun 04 13:13:20 2013
 *  Copyright  2013  Bahram Maleki-Fard
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

#include "act_thread.h"

#include "arm_dummy.h"
#include "arm_kindrv.h"
#include "goto_thread.h"
#include "openrave_thread.h"
#include "types.h"

#include <core/utils/refptr.h>
#include <interfaces/JacoInterface.h>

using namespace fawkes;

/** @class JacoActThread "act_thread.h"
 * Jaco Arm control thread.
 *
 * @author Bahram Maleki-Fard
 */

/** Constructor.
 * @param name thread name
 * @param arm pointer to jaco_arm_t struct, to be used in this thread
 */
JacoActThread::JacoActThread(const char *name, jaco_arm_t *arm)
: Thread(name, Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT)
{
	arm_        = arm;
	arm_->arm   = NULL;
	arm_->iface = NULL;

	arm_->goto_thread     = NULL;
	arm_->openrave_thread = NULL;
}

/** Destructor. */
JacoActThread::~JacoActThread()
{
}

/** Initialize.
 * Depending on single or dual_arm setup (defined by config flag),
 * appropriate arms are loaded and then initialized if required to.
 * This method also sets the correct function pointers that are used in
 * the main loop() method. */
void
JacoActThread::init()
{
	cfg_auto_init_  = config->get_bool("/hardware/jaco/auto_initialize");
	cfg_auto_calib_ = config->get_bool("/hardware/jaco/auto_calibrate");

	std::string cfg_arm = config->get_string("/hardware/jaco/arm");

	if (cfg_arm.compare("libkindrv") && cfg_arm.compare("dummy"))
		throw fawkes::Exception("Bad config entry /hardware/jaco/arm '%s'", cfg_arm.c_str());

	std::string arm_name, arm_iface;
	switch (arm_->config) {
	case CONFIG_SINGLE:
		arm_name  = config->get_string("/hardware/jaco/config/single/name");
		arm_iface = config->get_string("/hardware/jaco/config/single/interface");
		break;

	case CONFIG_LEFT:
		arm_name  = config->get_string("/hardware/jaco/config/left/name");
		arm_iface = config->get_string("/hardware/jaco/config/left/interface");
		break;

	case CONFIG_RIGHT:
		arm_name  = config->get_string("/hardware/jaco/config/right/name");
		arm_iface = config->get_string("/hardware/jaco/config/right/interface");
		break;

	default: throw fawkes::Exception("Unknown arm config given"); break;
	}

	// create the JacoArm object and assign correctly to config
	try {
		if (!cfg_arm.compare("dummy")) {
			arm_->arm = new JacoArmDummy("JacoDummy");
		} else {
			arm_->arm = new JacoArmKindrv(arm_name.c_str());
		}
	} catch (fawkes::Exception &e) {
		logger->log_error(name(), "Could not connect to JacoArm. Exception follows.");
		throw;
	}

	// open interface for writing
	try {
		arm_->iface = blackboard->open_for_writing<JacoInterface>(arm_iface.c_str());
	} catch (fawkes::Exception &e) {
		logger->log_error(name(),
		                  "Could not open interface %s for writing. Exception follows.",
		                  arm_iface.c_str());
		delete arm_->arm;
		arm_ = NULL;
		throw;
	}

	// create target/trajectory queues and mutexes
	arm_->target_mutex = RefPtr<Mutex>(new Mutex());
	arm_->trajec_mutex = RefPtr<Mutex>(new Mutex());
	arm_->target_queue = RefPtr<jaco_target_queue_t>(new jaco_target_queue_t());

	// set trajectory colors (TODO: configurable)
	arm_->trajec_color[0] = 0.f;
	arm_->trajec_color[1] = 0.f;
	arm_->trajec_color[2] = 1.f;
	arm_->trajec_color[3] = 1.f;
	if (arm_->config == CONFIG_RIGHT) {
		arm_->trajec_color[0] = 1.f;
		arm_->trajec_color[2] = 0.f;
	}

	// initalize arm
	_initialize();
}

/** Finalize.
 * Close all writing interfaces and delete JacoArm instances.
 */
void
JacoActThread::finalize()
{
	try {
		blackboard->close(arm_->iface);
	} catch (fawkes::Exception &e) {
		logger->log_warn(name(),
		                 "Could not close JacoInterface interface. Er:%s",
		                 e.what_no_backtrace());
	}

	delete arm_->arm;
}

/** Main loop.
 * The structure is pretty obvious. We first submit changes made to the interface
 * from threads before the ACT-hook (they might be used by other threads lateron).
 * Then we make sure the arm is initialized, before processing incoming messages
 * and submiting interface changes once again.
 */
void
JacoActThread::loop()
{
	if (arm_ == NULL || arm_->iface == NULL || arm_->openrave_thread == NULL)
		return;

	// firts of all, submit interface updates (that other threads might have done)!
	arm_->iface->write();

	// check if still initializing
	if (_is_initializing())
		return;

#ifdef HAVE_OPENRAVE
	// make sure openrave-thread is ready!
	if (!arm_->openrave_thread->started())
		return;
#endif

	// process incoming interface messages
	_process_msgs();

	// finally, again submit interface updates
	arm_->iface->write();

#ifdef HAVE_OPENRAVE
	arm_->openrave_thread->update_openrave();
#endif
	arm_->iface->set_final(arm_->goto_thread->final());
}

/* ##########################################################################
 *  private methods , referenced to by the function pointers from loop().
 *
 *  we have one for each single_arm and dual_arm setup.
 * ########################################################################## */
/** Initialize and/or calibrate single arm, if requested by config flags */
void
JacoActThread::_initialize()
{
	//check if we need to initialize arm
	if (!arm_->arm->initialized() && cfg_auto_init_) {
		logger->log_debug(name(), "Initializing arm, wait until finished");
		arm_->arm->initialize();
		arm_->iface->set_final(false);
		//arm_.goto_thread->pos_ready();

	} else if (arm_->arm->initialized() && cfg_auto_calib_) {
		arm_->goto_thread->pos_ready();
	}

	arm_->iface->set_initialized(arm_->arm->initialized());
	arm_->iface->write();
}

/** Check if arm is being initialized. */
bool
JacoActThread::_is_initializing()
{
	arm_->iface->set_initialized(arm_->arm->initialized());

	if (!arm_->arm->initialized() && cfg_auto_init_) {
		logger->log_debug(name(), "wait for arm to initialize");
		//arm_->initialized = arm_->iface->is_final();
		return true;
	}

	return false;
}

/** Process interface messages. */
void
JacoActThread::_process_msgs()
{
	while (!arm_->iface->msgq_empty()) {
		Message *m = arm_->iface->msgq_first(m);
		arm_->iface->set_msgid(m->id());
		arm_->iface->set_final(false);
		arm_->iface->set_error_code(JacoInterface::ERROR_NONE);
		arm_->iface->write();

		if (arm_->iface->msgq_first_is<JacoInterface::StopMessage>()) {
			JacoInterface::StopMessage *msg = arm_->iface->msgq_first(msg);
			logger->log_debug(name(), "%s: StopMessage rcvd", arm_->iface->id());

			arm_->goto_thread->stop();

		} else if (arm_->iface->msgq_first_is<JacoInterface::CalibrateMessage>()) {
			JacoInterface::CalibrateMessage *msg = arm_->iface->msgq_first(msg);
			logger->log_debug(name(), "%s: CalibrateMessage rcvd", arm_->iface->id());

			// Stop all (current and planned) motion. Then calibrate
			arm_->goto_thread->stop();
			arm_->goto_thread->pos_ready();

		} else if (arm_->iface->msgq_first_is<JacoInterface::RetractMessage>()) {
			JacoInterface::RetractMessage *msg = arm_->iface->msgq_first(msg);
			logger->log_debug(name(), "%s: RetractMessage rcvd", arm_->iface->id());

			// Stop all (current and planned) motion. Then retract
			arm_->goto_thread->stop();
			arm_->goto_thread->pos_retract();

		} else if (arm_->iface->msgq_first_is<JacoInterface::SetPlannerParamsMessage>()) {
			JacoInterface::SetPlannerParamsMessage *msg = arm_->iface->msgq_first(msg);
			logger->log_debug(name(),
			                  "%s: SetPlannerParamsMessage rcvd. params:%s",
			                  arm_->iface->id(),
			                  msg->params());

#ifdef HAVE_OPENRAVE
			arm_->openrave_thread->set_plannerparams(msg->params());
#endif

		} else if (arm_->iface->msgq_first_is<JacoInterface::CartesianGotoMessage>()) {
			JacoInterface::CartesianGotoMessage *msg = arm_->iface->msgq_first(msg);
			logger->log_debug(name(),
			                  "%s: CartesianGotoMessage rcvd. x:%f  y:%f  z:%f  e1:%f  e2:%f  e3:%f",
			                  arm_->iface->id(),
			                  msg->x(),
			                  msg->y(),
			                  msg->z(),
			                  msg->e1(),
			                  msg->e2(),
			                  msg->e3());
#ifdef HAVE_OPENRAVE
			logger->log_debug(name(),
			                  "%s: CartesianGotoMessage is being passed to openrave",
			                  arm_->iface->id());
			// add target to OpenRAVE queue for planning
			bool solvable = arm_->openrave_thread->add_target(
			  msg->x(), msg->y(), msg->z(), msg->e1(), msg->e2(), msg->e3());
			if (!solvable) {
				arm_->iface->set_error_code(JacoInterface::ERROR_NO_IK);
				logger->log_warn(name(),
				                 "Failed executing CartesianGotoMessage, arm %s and/or thread %s could not "
				                 "find IK solution",
				                 arm_->arm->get_name().c_str(),
				                 arm_->openrave_thread->name());
			}
#else
			arm_->goto_thread->set_target(msg->x(), msg->y(), msg->z(), msg->e1(), msg->e2(), msg->e3());
#endif

		} else if (arm_->iface->msgq_first_is<JacoInterface::AngularGotoMessage>()) {
			JacoInterface::AngularGotoMessage *msg = arm_->iface->msgq_first(msg);

			logger->log_debug(name(),
			                  "%s: AngularGotoMessage rcvd. x:%f  y:%f  z:%f  e1:%f  e2:%f  e3:%f",
			                  arm_->iface->id(),
			                  msg->j1(),
			                  msg->j2(),
			                  msg->j3(),
			                  msg->j4(),
			                  msg->j5(),
			                  msg->j6());
#ifdef HAVE_OPENRAVE
			logger->log_debug(name(),
			                  "%s: AngularGotoMessage is being passed to openrave",
			                  arm_->iface->id());
			// add target to OpenRAVE queue for planning
			bool joints_valid = arm_->openrave_thread->add_target_ang(
			  msg->j1(), msg->j2(), msg->j3(), msg->j4(), msg->j5(), msg->j6());
			if (!joints_valid) {
				arm_->iface->set_error_code(JacoInterface::ERROR_NO_IK);
				logger->log_warn(name(),
				                 "Failed executing AngularGotoMessage, given target joints for arm %s are "
				                 "invalid or in self-collision",
				                 arm_->arm->get_name().c_str());
			}
#else
			arm_->goto_thread->set_target_ang(
			  msg->j1(), msg->j2(), msg->j3(), msg->j4(), msg->j5(), msg->j6());
#endif

		} else if (arm_->iface->msgq_first_is<JacoInterface::MoveGripperMessage>()) {
			JacoInterface::MoveGripperMessage *msg = arm_->iface->msgq_first(msg);
			logger->log_debug(name(),
			                  "%s: MoveGripperMessage rcvd. f1:%f  f2:%f  f3:%f",
			                  arm_->iface->id(),
			                  msg->finger1(),
			                  msg->finger2(),
			                  msg->finger3());

			arm_->goto_thread->move_gripper(msg->finger1(), msg->finger2(), msg->finger3());

		} else if (arm_->iface->msgq_first_is<JacoInterface::JoystickPushMessage>()) {
			JacoInterface::JoystickPushMessage *msg = arm_->iface->msgq_first(msg);
			logger->log_debug(name(), "%s: JoystickPush %u rcvd", arm_->iface->id(), msg->button());

			arm_->arm->push_joystick(msg->button());

		} else if (arm_->iface->msgq_first_is<JacoInterface::JoystickReleaseMessage>()) {
			JacoInterface::JoystickReleaseMessage *msg = arm_->iface->msgq_first(msg);
			logger->log_debug(name(), "%s: JoystickRelease rcvd", arm_->iface->id());

			arm_->arm->release_joystick();
			arm_->iface->set_final(true);

		} else {
			logger->log_warn(name(), "%s: Unknown message received. Skipping", arm_->iface->id());
		}

		arm_->iface->msgq_pop();
	}
}
