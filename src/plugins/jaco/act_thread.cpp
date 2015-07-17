
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
#include "goto_thread.h"
#include "openrave_thread.h"

#include "types.h"
#include "arm_kindrv.h"
#include "arm_dummy.h"

#include <interfaces/JacoInterface.h>
#include <core/utils/refptr.h>

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
JacoActThread::JacoActThread(const char *name, jaco_arm_t* arm)
  : Thread(name, Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT)
{
  __arm = arm;
  __arm->arm = NULL;
  __arm->iface = NULL;

  __arm->goto_thread = NULL;
  __arm->openrave_thread = NULL;
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
  __cfg_auto_init       = config->get_bool("/hardware/jaco/auto_initialize");
  __cfg_auto_calib      = config->get_bool("/hardware/jaco/auto_calibrate");

  std::string cfg_arm = config->get_string("/hardware/jaco/arm");

  if( cfg_arm.compare("libkindrv")
   && cfg_arm.compare("dummy") )
    throw fawkes::Exception("Bad config entry /hardware/jaco/arm '%s'", cfg_arm.c_str());

  std::string arm_name, arm_iface;
  switch( __arm->config ) {
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

    default:
      throw fawkes::Exception("Unknown arm config given");
      break;
  }

  // create the JacoArm object and assign correctly to config
  try {
    if( !cfg_arm.compare("dummy") ) {
      __arm->arm = new JacoArmDummy("JacoDummy");
    } else {
      __arm->arm = new JacoArmKindrv(arm_name.c_str());
    }
  } catch(fawkes::Exception &e) {
    logger->log_error(name(), "Could not connect to JacoArm. Exception follows.");
    throw;
  }

  // open interface for writing
  try {
    __arm->iface = blackboard->open_for_writing<JacoInterface>(arm_iface.c_str());
  } catch(fawkes::Exception &e) {
    logger->log_error(name(), "Could not open interface %s for writing. Exception follows.", arm_iface.c_str());
    delete __arm->arm;
    __arm = NULL;
    throw;
  }

  // create target/trajectory queues and mutexes
  __arm->target_mutex = RefPtr<Mutex>(new Mutex());
  __arm->trajec_mutex = RefPtr<Mutex>(new Mutex());
  __arm->target_queue = RefPtr<jaco_target_queue_t>(new jaco_target_queue_t());

  // set trajectory colors (TODO: configurable)
  __arm->trajec_color[0] = 0.f;
  __arm->trajec_color[1] = 0.f;
  __arm->trajec_color[2] = 1.f;
  __arm->trajec_color[3] = 1.f;
  if( __arm->config==CONFIG_RIGHT ) {
    __arm->trajec_color[0] = 1.f;
    __arm->trajec_color[2] = 0.f;
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
    blackboard->close(__arm->iface);
  } catch(fawkes::Exception& e) {
    logger->log_warn(name(), "Could not close JacoInterface interface. Er:%s", e.what_no_backtrace());
  }

  delete __arm->arm;
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
  if( __arm==NULL || __arm->iface==NULL || __arm->openrave_thread==NULL)
    return;

  // firts of all, submit interface updates (that other threads might have done)!
  __arm->iface->write();

  // check if still initializing
  if( _is_initializing() )
    return;

#ifdef HAVE_OPENRAVE
  // make sure openrave-thread is ready!
  if( !__arm->openrave_thread->started() )
    return;
#endif

  // process incoming interface messages
  _process_msgs();

  // finally, again submit interface updates
  __arm->iface->write();

#ifdef HAVE_OPENRAVE
  __arm->openrave_thread->update_openrave();
#endif
  __arm->iface->set_final(__arm->goto_thread->final());
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
  if( !__arm->arm->initialized() && __cfg_auto_init ) {
    logger->log_debug(name(), "Initializing arm, wait until finished");
    __arm->arm->initialize();
    __arm->iface->set_final(false);
    //__arm.goto_thread->pos_ready();

  } else if( __arm->arm->initialized() && __cfg_auto_calib ) {
    __arm->goto_thread->pos_ready();
  }

  __arm->iface->set_initialized(__arm->arm->initialized());
  __arm->iface->write();
}

/** Check if arm is being initialized. */
bool
JacoActThread::_is_initializing()
{
  __arm->iface->set_initialized(__arm->arm->initialized());

  if( !__arm->arm->initialized() && __cfg_auto_init ) {
    logger->log_debug(name(), "wait for arm to initialize");
    //__arm->initialized = __arm->iface->is_final();
    return true;
  }

  return false;
}

/** Process interface messages. */
void
JacoActThread::_process_msgs()
{
  while( ! __arm->iface->msgq_empty() ) {
    Message *m = __arm->iface->msgq_first(m);
    __arm->iface->set_msgid(m->id());
    __arm->iface->set_final(false);
    __arm->iface->set_error_code(JacoInterface::ERROR_NONE);
    __arm->iface->write();

    if( __arm->iface->msgq_first_is<JacoInterface::StopMessage>() ) {
      JacoInterface::StopMessage *msg = __arm->iface->msgq_first(msg);
      logger->log_debug(name(), "%s: StopMessage rcvd", __arm->iface->id());

      __arm->goto_thread->stop();

    } else if( __arm->iface->msgq_first_is<JacoInterface::CalibrateMessage>() ) {
      JacoInterface::CalibrateMessage *msg = __arm->iface->msgq_first(msg);
      logger->log_debug(name(), "%s: CalibrateMessage rcvd", __arm->iface->id());

      // Stop all (current and planned) motion. Then calibrate
      __arm->goto_thread->stop();
      __arm->goto_thread->pos_ready();

    } else if( __arm->iface->msgq_first_is<JacoInterface::RetractMessage>() ) {
      JacoInterface::RetractMessage *msg = __arm->iface->msgq_first(msg);
      logger->log_debug(name(), "%s: RetractMessage rcvd", __arm->iface->id());

      // Stop all (current and planned) motion. Then retract
      __arm->goto_thread->stop();
      __arm->goto_thread->pos_retract();

    } else if( __arm->iface->msgq_first_is<JacoInterface::SetPlannerParamsMessage>() ) {
      JacoInterface::SetPlannerParamsMessage *msg = __arm->iface->msgq_first(msg);
      logger->log_debug(name(), "%s: SetPlannerParamsMessage rcvd. params:%s", __arm->iface->id(), msg->params());

    #ifdef HAVE_OPENRAVE
      __arm->openrave_thread->set_plannerparams(msg->params());
    #endif

    } else if( __arm->iface->msgq_first_is<JacoInterface::CartesianGotoMessage>() ) {
      JacoInterface::CartesianGotoMessage *msg = __arm->iface->msgq_first(msg);
      logger->log_debug(name(), "%s: CartesianGotoMessage rcvd. x:%f  y:%f  z:%f  e1:%f  e2:%f  e3:%f", __arm->iface->id(),
                        msg->x(), msg->y(), msg->z(), msg->e1(), msg->e2(), msg->e3());
    #ifdef HAVE_OPENRAVE
      logger->log_debug(name(), "%s: CartesianGotoMessage is being passed to openrave", __arm->iface->id());
      // add target to OpenRAVE queue for planning
      bool solvable = __arm->openrave_thread->add_target(msg->x(), msg->y(), msg->z(), msg->e1(), msg->e2(), msg->e3());
      if( !solvable ) {
        __arm->iface->set_error_code(JacoInterface::ERROR_NO_IK);
        logger->log_warn(name(), "Failed executing CartesianGotoMessage, arm %s and/or thread %s could not find IK solution",
                         __arm->arm->get_name().c_str(), __arm->openrave_thread->name());
      }
    #else
      __arm->goto_thread->set_target(msg->x(), msg->y(), msg->z(), msg->e1(), msg->e2(), msg->e3());
    #endif

    } else if( __arm->iface->msgq_first_is<JacoInterface::AngularGotoMessage>() ) {
      JacoInterface::AngularGotoMessage *msg = __arm->iface->msgq_first(msg);

      logger->log_debug(name(), "%s: AngularGotoMessage rcvd. x:%f  y:%f  z:%f  e1:%f  e2:%f  e3:%f", __arm->iface->id(),
                        msg->j1(), msg->j2(), msg->j3(), msg->j4(), msg->j5(), msg->j6());
    #ifdef HAVE_OPENRAVE
      logger->log_debug(name(), "%s: AngularGotoMessage is being passed to openrave", __arm->iface->id());
      // add target to OpenRAVE queue for planning
      bool joints_valid = __arm->openrave_thread->add_target_ang(msg->j1(), msg->j2(), msg->j3(), msg->j4(), msg->j5(), msg->j6());
      if( !joints_valid ) {
        __arm->iface->set_error_code(JacoInterface::ERROR_NO_IK);
        logger->log_warn(name(), "Failed executing AngularGotoMessage, given target joints for arm %s are invalid or in self-collision",
                         __arm->arm->get_name().c_str());
      }
    #else
      __arm->goto_thread->set_target_ang(msg->j1(), msg->j2(), msg->j3(), msg->j4(), msg->j5(), msg->j6());
    #endif

    } else if( __arm->iface->msgq_first_is<JacoInterface::MoveGripperMessage>() ) {
      JacoInterface::MoveGripperMessage *msg = __arm->iface->msgq_first(msg);
      logger->log_debug(name(), "%s: MoveGripperMessage rcvd. f1:%f  f2:%f  f3:%f", __arm->iface->id(),
                        msg->finger1(), msg->finger2(), msg->finger3());

      __arm->goto_thread->move_gripper(msg->finger1(), msg->finger2(), msg->finger3());

    } else if( __arm->iface->msgq_first_is<JacoInterface::JoystickPushMessage>() ) {
      JacoInterface::JoystickPushMessage *msg = __arm->iface->msgq_first(msg);
      logger->log_debug(name(), "%s: JoystickPush %u rcvd", __arm->iface->id(), msg->button());

      __arm->arm->push_joystick(msg->button());

    } else if( __arm->iface->msgq_first_is<JacoInterface::JoystickReleaseMessage>() ) {
      JacoInterface::JoystickReleaseMessage *msg = __arm->iface->msgq_first(msg);
      logger->log_debug(name(), "%s: JoystickRelease rcvd", __arm->iface->id());

      __arm->arm->release_joystick();
      __arm->iface->set_final(true);

    } else {
      logger->log_warn(name(), "%s: Unknown message received. Skipping", __arm->iface->id());
    }

    __arm->iface->msgq_pop();
  }
}
