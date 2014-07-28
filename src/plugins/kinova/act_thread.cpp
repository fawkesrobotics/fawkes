
/***************************************************************************
 *  act_thread.cpp - Kinova plugin Act Thread
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

#include <libkindrv/kindrv.h>

#include <interfaces/JacoInterface.h>

using namespace fawkes;
using namespace KinDrv;

/** @class KinovaActThread "act_thread.h"
 * Jaco Arm control thread.
 *
 * @author Bahram Maleki-Fard
 */

/** Constructor.
 * @param thread_name thread name
 */
KinovaActThread::KinovaActThread(KinovaInfoThread *info_thread,
                                   KinovaGotoThread *goto_thread,
                                   JacoOpenraveThread *openrave_thread)
  : Thread("KinovaActThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT)
{
  __arm.arm = NULL;
  __arm.iface = NULL;
  __info_thread = info_thread;
  __goto_thread = goto_thread;
  __openrave_thread = openrave_thread;
}


/** Destructor. */
KinovaActThread::~KinovaActThread()
{
}

void
KinovaActThread::init()
{
  __cfg_auto_init       = config->get_bool("/hardware/jaco/auto_initialize");
  __cfg_auto_calib      = config->get_bool("/hardware/jaco/auto_calibrate");

  try {
    // create new JacoArm object
    __arm.arm = new JacoArm();

    // register arm in other threads
    __info_thread->register_arm(__arm.arm);
    __goto_thread->register_arm(__arm.arm);
    __openrave_thread->register_arm(__arm.arm);

  } catch(fawkes::Exception &e) {
    logger->log_warn(name(), "Could not connect to JacoArm. Ex:%s", e.what());
  }


  try {
    // open interface for writing
    __arm.iface = blackboard->open_for_writing<JacoInterface>("JacoArm");

    // set interface in other threads
    __info_thread->set_interface(__arm.iface);
    __goto_thread->set_interface(__arm.iface);
    __openrave_thread->set_interface(__arm.iface);

  } catch(fawkes::Exception &e) {
    logger->log_warn(name(), "Could not open JacoInterface interface for writing. Er:%s", e.what());
  }

  //check if we need to initialize arm
  jaco_retract_mode_t mode = __arm.arm->get_status();
  if( mode == MODE_NOINIT ) {
    __arm.initialized = false;
    if( __cfg_auto_init ) {
      logger->log_debug(name(), "Initializing arm, wait until finished");
      __arm.iface->set_final(false);
      __goto_thread->pos_ready();
    }

  } else {
    __arm.initialized = true;
    if( __cfg_auto_calib )
      __goto_thread->pos_ready();
  }

  __arm.iface->set_initialized(__arm.initialized);
  __arm.iface->write();
}

void
KinovaActThread::finalize()
{
  try {
    blackboard->close(__arm.iface);
  } catch(fawkes::Exception& e) {
    logger->log_warn(name(), "Could not close JacoInterface interface. Er:%s", e.what());
  }

  delete __arm.arm;
}

void
KinovaActThread::loop()
{
  if( !__arm.initialized && __cfg_auto_init ) {
    logger->log_debug(name(), "wait for arm to calibrate");
    __arm.initialized = __arm.iface->is_final();
    __arm.iface->write();
    return;
  }
  __arm.iface->set_initialized(__arm.initialized);

  while( ! __arm.iface->msgq_empty() ) {
    Message *m = __arm.iface->msgq_first(m);
    __arm.iface->set_msgid(m->id());
    __arm.iface->set_final(false);
    __arm.iface->write();

    if( __arm.iface->msgq_first_is<JacoInterface::StopMessage>() ) {
      JacoInterface::StopMessage *msg = __arm.iface->msgq_first(msg);
      logger->log_debug(name(), "StopMessage rcvd");

      __goto_thread->stop();

    } else if( __arm.iface->msgq_first_is<JacoInterface::CalibrateMessage>() ) {
      JacoInterface::CalibrateMessage *msg = __arm.iface->msgq_first(msg);
      logger->log_debug(name(), "CalibrateMessage rcvd");

      __goto_thread->pos_ready();

    } else if( __arm.iface->msgq_first_is<JacoInterface::RetractMessage>() ) {
      JacoInterface::RetractMessage *msg = __arm.iface->msgq_first(msg);
      logger->log_debug(name(), "RetractMessage rcvd");

      __goto_thread->pos_retract();

    } else if( __arm.iface->msgq_first_is<JacoInterface::CartesianGotoMessage>() ) {
      JacoInterface::CartesianGotoMessage *msg = __arm.iface->msgq_first(msg);
      logger->log_debug(name(), "CartesianGotoMessage rcvd. x:%f  y:%f  z:%f  e1:%f  e2:%f  e3:%f",
                        msg->x(), msg->y(), msg->z(), msg->e1(), msg->e2(), msg->e3());
    #ifdef HAVE_OPENRAVE
      logger->log_debug(name(), "CartesianGotoMessage is being passed to openrave");
      std::vector<float> v = __openrave_thread->set_target(msg->x(), msg->y(), msg->z(), msg->e1(), msg->e2(), msg->e3());
      if( v.size() == 6 )
        __goto_thread->set_target_ang(v.at(0), v.at(1), v.at(2), v.at(3), v.at(4), v.at(5));
    #else
      __goto_thread->set_target(msg->x(), msg->y(), msg->z(), msg->e1(), msg->e2(), msg->e3());
    #endif

    } else if( __arm.iface->msgq_first_is<JacoInterface::AngularGotoMessage>() ) {
      JacoInterface::AngularGotoMessage *msg = __arm.iface->msgq_first(msg);

      logger->log_debug(name(), "AngularGotoMessage rcvd. x:%f  y:%f  z:%f  e1:%f  e2:%f  e3:%f",
                        msg->j1(), msg->j2(), msg->j3(), msg->j4(), msg->j5(), msg->j6());
      __goto_thread->set_target_ang(msg->j1(), msg->j2(), msg->j3(), msg->j4(), msg->j5(), msg->j6());

    } else if( __arm.iface->msgq_first_is<JacoInterface::MoveGripperMessage>() ) {
      JacoInterface::MoveGripperMessage *msg = __arm.iface->msgq_first(msg);
      logger->log_debug(name(), "MoveGripperMessage rcvd. f1:%f  f2:%f  f3:%f",
                        msg->finger1(), msg->finger2(), msg->finger3());

      __goto_thread->move_gripper(msg->finger1(), msg->finger2(), msg->finger3());

    } else if( __arm.iface->msgq_first_is<JacoInterface::JoystickPushMessage>() ) {
      JacoInterface::JoystickPushMessage *msg = __arm.iface->msgq_first(msg);
      logger->log_debug(name(), "JoystickPush %u rcvd", msg->button());

      __arm.arm->start_api_ctrl();
      __arm.arm->push_joystick_button(msg->button());

    } else if( __arm.iface->msgq_first_is<JacoInterface::JoystickReleaseMessage>() ) {
      JacoInterface::JoystickReleaseMessage *msg = __arm.iface->msgq_first(msg);
      logger->log_debug(name(), "JoystickRelease rcvd");

      __arm.arm->start_api_ctrl();
      __arm.arm->release_joystick();
      __arm.iface->set_final(true);

    } else {
      logger->log_warn(name(), "Unknown message received");
    }

    __arm.iface->msgq_pop();
  }

  __arm.iface->write();
}
