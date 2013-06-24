
/***************************************************************************
 *  jaco_thread.cpp - Kinova plugin Jaco Thread
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

#include "jaco_thread.h"
#include "kinova_api.h"

#include <interfaces/JacoInterface.h>

using namespace fawkes;

/** @class KinovaJacoThread "jaco_thread.h"
 * Jaco Arm control thread.
 *
 * @author Bahram Maleki-Fard
 */

/** Constructor.
 * @param thread_name thread name
 */
KinovaJacoThread::KinovaJacoThread(KinovaInfoThread *info_thread,
                                   KinovaGotoThread *goto_thread,
                                   JacoOpenraveThread *openrave_thread)
  : Thread("KinovaJacoThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT)
{
  __arm = NULL;
  __if_jaco = NULL;
  __info_thread = info_thread;
  __goto_thread = goto_thread;
  __openrave_thread = openrave_thread;
  __initialized = false;
}


/** Destructor. */
KinovaJacoThread::~KinovaJacoThread()
{
}

void
KinovaJacoThread::init()
{
  __cfg_auto_init       = config->get_bool("/hardware/jaco/auto_initialize");
  __cfg_auto_calib      = config->get_bool("/hardware/jaco/auto_calibrate");

  try {
    // create new JacoArm object (connects to arm via libusb)
    __arm = new JacoArm();

    // register arm in other threads
    __info_thread->register_arm(__arm);
    __goto_thread->register_arm(__arm);
    __openrave_thread->register_arm(__arm);

  } catch(fawkes::Exception &e) {
    logger->log_warn(name(), "Could not connect to JacoArm. Ex:%s", e.what());
  }


  try {
    // open interface for writing
    __if_jaco = blackboard->open_for_writing<JacoInterface>("JacoArm");

    // set interface in other threads
    __info_thread->set_interface(__if_jaco);
    __goto_thread->set_interface(__if_jaco);
    __openrave_thread->set_interface(__if_jaco);

  } catch(fawkes::Exception &e) {
    logger->log_warn(name(), "Could not open JacoInterface interface for writing. Er:%s", e.what());
  }

  //check if we need to initialize arm
  jaco_retract_mode_t mode = __arm->get_status();
  if( mode == MODE_NOINIT ) {
    __initialized = false;
    if( __cfg_auto_init ) {
      logger->log_debug(name(), "Initializing arm, wait until finished");
      __if_jaco->set_final(false);
      __goto_thread->pos_ready();
    }

  } else {
    __initialized = true;
    if( __cfg_auto_calib )
      __goto_thread->pos_ready();
  }

  __if_jaco->set_initialized(__initialized);
  __if_jaco->write();
}

void
KinovaJacoThread::finalize()
{
  try {
    blackboard->close(__if_jaco);
  } catch(fawkes::Exception& e) {
    logger->log_warn(name(), "Could not close JacoInterface interface. Er:%s", e.what());
  }

  delete __arm;
}

void
KinovaJacoThread::loop()
{
  if( !__initialized && __cfg_auto_init ) {
    logger->log_debug(name(), "wait for arm to calibrate");
    __initialized = __if_jaco->is_final();
    __if_jaco->write();
    return;
  }
  __if_jaco->set_initialized(__initialized);

  while( ! __if_jaco->msgq_empty() ) {
    Message *m = __if_jaco->msgq_first(m);
    __if_jaco->set_msgid(m->id());
    __if_jaco->set_final(false);
    __if_jaco->write();

    if( __if_jaco->msgq_first_is<JacoInterface::StopMessage>() ) {
      JacoInterface::StopMessage *msg = __if_jaco->msgq_first(msg);
      logger->log_debug(name(), "StopMessage rcvd");

      __goto_thread->stop();

    } else if( __if_jaco->msgq_first_is<JacoInterface::CalibrateMessage>() ) {
      JacoInterface::CalibrateMessage *msg = __if_jaco->msgq_first(msg);
      logger->log_debug(name(), "CalibrateMessage rcvd");

      __goto_thread->pos_ready();

    } else if( __if_jaco->msgq_first_is<JacoInterface::RetractMessage>() ) {
      JacoInterface::RetractMessage *msg = __if_jaco->msgq_first(msg);
      logger->log_debug(name(), "RetractMessage rcvd");

      __goto_thread->pos_retract();

    } else if( __if_jaco->msgq_first_is<JacoInterface::CartesianGotoMessage>() ) {
      JacoInterface::CartesianGotoMessage *msg = __if_jaco->msgq_first(msg);
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

    } else if( __if_jaco->msgq_first_is<JacoInterface::AngularGotoMessage>() ) {
      JacoInterface::AngularGotoMessage *msg = __if_jaco->msgq_first(msg);

      logger->log_debug(name(), "AngularGotoMessage rcvd. x:%f  y:%f  z:%f  e1:%f  e2:%f  e3:%f",
                        msg->j1(), msg->j2(), msg->j3(), msg->j4(), msg->j5(), msg->j6());
      __goto_thread->set_target_ang(msg->j1(), msg->j2(), msg->j3(), msg->j4(), msg->j5(), msg->j6());

    } else if( __if_jaco->msgq_first_is<JacoInterface::OpenGripperMessage>() ) {
      JacoInterface::OpenGripperMessage *msg = __if_jaco->msgq_first(msg);
      logger->log_debug(name(), "OpenGripperMessage rcvd");

      __goto_thread->open_gripper();

    } else if( __if_jaco->msgq_first_is<JacoInterface::CloseGripperMessage>() ) {
      JacoInterface::CloseGripperMessage *msg = __if_jaco->msgq_first(msg);
      logger->log_debug(name(), "CloseGripperMessage rcvd");

      __goto_thread->close_gripper();

    } else if( __if_jaco->msgq_first_is<JacoInterface::JoystickPushMessage>() ) {
      JacoInterface::JoystickPushMessage *msg = __if_jaco->msgq_first(msg);
      logger->log_debug(name(), "JoystickPush %u rcvd", msg->button());

      __arm->start_api_ctrl();
      __arm->push_joystick_button(msg->button());

    } else if( __if_jaco->msgq_first_is<JacoInterface::JoystickReleaseMessage>() ) {
      JacoInterface::JoystickReleaseMessage *msg = __if_jaco->msgq_first(msg);
      logger->log_debug(name(), "JoystickRelease rcvd");

      __arm->start_api_ctrl();
      __arm->release_joystick();
      __if_jaco->set_final(true);

    } else {
      logger->log_warn(name(), "Unknown message received");
    }

    __if_jaco->msgq_pop();
  }

  __if_jaco->write();
}