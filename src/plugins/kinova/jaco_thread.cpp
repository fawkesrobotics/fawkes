
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
KinovaJacoThread::KinovaJacoThread(KinovaInfoThread *info_thread, KinovaGotoThread *goto_thread)
  : Thread("KinovaJacoThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT)
{
  __arm = NULL;
  __if_jaco = NULL;
  __info_thread = info_thread;
  __goto_thread = goto_thread;
}


/** Destructor. */
KinovaJacoThread::~KinovaJacoThread()
{
}

void
KinovaJacoThread::init()
{
  try {
    // create new JacoArm object (connects to arm via libusb)
    __arm = new JacoArm();

    // register arm in other threads
    __info_thread->register_arm(__arm);
    __goto_thread->register_arm(__arm);

  } catch(fawkes::Exception &e) {
    logger->log_warn(name(), "Could not connect to JacoArm. Ex:%s", e.what());
  }


  try {
    // open interface for writing
    __if_jaco = blackboard->open_for_writing<JacoInterface>("JacoArm");

    // set interface in other threads
    __info_thread->set_interface(__if_jaco);
    __goto_thread->set_interface(__if_jaco);

  } catch(fawkes::Exception &e) {
    logger->log_warn(name(), "Could not open JacoInterface interface for writing. Er:%s", e.what());
  }
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
  while( ! __if_jaco->msgq_empty() ) {
    Message *m = __if_jaco->msgq_first(m);
    __if_jaco->set_msgid(m->id());
    __if_jaco->set_final(false);
    __if_jaco->write();

    if( __if_jaco->msgq_first_is<JacoInterface::CartesianGotoMessage>() ) {
      JacoInterface::CartesianGotoMessage *msg = __if_jaco->msgq_first(msg);
      logger->log_debug(name(), "CartesianGotoMessage rcvd. x:%f  y:%f  z:%f  e1:%f  e2:%f  e3:%f",
                        msg->x(), msg->y(), msg->z(), msg->e1(), msg->e2(), msg->e3());
      __goto_thread->set_target(msg->x(), msg->y(), msg->z(), msg->e1(), msg->e2(), msg->e3());

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

    } else if( __if_jaco->msgq_first_is<JacoInterface::StopMessage>() ) {
      JacoInterface::StopMessage *msg = __if_jaco->msgq_first(msg);
      logger->log_debug(name(), "StopMessage rcvd");

      __goto_thread->stop();

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