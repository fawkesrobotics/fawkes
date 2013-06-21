
/***************************************************************************
 *  goto_thread.cpp - Kinova plugin Jaco movement thread
 *
 *  Created: Thu Jun 20 15:04:20 2013
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

#include "goto_thread.h"
#include "kinova_api.h"

#include <interfaces/JacoInterface.h>

#include <stdio.h>
#include <cstring>
#include <complex>

using namespace fawkes;

/** @class KinovaGotoThread "goto_thread.h"
 * Jaco Arm movement thread.
 * This thread handles the movement of the arm.
 *
 * @author Bahram Maleki-Fard
 */

/** Constructor.
 * @param thread_name thread name
 */
KinovaGotoThread::KinovaGotoThread()
  : Thread("KinovaGotoThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT_EXEC)
{
  __arm = NULL;
  __if_jaco = NULL;

  __new_target = false;
  __target_angular = false;
  __final = true;
}


/** Destructor. */
KinovaGotoThread::~KinovaGotoThread()
{
}

void
KinovaGotoThread::init()
{
}

void
KinovaGotoThread::finalize()
{
}

void
KinovaGotoThread::register_arm(JacoArm *arm) {
  __arm = arm;
}

void
KinovaGotoThread::unregister_arm() {
  __arm = NULL;
}

void
KinovaGotoThread::set_interface(JacoInterface *if_jaco)
{
  __if_jaco = if_jaco;
}

void
KinovaGotoThread::set_target(float x, float y, float z,
			     float e1, float e2, float e3,
                             float f1, float f2, float f3)
{
  __x     = x;
  __y     = y;
  __z     = z;
  __e1   = e1;
  __e2   = e2;
  __e3   = e3;
  if( f1 > 0.f && f2 > 0.f && f3 > 0.f ) {
    __f1 = f1;
    __f2 = f2;
    __f3 = f3;
  } else {
    __f1 = __if_jaco->finger1();
    __f2 = __if_jaco->finger2();
    __f3 = __if_jaco->finger3();
  }
  __new_target = true;
  __target_angular = false;
}

void
KinovaGotoThread::set_target_ang(float j1, float j2, float j3,
                                 float j4, float j5, float j6,
                                 float f1, float f2, float f3)
{
  __joints[0] = j1;
  __joints[1] = j2;
  __joints[2] = j3;
  __joints[3] = j4;
  __joints[4] = j5;
  __joints[5] = j6;

  if( f1 > 0.f && f2 > 0.f && f3 > 0.f ) {
    __f1 = f1;
    __f2 = f2;
    __f3 = f3;
  } else {
    __f1 = __if_jaco->finger1();
    __f2 = __if_jaco->finger2();
    __f3 = __if_jaco->finger3();
  }

  __new_target = true;
  __target_angular = true;
}

void
KinovaGotoThread::open_gripper()
{
  __x  = __if_jaco->x();
  __y  = __if_jaco->y();
  __z  = __if_jaco->z();
  __e1 = __if_jaco->euler1();
  __e2 = __if_jaco->euler2();
  __e3 = __if_jaco->euler3();
  __f1 = 0.25f;
  __f2 = 0.25f;
  __f3 = 0.25f;

  __new_target = true;
}

void
KinovaGotoThread::close_gripper()
{
  __x  = __if_jaco->x();
  __y  = __if_jaco->y();
  __z  = __if_jaco->z();
  __e1 = __if_jaco->euler1();
  __e2 = __if_jaco->euler2();
  __e3 = __if_jaco->euler3();
  __f1 = 52.f;
  __f2 = 52.f;
  __f3 = 52.f;

  __new_target = true;
}

void
KinovaGotoThread::stop()
{
  try {
    __arm->stop_api_ctrl();

    __final = true;
    __new_target = false;

  } catch( Exception &e ) {
    logger->log_warn(name(), "Error sending stop command to arm. Ex:%s", e.what());
  }
}

void
KinovaGotoThread::check_final()
{
  __final = true;
  if( __target_angular) {
    for( unsigned int i=0; i<6; ++i ) {
      __final &= (std::abs(__joints[i] - __if_jaco->joints(i)) < 1.0);
    }
  } else {
    __final &= (std::abs(__x - __if_jaco->x()) < 0.01);
    __final &= (std::abs(__y - __if_jaco->y()) < 0.01);
    __final &= (std::abs(__z - __if_jaco->z()) < 0.01);
    __final &= (std::abs(__e1 - __if_jaco->euler1()) < 0.01);
    __final &= (std::abs(__e2 - __if_jaco->euler2()) < 0.01);
    __final &= (std::abs(__e3 - __if_jaco->euler3()) < 0.01);
  }

  if( !__final )
    return;

  // also check fingeres
  if( __finger_last[0] == __if_jaco->finger1() &&
      __finger_last[1] == __if_jaco->finger2() &&
      __finger_last[2] == __if_jaco->finger3() ) {
    __finger_last[3] += 1;
  } else {
    __finger_last[0] = __if_jaco->finger1();
    __finger_last[1] = __if_jaco->finger2();
    __finger_last[2] = __if_jaco->finger3();
    __finger_last[3] = 0; // counter
  }

  __final &= __finger_last[3] > 5;
}

void
KinovaGotoThread::loop()
{
  if(__arm == NULL || __if_jaco == NULL) {
    return;
  }

  if( __new_target ) {
    logger->log_debug(name(), "rcvd new target. skip old one, set this. using current finger positions");

    __finger_last[0] = __if_jaco->finger1();
    __finger_last[1] = __if_jaco->finger2();
    __finger_last[2] = __if_jaco->finger3();
    __finger_last[3] = 0; // counter

    // process new target
    try {
      __arm->stop_api_ctrl(); // stop old movement
      usleep(500);
      __arm->start_api_ctrl();
      if( __target_angular ) {
        __arm->set_control_ang();
        usleep(500);
        float fing[3] = {__f1, __f2, __f3};
        __arm->set_target_ang(__joints, fing);
      } else {
        __arm->set_control_cart();
        usleep(500);
        __arm->set_target_cart(__y, -__x, __z, __e1, __e2, __e3, __f1, __f2, __f3);
      }

      __final = false;
      __new_target = false;

    } catch( Exception &e ) {
      logger->log_warn(name(), "Error sending command to arm. Ex:%s", e.what());
    }

  } else if( !__final ) {
    // check for final position
    check_final();
  }

  if( __final )
    __arm->stop_api_ctrl(); // need to do this to be able to send a command next time

  __if_jaco->set_final(__final);
}