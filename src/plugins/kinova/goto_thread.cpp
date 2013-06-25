
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
#include <utils/math/angle.h>

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
  __final = true;

  __wait_status_check = 0; //wait loops to check for jaco_retract_mode_t again
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
  __target_type = TARGET_CARTESIAN;
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
  __target_type = TARGET_ANGULAR;
}

void
KinovaGotoThread::pos_ready()
{
  __new_target = true;
  __target_type = TARGET_READY;
}

void
KinovaGotoThread::pos_retract()
{
  __new_target = true;
  __target_type = TARGET_RETRACT;
}


void
KinovaGotoThread::move_gripper(float f1, float f2 ,float f3)
{
  __joints[0] = __if_jaco->joints(0);
  __joints[1] = __if_jaco->joints(1);
  __joints[2] = __if_jaco->joints(2);
  __joints[3] = __if_jaco->joints(3);
  __joints[4] = __if_jaco->joints(4);
  __joints[5] = __if_jaco->joints(5);

  __f1 = f1;
  __f2 = f2;
  __f3 = f3;

  __new_target = true;
  __target_type = TARGET_ANGULAR;
}


void
KinovaGotoThread::stop()
{
  try {
    __arm->release_joystick();

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
  bool check_fingers = false;

  logger->log_debug(name(), "check final");
  switch( __target_type ) {
    case TARGET_ANGULAR:
      logger->log_debug(name(), "check final for TARGET ANGULAR");
      for( unsigned int i=0; i<6; ++i ) {
        __final &= (std::abs(normalize_mirror_rad(deg2rad(__joints[i] - __if_jaco->joints(i)))) < 0.01);
      }
      check_fingers = true;
      break;

    case TARGET_READY:
      logger->log_debug(name(), "check final for TARGET READY");
      if( __wait_status_check == 0 ) {
        logger->log_debug(name(), "check final for TARGET READY now");
        //__wait_status_check = 0;
        jaco_retract_mode_t mode = __arm->get_status();
        logger->log_debug(name(), "current mode: %u", mode);
        ++__wait_status_check;
        __final = (mode == MODE_READY_STANDBY);

        if( __final )
          __arm->release_joystick();
        else if( mode == MODE_READY_TO_RETRACT ) {
          __arm->release_joystick();
          __arm->push_joystick_button(2);
        }
      } else {
        logger->log_debug(name(), "check final for TARGET READY not yet");
        __final = false;
        if( __wait_status_check >= 10 )
          __wait_status_check = 0;
        else
          ++__wait_status_check;
      }
      break;

    case TARGET_RETRACT:
      logger->log_debug(name(), "check final for TARGET RETRACT");
      if( __wait_status_check == 0 ) {
        logger->log_debug(name(), "check final for TARGET RETRACT now");
        //__wait_status_check = 0;
        jaco_retract_mode_t mode = __arm->get_status();
        logger->log_debug(name(), "current mode: %u", mode);
        ++__wait_status_check;
        __final = (mode == MODE_RETRACT_STANDBY);
        if( __final )
          __arm->release_joystick();
      } else {
        logger->log_debug(name(), "check final for TARGET RETRACT");
        __final = false;
        if( __wait_status_check >= 10 )
          __wait_status_check = 0;
        else
          ++__wait_status_check;
      }
      break;


    default: //TARGET_CARTESIAN
      logger->log_debug(name(), "check final for TARGET CARTESIAN");
      /*
      logger->log_debug(name(), "target: %f  %f  %f  |  %f  %f  %f", __x, __y, __z, __e1, __e2, __e3);
      logger->log_debug(name(), "if_jac: %f  %f  %f  |  %f  %f  %f", __if_jaco->x(), __if_jaco->y(), __if_jaco->z(),
                                                                   __if_jaco->euler1(), __if_jaco->euler2(), __if_jaco->euler3());
      logger->log_debug(name(), "diff  : %f  %f  %f  |  %f  %f  %f",
                        angle_distance(__x , __if_jaco->x()),
                        angle_distance(__y , __if_jaco->y()),
                        angle_distance(__z , __if_jaco->z()),
                        angle_distance(__e1 , __if_jaco->euler1()),
                        angle_distance(__e2 , __if_jaco->euler2()),
                        angle_distance(__e3 , __if_jaco->euler3()) );

      //*/
      __final &= (std::abs(angle_distance(__x , __if_jaco->x())) < 0.01);
      __final &= (std::abs(angle_distance(__y , __if_jaco->y())) < 0.01);
      __final &= (std::abs(angle_distance(__z , __if_jaco->z())) < 0.01);
      __final &= (std::abs(angle_distance(__e1 , __if_jaco->euler1())) < 0.1);
      __final &= (std::abs(angle_distance(__e2 , __if_jaco->euler2())) < 0.1);
      __final &= (std::abs(angle_distance(__e3 , __if_jaco->euler3())) < 0.1);
/*
      __final &= (std::abs(__x - __if_jaco->x()) < 0.01);
      __final &= (std::abs(__y - __if_jaco->y()) < 0.01);
      __final &= (std::abs(__z - __if_jaco->z()) < 0.01);
      __final &= (std::abs(__e1 - __if_jaco->euler1()) < 0.1);
      __final &= (std::abs(__e2 - __if_jaco->euler2()) < 0.1);
      __final &= (std::abs(__e3 - __if_jaco->euler3()) < 0.1);
*/
      check_fingers = true;
      break;
  }

  //logger->log_debug(name(), "check final: %u", __final);

  if( check_fingers && __final ) {
    logger->log_debug(name(), "check fingeres for final");

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

    __final &= __finger_last[3] > 30;
  }
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

    logger->log_debug(name(), "loop: set final=false");
    __final = false;
    __new_target = false;

    // process new target
    try {
      __arm->release_joystick(); // stop old movement
      __arm->start_api_ctrl();

      switch( __target_type ) {
        case TARGET_ANGULAR:
          logger->log_debug(name(), "target_type: TARGET_ANGULAR");
          { __arm->set_control_ang();
            usleep(500);
            float fing[3] = {__f1, __f2, __f3};
            __arm->set_target_ang(__joints, fing);
            break;
          }

        case TARGET_READY:
          { logger->log_debug(name(), "loop: target_type: TARGET_READY");
            jaco_retract_mode_t mode = __arm->get_status();
            __wait_status_check = 0;
            logger->log_debug(name(), "loop: current mode: %u", mode);
            switch( mode ) {
              case MODE_RETRACT_TO_READY:
                logger->log_debug(name(), "loop: 2 buttons needed");
                __arm->push_joystick_button(2);
                __arm->release_joystick();
                __arm->push_joystick_button(2);
                break;

              case MODE_NORMAL_TO_READY:
              case MODE_READY_TO_RETRACT:
              case MODE_RETRACT_STANDBY:
              case MODE_NORMAL:
              case MODE_NOINIT:
                logger->log_debug(name(), "loop: 1 button needed");
                __arm->push_joystick_button(2);
                break;

              case MODE_ERROR:
                logger->log_error(name(), "some error occured!!");
                break;

              case MODE_READY_STANDBY:
                logger->log_debug(name(), "loop: no action. error?");
                __final = true;
                break;
            }
            break;
          }


        case TARGET_RETRACT:
          { logger->log_debug(name(), "target_type: TARGET_RETRACT");
            jaco_retract_mode_t mode = __arm->get_status();
            __wait_status_check = 0;
            logger->log_debug(name(), "loop: current mode: %u", mode);
            switch( mode ) {
              case MODE_READY_TO_RETRACT:
                logger->log_debug(name(), "loop: 2 buttons needed");
                __arm->push_joystick_button(2);
                __arm->release_joystick();
                __arm->push_joystick_button(2);
                break;

              case MODE_READY_STANDBY:
              case MODE_RETRACT_TO_READY:
                logger->log_debug(name(), "loop: 1 button needed");
                __arm->push_joystick_button(2);
                break;

              case MODE_NORMAL_TO_READY:
              case MODE_NORMAL:
              case MODE_NOINIT:
                logger->log_warn(name(), "loop: cannot go from NORMAL/NOINIT to RETRACT");
                __final = true;
                break;

              case MODE_ERROR:
                logger->log_error(name(), "some error occured!!");
                break;

              case MODE_RETRACT_STANDBY:
                logger->log_debug(name(), "loop: no action. error?");
                __final = true;
                break;
            }
            break;
          }

        default: //TARGET_CARTESIAN
          logger->log_debug(name(), "target_type: TARGET_CARTESIAN");
          __arm->set_control_cart();
          usleep(500);
          __arm->set_target_cart(__y, -__x, __z, __e1, __e2, __e3, __f1, __f2, __f3);
          break;
      }

    } catch( Exception &e ) {
      logger->log_warn(name(), "Error sending command to arm. Ex:%s", e.what());
    }

  } else if( !__final ) {
    // check for final position
    check_final();

    /*
    if( __final ) {
      __arm->stop_api_ctrl(); // need to do this to be able to send a command next time
      usleep(500);
    }
    //*/
  }

  __if_jaco->set_final(__final);
}