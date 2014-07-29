
/***************************************************************************
 *  arm_kindrv.cpp - Class for a Kinova arm, using libkindrv
 *
 *  Created: Tue Jul 29 14:58:32 2014
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

#include "arm_kindrv.h"

#include <libkindrv/kindrv.h>

using namespace KinDrv;

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class KinovaArmKindrv <plugins/kinova/arm_kindrv.h>
 * Class for commanding a Kinova Arm, using libkindrv.
 * @author Bahram Maleki-Fard
 */

/** Constructor. */
KinovaArmKindrv::KinovaArmKindrv()
{
  __arm = new JacoArm();
  __name = __arm->get_client_config(true).name;

  __initialized = false;
  __final = false;
}

/** Destructor. */
KinovaArmKindrv::~KinovaArmKindrv()
{
  delete(__arm);
}

void
KinovaArmKindrv::initialize()
{
  goto_ready();
}




bool
KinovaArmKindrv::final()
{
  if( __final )
    return true;

  switch( __target_type ) {

    case TARGET_READY:
      {
        jaco_retract_mode_t mode = __arm->get_status();
        __final = (mode == MODE_READY_STANDBY);

        if( __final ) {
          __arm->release_joystick();
        } else if( mode == MODE_READY_TO_RETRACT ) {
          // is moving in wrong direction
          __arm->release_joystick();
          __arm->push_joystick_button(2);
        }
      }
      break;

    case TARGET_RETRACT:
      {
        jaco_retract_mode_t mode = __arm->get_status();
        __final = (mode == MODE_RETRACT_STANDBY);
      }
      if( __final )
        __arm->release_joystick();
      break;

    default: //TARGET_ANGULAR, TARGET_CARTESIAN
      __final = true;
      {
        jaco_position_t vel = __arm->get_ang_vel();
        for( unsigned int i=0; i<6; ++i ) {
          __final &= std::abs(vel.joints[i]) < 0.01;
        }
        for( unsigned int i=0; i<3; ++i ) {
          __final &= std::abs(vel.finger_position[i]) < 0.01;
        }
      }
      break;
  }

  return __final;
}

bool
KinovaArmKindrv::initialized()
{
  if( !__initialized) {
    jaco_retract_mode_t mode = __arm->get_status();
    __initialized = (mode != MODE_NOINIT);
  }

  return __initialized;
}




void
KinovaArmKindrv::get_coords(std::vector<float> &to) const
{
  jaco_position_t pos = __arm->get_cart_pos();

  to.clear();
  to.push_back(-pos.position[1]);
  to.push_back( pos.position[0]);
  to.push_back( pos.position[2]);
  to.push_back(pos.rotation[0]);
  to.push_back(pos.rotation[1]);
  to.push_back(pos.rotation[2]);
}

void
KinovaArmKindrv::get_joints(std::vector<float> &to) const
{
  jaco_position_t pos = __arm->get_ang_pos();

  to.clear();
  to.push_back(pos.joints[0]);
  to.push_back(pos.joints[1]);
  to.push_back(pos.joints[2]);
  to.push_back(pos.joints[3]);
  to.push_back(pos.joints[4]);
  to.push_back(pos.joints[5]);
}

void
KinovaArmKindrv::get_fingers(std::vector<float> &to) const
{
  jaco_position_t pos = __arm->get_cart_pos();

  to.clear();
  to.push_back(pos.finger_position[0]);
  to.push_back(pos.finger_position[1]);
  to.push_back(pos.finger_position[2]);
}




void
KinovaArmKindrv::stop()
{
  __arm->release_joystick();
  __final = true;
}

void
KinovaArmKindrv::push_joystick(unsigned int button)
{
  __arm->start_api_ctrl();
  __arm->push_joystick_button(button);
  __final = false;
}

void
KinovaArmKindrv::release_joystick()
{
  __arm->start_api_ctrl();
  __arm->release_joystick();
  __final = true;
}




void
KinovaArmKindrv::goto_joints(std::vector<float> &joints, std::vector<float> &fingers)
{
  __target_type = TARGET_ANGULAR;
  __final = false;

  __arm->start_api_ctrl();
  __arm->set_control_ang();
  usleep(500);
  __arm->set_target_ang(joints.at(0), joints.at(1), joints.at(2), joints.at(3), joints.at(4), joints.at(5),
                        fingers.at(0), fingers.at(1), fingers.at(2));
}

void
KinovaArmKindrv::goto_coords(std::vector<float> &coords, std::vector<float> &fingers)
{
  __target_type = TARGET_CARTESIAN;
  __final = false;

  __arm->start_api_ctrl();
  __arm->set_control_cart();
  usleep(500);
  //__arm->arm->set_target_cart(__y, -__x, __z, __e1, __e2, __e3, __f1, __f2, __f3);
  __arm->set_target_cart(coords.at(1), -coords.at(0), coords.at(2), coords.at(3), coords.at(4), coords.at(5),
                         fingers.at(0), fingers.at(1), fingers.at(2));
}

void
KinovaArmKindrv::goto_ready()
{
  __target_type = TARGET_READY;
  __final = false;

  __arm->start_api_ctrl();
  jaco_retract_mode_t mode = __arm->get_status();
  switch( mode ) {
    case MODE_RETRACT_TO_READY:
      //2 buttons needed
      __arm->push_joystick_button(2);
      __arm->release_joystick();
      __arm->push_joystick_button(2);
      break;

    case MODE_NORMAL_TO_READY:
    case MODE_READY_TO_RETRACT:
    case MODE_RETRACT_STANDBY:
    case MODE_NORMAL:
    case MODE_NOINIT:
      //1 button needed
      __arm->push_joystick_button(2);
      break;

    case MODE_ERROR:
      // error: some error occured
      // TODO: return something?
      break;

    case MODE_READY_STANDBY:
      // no action. error?
      // __final = true;
      break;
  }
}

void
KinovaArmKindrv::goto_retract()
{
  __target_type = TARGET_RETRACT;
  __final = false;

  __arm->start_api_ctrl();
  jaco_retract_mode_t mode = __arm->get_status();
  switch( mode ) {
    case MODE_READY_TO_RETRACT:
      // 2 buttons needed
      __arm->push_joystick_button(2);
      __arm->release_joystick();
      __arm->push_joystick_button(2);
      break;

    case MODE_READY_STANDBY:
    case MODE_RETRACT_TO_READY:
      // 1 button needed
      __arm->push_joystick_button(2);
      break;

    case MODE_NORMAL_TO_READY:
    case MODE_NORMAL:
    case MODE_NOINIT:
      // warn: cannot go from NORMAL/NOINIT to RETRACT");
      //__final = true;
      break;

    case MODE_ERROR:
      // error: some error occured!!
      // TODO: return something?
      break;

    case MODE_RETRACT_STANDBY:
      // no action. error?
      //__final = true;
      break;
  }
}

} // end of namespace fawkes
