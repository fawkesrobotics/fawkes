
/***************************************************************************
 *  arm_kindrv.cpp - Class for a Kinova Jaco arm, using libkindrv
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

#include <core/exception.h>

#include <libkindrv/kindrv.h>

#include <cstdio>
#include <vector>

using namespace KinDrv;

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class JacoArmKindrv <plugins/jaco/arm_kindrv.h>
 * Class for commanding a Kinova Jaco Arm, using libkindrv.
 * @author Bahram Maleki-Fard
 */

/** Constructor.
 * @param name The name of the arm we want to connect to.
 */
JacoArmKindrv::JacoArmKindrv(const char *name)
{
  // take the first arm we can connect to
  __arm = new KinDrv::JacoArm();
  __name = __arm->get_client_config(true).name;
  // trim tailing whitespaces
  __name.erase(__name.find_last_not_of(" ")+1);

  std::string found_names = "'" + __name + "'";

  if( name!=NULL ) {
    // Check all connected arms until the right one is found.
    std::vector<KinDrv::JacoArm*> arms;
    while( __name.compare(name)!=0 ) {
      arms.push_back(__arm);
      try {
        __arm = new KinDrv::JacoArm();
        __name = __arm->get_client_config(true).name;
        __name.erase(__name.find_last_not_of(" ")+1);
        found_names += ", '" + __name + "'";
      } catch(KinDrvException& e) {
        // don't throw yet, we need to delete the occupied arms first.
        __arm = NULL;
        break;
      }
    }

    for(unsigned int i=0; i<arms.size(); ++i) {
      delete arms[i];
      arms[i] = NULL;
    }
  }

  if( __arm==NULL )
  {
    throw fawkes::Exception("Could not connect to Jaco arm '%s' with libkindrv. But I found the following arms: %s", name, found_names.c_str());
  }

  __initialized = false;
  __final = true;
  __ctrl_ang = true;
}

/** Destructor. */
JacoArmKindrv::~JacoArmKindrv()
{
  delete(__arm);
}

void
JacoArmKindrv::initialize()
{
  goto_ready();
}




bool
JacoArmKindrv::final()
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
JacoArmKindrv::initialized()
{
  if( !__initialized ) {
    jaco_retract_mode_t mode = __arm->get_status();
    __initialized = (mode != MODE_NOINIT);
  }

  return __initialized;
}




void
JacoArmKindrv::get_coords(std::vector<float> &to)
{
  if( __ctrl_ang ) {
    // nedd to set control to cart, otherwise we will not get updated data
    __arm->set_control_cart();
    __ctrl_ang = false;
  }
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
JacoArmKindrv::get_joints(std::vector<float> &to) const
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
JacoArmKindrv::get_fingers(std::vector<float> &to) const
{
  jaco_position_t pos = __arm->get_cart_pos();

  to.clear();
  to.push_back(pos.finger_position[0]);
  to.push_back(pos.finger_position[1]);
  to.push_back(pos.finger_position[2]);
}




void
JacoArmKindrv::stop()
{
  __arm->release_joystick();
  __final = true;
}

void
JacoArmKindrv::push_joystick(unsigned int button)
{
  __arm->start_api_ctrl();
  __arm->push_joystick_button(button);
  __final = false;
}

void
JacoArmKindrv::release_joystick()
{
  __arm->start_api_ctrl();
  __arm->release_joystick();
  __final = true;
}


void
JacoArmKindrv::goto_trajec(std::vector< std::vector<float> >* trajec, std::vector<float> &fingers)
{
  __arm->start_api_ctrl();
  __arm->set_control_ang();
  __ctrl_ang = true;
  usleep(500);
  for( unsigned int i=0; i<trajec->size(); ++i ) {
    __arm->set_target_ang(trajec->at(i).at(0), trajec->at(i).at(1), trajec->at(i).at(2),
                          trajec->at(i).at(3), trajec->at(i).at(4), trajec->at(i).at(5),
                          fingers.at(0), fingers.at(1), fingers.at(2));
  }
}

void
JacoArmKindrv::goto_joints(std::vector<float> &joints, std::vector<float> &fingers, bool followup)
{
  __target_type = TARGET_ANGULAR;
  __final = false;

  if(!followup) {
    __arm->start_api_ctrl();
    __arm->set_control_ang();
    __ctrl_ang = true;
    usleep(500);
  }

  __arm->set_target_ang(joints.at(0), joints.at(1), joints.at(2), joints.at(3), joints.at(4), joints.at(5),
                        fingers.at(0), fingers.at(1), fingers.at(2));
}

void
JacoArmKindrv::goto_coords(std::vector<float> &coords, std::vector<float> &fingers)
{
  __target_type = TARGET_CARTESIAN;
  __final = false;

  __arm->start_api_ctrl();
  __arm->set_control_cart();
  __ctrl_ang = false;
  usleep(500);
  //__arm->arm->set_target_cart(__y, -__x, __z, __e1, __e2, __e3, __f1, __f2, __f3);
  __arm->set_target_cart(coords.at(1), -coords.at(0), coords.at(2), coords.at(3), coords.at(4), coords.at(5),
                         fingers.at(0), fingers.at(1), fingers.at(2));
}

void
JacoArmKindrv::goto_ready()
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
JacoArmKindrv::goto_retract()
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
