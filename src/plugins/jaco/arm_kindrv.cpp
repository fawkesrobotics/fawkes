
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
  arm_ = new KinDrv::JacoArm();
  name_ = arm_->get_client_config(true).name;
  // trim tailing whitespaces
  name_.erase(name_.find_last_not_of(" ")+1);

  std::string found_names = "'" + name_ + "'";

  if( name!=NULL ) {
    // Check all connected arms until the right one is found.
    std::vector<KinDrv::JacoArm*> arms;
    while( name_.compare(name)!=0 ) {
      arms.push_back(arm_);
      try {
        arm_ = new KinDrv::JacoArm();
        name_ = arm_->get_client_config(true).name;
        name_.erase(name_.find_last_not_of(" ")+1);
        found_names += ", '" + name_ + "'";
      } catch(KinDrvException& e) {
        // don't throw yet, we need to delete the occupied arms first.
        arm_ = NULL;
        break;
      }
    }

    for(unsigned int i=0; i<arms.size(); ++i) {
      delete arms[i];
      arms[i] = NULL;
    }
  }

  if( arm_==NULL )
  {
    throw fawkes::Exception("Could not connect to Jaco arm '%s' with libkindrv. But I found the following arms: %s", name, found_names.c_str());
  }

  initialized_ = false;
  final_ = true;
  ctrl_ang_ = true;
}

/** Destructor. */
JacoArmKindrv::~JacoArmKindrv()
{
  delete(arm_);
}

void
JacoArmKindrv::initialize()
{
  goto_ready();
}




bool
JacoArmKindrv::final()
{
  if( final_ )
    return true;

  switch( target_type_ ) {

    case TARGET_READY:
      {
        jaco_retract_mode_t mode = arm_->get_status();
        final_ = (mode == MODE_READY_STANDBY);

        if( final_ ) {
          arm_->release_joystick();
        } else if( mode == MODE_READY_TO_RETRACT ) {
          // is moving in wrong direction
          arm_->release_joystick();
          arm_->push_joystick_button(2);
        }
      }
      break;

    case TARGET_RETRACT:
      {
        jaco_retract_mode_t mode = arm_->get_status();
        final_ = (mode == MODE_RETRACT_STANDBY);
      }
      if( final_ )
        arm_->release_joystick();
      break;

    default: //TARGET_ANGULAR, TARGET_CARTESIAN
      final_ = true;
      {
        jaco_position_t vel = arm_->get_ang_vel();
        for( unsigned int i=0; i<6; ++i ) {
          final_ &= std::abs(vel.joints[i]) < 0.01;
        }
        for( unsigned int i=0; i<3; ++i ) {
          final_ &= std::abs(vel.finger_position[i]) < 0.01;
        }
      }
      break;
  }

  return final_;
}

bool
JacoArmKindrv::initialized()
{
  if( !initialized_ ) {
    jaco_retract_mode_t mode = arm_->get_status();
    initialized_ = (mode != MODE_NOINIT);
  }

  return initialized_;
}




void
JacoArmKindrv::get_coords(std::vector<float> &to)
{
  if( ctrl_ang_ ) {
    // nedd to set control to cart, otherwise we will not get updated data
    arm_->set_control_cart();
    ctrl_ang_ = false;
  }
  jaco_position_t pos = arm_->get_cart_pos();

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
  jaco_position_t pos = arm_->get_ang_pos();

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
  jaco_position_t pos = arm_->get_cart_pos();

  to.clear();
  to.push_back(pos.finger_position[0]);
  to.push_back(pos.finger_position[1]);
  to.push_back(pos.finger_position[2]);
}




void
JacoArmKindrv::stop()
{
  arm_->release_joystick();
  final_ = true;
}

void
JacoArmKindrv::push_joystick(unsigned int button)
{
  arm_->start_api_ctrl();
  arm_->push_joystick_button(button);
  final_ = false;
}

void
JacoArmKindrv::release_joystick()
{
  arm_->start_api_ctrl();
  arm_->release_joystick();
  final_ = true;
}


void
JacoArmKindrv::goto_trajec(std::vector< std::vector<float> >* trajec, std::vector<float> &fingers)
{
  arm_->start_api_ctrl();
  arm_->set_control_ang();
  ctrl_ang_ = true;
  usleep(500);
  for( unsigned int i=0; i<trajec->size(); ++i ) {
    arm_->set_target_ang(trajec->at(i).at(0), trajec->at(i).at(1), trajec->at(i).at(2),
                          trajec->at(i).at(3), trajec->at(i).at(4), trajec->at(i).at(5),
                          fingers.at(0), fingers.at(1), fingers.at(2));
  }
}

void
JacoArmKindrv::goto_joints(std::vector<float> &joints, std::vector<float> &fingers, bool followup)
{
  target_type_ = TARGET_ANGULAR;
  final_ = false;

  if(!followup) {
    arm_->start_api_ctrl();
    arm_->set_control_ang();
    ctrl_ang_ = true;
    usleep(500);
  }

  arm_->set_target_ang(joints.at(0), joints.at(1), joints.at(2), joints.at(3), joints.at(4), joints.at(5),
                        fingers.at(0), fingers.at(1), fingers.at(2));
}

void
JacoArmKindrv::goto_coords(std::vector<float> &coords, std::vector<float> &fingers)
{
  target_type_ = TARGET_CARTESIAN;
  final_ = false;

  arm_->start_api_ctrl();
  arm_->set_control_cart();
  ctrl_ang_ = false;
  usleep(500);
  //arm_->arm->set_target_cart(y_, -x_, z_, e1_, e2_, e3_, f1_, f2_, f3_);
  arm_->set_target_cart(coords.at(1), -coords.at(0), coords.at(2), coords.at(3), coords.at(4), coords.at(5),
                         fingers.at(0), fingers.at(1), fingers.at(2));
}

void
JacoArmKindrv::goto_ready()
{
  target_type_ = TARGET_READY;
  final_ = false;

  arm_->start_api_ctrl();
  jaco_retract_mode_t mode = arm_->get_status();
  switch( mode ) {
    case MODE_RETRACT_TO_READY:
      //2 buttons needed
      arm_->push_joystick_button(2);
      arm_->release_joystick();
      arm_->push_joystick_button(2);
      break;

    case MODE_NORMAL_TO_READY:
    case MODE_READY_TO_RETRACT:
    case MODE_RETRACT_STANDBY:
    case MODE_NORMAL:
    case MODE_NOINIT:
      //1 button needed
      arm_->push_joystick_button(2);
      break;

    case MODE_ERROR:
      // error: some error occured
      // TODO: return something?
      break;

    case MODE_READY_STANDBY:
      // no action. error?
      // final_ = true;
      break;
  }
}

void
JacoArmKindrv::goto_retract()
{
  target_type_ = TARGET_RETRACT;
  final_ = false;

  arm_->start_api_ctrl();
  jaco_retract_mode_t mode = arm_->get_status();
  switch( mode ) {
    case MODE_READY_TO_RETRACT:
      // 2 buttons needed
      arm_->push_joystick_button(2);
      arm_->release_joystick();
      arm_->push_joystick_button(2);
      break;

    case MODE_READY_STANDBY:
    case MODE_RETRACT_TO_READY:
      // 1 button needed
      arm_->push_joystick_button(2);
      break;

    case MODE_NORMAL_TO_READY:
    case MODE_NORMAL:
    case MODE_NOINIT:
      // warn: cannot go from NORMAL/NOINIT to RETRACT");
      //final_ = true;
      break;

    case MODE_ERROR:
      // error: some error occured!!
      // TODO: return something?
      break;

    case MODE_RETRACT_STANDBY:
      // no action. error?
      //final_ = true;
      break;
  }
}

} // end of namespace fawkes
