
/***************************************************************************
 *  controller_openrave.cpp - OpenRAVE Controller class for katana arm
 *
 *  Created: Sat Jan 07 16:10:54 2012
 *  Copyright  2012-2014  Bahram Maleki-Fard
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

#include "controller_openrave.h"
#include "exception.h"

#include <core/exceptions/software.h>

#include <plugins/openrave/aspect/openrave_connector.h>

#ifdef HAVE_OPENRAVE
#include <plugins/openrave/environment.h>
#include <plugins/openrave/robot.h>
#include <plugins/openrave/manipulator.h>

#include <cmath>

using namespace OpenRAVE;
#endif

namespace fawkes {

/** @class KatanaControllerOpenrave <plugins/katana/controller_kni.h>
 * Controller class for a Neuronics Katana, using libkni to interact
 * with the real Katana arm.
 * @author Bahram Maleki-Fard
 */

#ifdef HAVE_OPENRAVE

/** Constructor.
 * @param openrave pointer to OpenRaveConnector aspect.
 */
KatanaControllerOpenrave::KatanaControllerOpenrave(fawkes::OpenRaveConnector* openrave)
{
  openrave_ = openrave;
  initialized_ = false;
}

/** Destructor. */
KatanaControllerOpenrave::~KatanaControllerOpenrave()
{
  // Setting to NULL also deletes instance (RefPtr)

  openrave_ = NULL;
  OR_env_   = NULL;
  OR_robot_ = NULL;
  OR_manip_ = NULL;
}


void
KatanaControllerOpenrave::init()
{
  try {
    OR_env_   = openrave_->get_environment();
    OR_robot_ = openrave_->get_active_robot();

    if( !OR_robot_ ) {
      throw fawkes::Exception("Cannot access OpenRaveRobot in current OpenRaveEnvironment.");
    }
 // TODO: get robot string and name, compare to this!
// robot_->GetName();

    OR_manip_ = OR_robot_->get_manipulator();
    env_ = OR_env_->get_env_ptr();
    robot_ = OR_robot_->get_robot_ptr();
    manip_ = robot_->GetActiveManipulator();
    initialized_ = true;

  } catch (OpenRAVE::openrave_exception &e) {
    throw fawkes::Exception("OpenRAVE Exception:%s", e.what());
  }
}

void
KatanaControllerOpenrave::set_max_velocity(unsigned int vel)
{
  check_init();
  try {
    EnvironmentMutex::scoped_lock lock(env_->GetMutex());
    std::vector<dReal> v;
    OR_manip_->get_angles(v);
    v.assign(v.size(), (dReal)(vel / 100.0));

    robot_->SetActiveDOFVelocities(v);
  } catch( /*OpenRAVE*/::openrave_exception &e) {
    throw fawkes::Exception("OpenRAVE Exception:%s", e.what());
  }
}


bool
KatanaControllerOpenrave::final()
{
  check_init();
  return robot_->GetController()->IsDone();
}

bool
KatanaControllerOpenrave::joint_angles()
{
  return true;
}
bool
KatanaControllerOpenrave::joint_encoders()
{
  return false;
}

void
KatanaControllerOpenrave::calibrate()
{
 // do nothing, arm in OpenRAVE does not need calibration
}

void
KatanaControllerOpenrave::stop()
{
  check_init();
  try {
    EnvironmentMutex::scoped_lock lock(env_->GetMutex());
    std::vector<dReal> v;
    robot_->GetActiveDOFValues(v);
    robot_->SetActiveDOFValues(v);
  } catch( /*OpenRAVE*/::openrave_exception &e) {
    throw fawkes::Exception("OpenRAVE Exception:%s", e.what());
  }
}

void
KatanaControllerOpenrave::turn_on()
{
}

void
KatanaControllerOpenrave::turn_off()
{
}

void
KatanaControllerOpenrave::read_coordinates(bool refresh)
{
  check_init();
  try {
    update_manipulator();
    EnvironmentMutex::scoped_lock lock(env_->GetMutex());
    Transform tf = manip_->GetEndEffectorTransform();
    x_ = tf.trans[0];
    y_ = tf.trans[1];
    z_ = tf.trans[2];
    //transform quat to euler.
    TransformMatrix m = matrixFromQuat(tf.rot);
    std::vector<dReal> v;
    OR_manip_->get_angles_device(v);
    phi_ = v.at(0) - 0.5*M_PI; //phi is directly derivable from joint0
    psi_ = 0.5*M_PI - v.at(4); //psi is directly derivable from joint4
    theta_ = acos(m.m[10]);
    //theta has correct range from 0-360°, but need to check if sign is also correct. use sinus for that
    if( asin(m.m[2] / sin(phi_)) < 0.0 )
      theta_ *= -1.0;
  } catch( /*OpenRAVE*/::openrave_exception &e ) {
    throw fawkes::Exception("OpenRAVE Exception:%s", e.what());
  }
}

void
KatanaControllerOpenrave::read_motor_data()
{
  //no need, simulation loop should always be running
}

void
KatanaControllerOpenrave::read_sensor_data()
{
  //no need, simulation loop should always be running
}

void
KatanaControllerOpenrave::gripper_open(bool blocking)
{

}

void
KatanaControllerOpenrave::gripper_close(bool blocking)
{

}

void
KatanaControllerOpenrave::move_to(float x, float y, float z, float phi, float theta, float psi, bool blocking)
{
  // This method is only here for conveniance, used by KNI
  throw fawkes::KatanaUnsupportedException("OpenRAVE Controller does not accept Euler Rotation");
}

void
KatanaControllerOpenrave::move_to(std::vector<int> encoders, bool blocking)
{
  throw fawkes::KatanaUnsupportedException("OpenRAVE Controller does not accept encoders");
}

void
KatanaControllerOpenrave::move_to(std::vector<float> angles, bool blocking)
{
  check_init();
  try {
    std::vector<dReal> v;
    OR_manip_->set_angles_device(angles);

    OR_manip_->get_angles(v);
    EnvironmentMutex::scoped_lock lock(env_->GetMutex());
    robot_->SetActiveDOFValues(v);
    usleep(2000);
  } catch( /*OpenRAVE*/::openrave_exception &e) {
    throw fawkes::Exception("OpenRAVE Exception:%s", e.what());
  }

  if( blocking ) {
    wait_finished();
  }
}

void
KatanaControllerOpenrave::move_motor_to(unsigned short id, int enc, bool blocking)
{
  throw fawkes::KatanaUnsupportedException("OpenRAVE Controller does not accept encoders");
}

void
KatanaControllerOpenrave::move_motor_to(unsigned short id, float angle, bool blocking)
{
  check_init();
  try {
    std::vector<dReal> v;
    OR_manip_->get_angles_device(v);
    v.at(id) = (dReal)angle;
    OR_manip_->set_angles_device(v);

    OR_manip_->get_angles(v);
    EnvironmentMutex::scoped_lock lock(env_->GetMutex());
    robot_->SetActiveDOFValues(v);
    usleep(2000);
  } catch( /*OpenRAVE*/::openrave_exception &e) {
    throw fawkes::Exception("OpenRAVE Exception:%s", e.what());
  }

  if( blocking ) {
    wait_finished();
  }
}

void
KatanaControllerOpenrave::move_motor_by(unsigned short id, int enc, bool blocking)
{
  throw fawkes::KatanaUnsupportedException("OpenRAVE Controller does not accept encoders");
}

void
KatanaControllerOpenrave::move_motor_by(unsigned short id, float angle, bool blocking)
{
  check_init();
  try {
    std::vector<dReal> v;
    OR_manip_->get_angles_device(v);
    v.at(id) += (dReal)angle;
    OR_manip_->set_angles_device(v);

    OR_manip_->get_angles(v);
    EnvironmentMutex::scoped_lock lock(env_->GetMutex());
    robot_->SetActiveDOFValues(v);
    usleep(2000);
  } catch( /*OpenRAVE*/::openrave_exception &e) {
    throw fawkes::Exception("OpenRAVE Exception:%s", e.what());
  }

  if( blocking ) {
    wait_finished();
  }
}


// getters
double
KatanaControllerOpenrave::x()
{
  return x_;
}

double
KatanaControllerOpenrave::y()
{
  return y_;
}

double
KatanaControllerOpenrave::z()
{
  return z_;
}

double
KatanaControllerOpenrave::phi()
{
  return phi_;
}

double
KatanaControllerOpenrave::theta()
{
  return theta_;
}

double
KatanaControllerOpenrave::psi()
{
  return psi_;
}

void
KatanaControllerOpenrave::get_sensors(std::vector<int>& to, bool refresh)
{
  check_init();
  to.clear();
  to.resize(0);
}

void
KatanaControllerOpenrave::get_encoders(std::vector<int>& to, bool refresh)
{
  throw fawkes::KatanaUnsupportedException("OpenRAVE Controller does not accept encoders");
}

void
KatanaControllerOpenrave::get_angles(std::vector<float>& to, bool refresh)
{
  check_init();
  try {
    EnvironmentMutex::scoped_lock lock(env_->GetMutex());
    std::vector<dReal> v;
    robot_->GetActiveDOFValues(v);
    OR_manip_->set_angles(v);

    OR_manip_->get_angles_device(to);
  } catch( /*OpenRAVE*/::openrave_exception &e) {
    throw fawkes::Exception("OpenRAVE Exception:%s", e.what());
  }
}


void
KatanaControllerOpenrave::update_manipulator()
{
  EnvironmentMutex::scoped_lock lock(env_->GetMutex());
  manip_ = robot_->GetActiveManipulator();
}

void
KatanaControllerOpenrave::wait_finished()
{
  while( !final() ) {
    usleep(1000);
  }
}

bool
KatanaControllerOpenrave::motor_oor(unsigned short id)
{
  check_init();
  std::vector<dReal> v;
  OR_manip_->get_angles_device(v);

  return id > v.size();
}

void
KatanaControllerOpenrave::check_init()
{
  if( !initialized_ ) {
    init();
    // init() will throw an exception if it fails
  }
}

#endif // HAVE_OPENRAVE

} // end of namespace fawkes
