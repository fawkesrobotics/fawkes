
/***************************************************************************
 *  goto_openrave_thread.cpp - Katana goto one-time thread using openrave lib
 *
 *  Created: Wed Jun 10 11:45:31 2009
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
 *                  2010  Bahram Maleki-Fard
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

#include "goto_openrave_thread.h"
#include "conversion.h"

#include <cstdlib>
#include <kniBase.h>

#ifdef HAVE_OPENRAVE

#include <plugins/openrave/robot.h>
#include <plugins/openrave/manipulators/katana6M180.h>
#include <plugins/openrave/types.h>

#include <vector>
#endif
using namespace fawkes;

/** @class KatanaGotoOpenRAVEThread "goto_openrave_thread.h"
 * Katana collision-free goto thread.
 * This thread moves the arm into a specified position,
 * using IK and path-planning from OpenRAVE.
 * @author Tim Niemueller (goto_thread.h/cpp)
 * @author Bahram Maleki-Fard (OpenRAVE extension)
 */

/** Constructor.
 * @param katana linear motion base class
 * @param logger logger
 * @param openrave pointer to OpenRAVEConnector aspect
 * @param poll_interval_ms interval in ms between two checks if the
 * final position has been reached
 * @param robot_file path to robot's xml-file
 * @param autoload_IK true, if IK databas should be automatically generated (recommended)
 * @param use_viewer true, if viewer should be started (default: false)
 */
KatanaGotoOpenRAVEThread::KatanaGotoOpenRAVEThread(fawkes::RefPtr<CLMBase> katana,
				   fawkes::Logger *logger,
                                   fawkes::OpenRAVEConnector* openrave,
				   unsigned int poll_interval_ms,
                                   std::string robot_file,
                                   bool autoload_IK,
                                   bool use_viewer)
  : KatanaMotionThread("KatanaGotoOpenRAVEThread", katana, logger),
  __OR_robot( 0 ),
  __OR_manip( 0 ),
  __target_object( "" ),
  __target_traj( 0 ),
  __cfg_robot_file( robot_file ),
  __cfg_autoload_IK( autoload_IK ),
  __cfg_use_viewer( use_viewer ),
  __is_target_object( 0 ),
  _openrave( openrave )
{
}

#ifdef HAVE_OPENRAVE

/** Set target position.
 * @param x X coordinate relative to base
 * @param y Y coordinate relative to base
 * @param z Z coordinate relative to base
 * @param phi Phi Euler angle of tool
 * @param theta Theta Euler angle of tool
 * @param psi Psi Euler angle of tool
 */
void
KatanaGotoOpenRAVEThread::set_target(float x, float y, float z,
			     float phi, float theta, float psi)
{
  __x     = x;
  __y     = y;
  __z     = z;
  __phi   = (phi);
  __theta = (theta);
  __psi   = (psi);

  __is_target_object = false;
}

/** Set target position.
 * @param object_name name of the object (kinbody) in OpenRAVEEnvironment
 */
void
KatanaGotoOpenRAVEThread::set_target(const std::string& object_name, float rot_x)
{
  __target_object = object_name;

  __is_target_object = true;
}

void
KatanaGotoOpenRAVEThread::init()
{
  try {
    __OR_robot = _openrave->add_robot(__cfg_robot_file, __cfg_autoload_IK);

    // configure manipulator
    // TODO: from config parameters? neccessary?
    __OR_manip = new OpenRAVEManipulatorKatana6M180(6, 5);
    __OR_manip->add_motor(0,0);
    __OR_manip->add_motor(1,1);
    __OR_manip->add_motor(2,2);
    __OR_manip->add_motor(4,3);
    __OR_manip->add_motor(5,4);

    // Set manipulator and offsets.
    // offsetZ: katana.kinbody is 0.165 above ground; coordinate system of real katana has origin in intersection of j1 and j2 (i.e. start of link L2: 0.2015 on z-axis)
    // offsetX: katana.kinbody is setup 0.0725 on +x axis
    _openrave->set_manipulator(__OR_robot, __OR_manip, 0.f, 0.f, 0.f);
  } catch (fawkes::Exception& e) {
    // TODO: not just simple throw....
    throw;
  } catch (/*KNI*/::Exception &e) {
    _logger->log_warn(name(), "Fetching position values failed: %s", e.what());
  }

  if( __cfg_use_viewer)
    _openrave->start_viewer();
}

void
KatanaGotoOpenRAVEThread::finalize()
{
  delete(__OR_robot);
  __OR_robot = NULL;

  delete(__OR_manip);
  __OR_manip = NULL;
}

void
KatanaGotoOpenRAVEThread::once()
{
  // Fetch motor encoder values
  if( !update_motor_data() ) {
    _logger->log_warn("KatanaGotoThread", "Fetching current motor values failed");
    _finished = true;
    return;
  }

  // Convert encoder values to angles, and set starting point for planner
  encToRad(__motor_encoders, __motor_angles);
  __OR_manip->set_angles_device(__motor_angles);

  // Checking if target has IK solution
  if( __is_target_object) {
    _logger->log_debug(name(), "Check IK for object (%s)", __target_object.c_str());

    if( !_openrave->set_target_object(__target_object, __OR_robot) ) {
      _logger->log_warn("KatanaGotoThread", "Initiating goto failed, no IK solution found");
      _finished = true;
      _error_code = fawkes::KatanaInterface::ERROR_NO_SOLUTION;
      return;
    }
  }
  else {
    _logger->log_debug(name(), "Check IK(%f,%f,%f, %f,%f,%f)",
		       __x, __y, __z, __phi, __theta, __psi);
    if( !__OR_robot->set_target_euler(EULER_ZXZ, __x, __y, __z, __phi, __theta, __psi) ) {
      _logger->log_warn("KatanaGotoThread", "Initiating goto failed, no IK solution found");
      _finished = true;
      _error_code = fawkes::KatanaInterface::ERROR_NO_SOLUTION;
      return;
    }
  }

  // Run planner
  try {
    _openrave->run_planner(__OR_robot);
  } catch (fawkes::Exception &e) {
    _logger->log_warn("KatanaGotoThread", "Planner failed (ignoring): %s", e.what());
    _finished = true;
    _error_code = fawkes::KatanaInterface::ERROR_UNSPECIFIC;
    return;
  }

  // Get trajectories and move katana along them
  __target_traj = __OR_robot->get_trajectory_device();
  try {
    bool final = false;
    __it = __target_traj->begin();
    while (!final) {
      final = move_katana();

      update_openrave_data();
    }

  } catch (/*KNI*/::Exception &e) {
    _logger->log_warn("KatanaGotoThread", "Moving along trajectory failed (ignoring): %s", e.what());
    _finished = true;
    _error_code = fawkes::KatanaInterface::ERROR_CMD_START_FAILED;
    _katana->freezeRobot();
    return;
  }


  // TODO: Check for finished motion
  // Can't use current version like in goto_thread, because target is
  // not set in libkni!  can check endeffector position instead, or
  // check last angle values from __target_traj with katana-arm values

  _finished = true;
}


/** Update data of arm in OpenRAVE model */
void
KatanaGotoOpenRAVEThread::update_openrave_data()
{
  // Fetch motor encoder values
  if( !update_motor_data() ) {
    _logger->log_warn("KatanaGotoThread", "Fetching current motor values failed");
    _finished = true;
    return;
  }

  // Convert encoder values to angles, and set starting point for planner
  encToRad(__motor_encoders, __motor_angles);
  __OR_manip->set_angles_device(__motor_angles);

  std::vector<float> angles;
  __OR_manip->get_angles(angles);

  __OR_robot->get_robot_ptr()->SetActiveDOFValues(angles);
}

/** Update motors and fetch current encoder values.
 * @return true if succesful, false otherwise
 */
bool
KatanaGotoOpenRAVEThread::update_motor_data()
{
  CKatBase *base = _katana->GetBase();
  short num_errors  = 0;

  // update motors
  while (1) {
    //usleep(__poll_interval_usec);
    try {
      base->GetSCT()->arr[0].recvDAT(); // update sensor values
      base->recvMPS(); // get position for all motors
      base->recvGMS(); // get status flags for all motors
    } catch (/*KNI*/::Exception &e) {
      if (++num_errors <= 10) {
        _logger->log_warn("KatanaGotoThread", "Receiving MPS/GMS failed, retrying");
        continue;
      } else {
        _logger->log_warn("KatanaGotoThread", "Receiving MPS/GMS failed too often, aborting");
        _error_code = fawkes::KatanaInterface::ERROR_COMMUNICATION;
        return 0;
      }
    }
    break;
  }

  // fetch encoder values
  num_errors = 0;
  while (1) {
    //usleep(__poll_interval_usec);
    try {
      __motor_encoders = _katana->getRobotEncoders(false); //fetch encoder values, param refreshEncoders=false
    } catch (/*KNI*/::Exception &e) {
      if (++num_errors <= 10) {
        _logger->log_warn("KatanaGotoThread", "Receiving encoder values failed, retrying");
        continue;
      } else {
        _logger->log_warn("KatanaGotoThread", "Receiving encoder values failed, aborting");
        _error_code = fawkes::KatanaInterface::ERROR_COMMUNICATION;
        return 0;
      }
    }
    break;
  }

  return 1;
}


/** Realize next trajectory point.
 * Take the next point from the current trajectory __target_traj, set its
 * joint values and advance the iterator.
 * @return true if the trajectory is finished, i.e. there are no more points
 * left, false otherwise.
 */
bool
KatanaGotoOpenRAVEThread::move_katana()
{
  for (unsigned int i = 0; i < __it->size(); ++i) {
    _katana->moveMotorTo(i, __it->at(i));
  }
  return (++__it == __target_traj->end());
}

#endif
