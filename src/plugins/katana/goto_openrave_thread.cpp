
/***************************************************************************
 *  goto_openrave_thread.cpp - Katana goto one-time thread using openrave lib
 *
 *  Created: Wed Jun 10 11:45:31 2009
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
 *             2011-2014  Bahram Maleki-Fard
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
#include "controller.h"
#include "exception.h"

#include <interfaces/KatanaInterface.h>

#include <cstdlib>

#include <utils/time/time.h>

#ifdef HAVE_OPENRAVE
#include <plugins/openrave/aspect/openrave_connector.h>

#include <plugins/openrave/robot.h>
#include <plugins/openrave/environment.h>
#include <plugins/openrave/manipulators/katana6M180.h>
#include <plugins/openrave/manipulators/neuronics_katana.h>

#include <vector>
#endif
using namespace fawkes;

/** @class KatanaGotoOpenRaveThread "goto_openrave_thread.h"
 * Katana collision-free goto thread.
 * This thread moves the arm into a specified position,
 * using IK and path-planning from OpenRAVE.
 * @author Tim Niemueller (goto_thread.h/cpp)
 * @author Bahram Maleki-Fard (OpenRAVE extension)
 */

#ifdef HAVE_OPENRAVE

/// @cond SELFEXPLAINING
const std::string KatanaGotoOpenRaveThread::DEFAULT_PLANNERPARAMS =
                  "minimumgoalpaths 16 postprocessingparameters <_nmaxiterations>100</_nmaxiterations>"
                  "<_postprocessing planner=\"parabolicsmoother\"><_nmaxiterations>200</_nmaxiterations>"
                  "</_postprocessing>\n";
const std::string KatanaGotoOpenRaveThread::DEFAULT_PLANNERPARAMS_STRAIGHT =
                  "maxdeviationangle 0.05";
/// @endcond


/** Constructor.
 * @param katana katana controller base class
 * @param logger logger
 * @param openrave pointer to OpenRaveConnector aspect
 * @param poll_interval_ms interval in ms between two checks if the
 * final position has been reached
 * @param robot_file path to robot's xml-file
 * @param arm_model arm model used in robot_file, either "5dof" or "6dof_dummy"
 * @param autoload_IK true, if IK databas should be automatically generated (recommended)
 * @param use_viewer true, if viewer should be started (default: false)
 */
KatanaGotoOpenRaveThread::KatanaGotoOpenRaveThread(fawkes::RefPtr<fawkes::KatanaController> katana,
                                   fawkes::Logger *logger,
                                   fawkes::OpenRaveConnector* openrave,
                                   unsigned int poll_interval_ms,
                                   std::string robot_file,
                                   std::string arm_model,
                                   bool autoload_IK,
                                   bool use_viewer)
  : KatanaMotionThread("KatanaGotoOpenRaveThread", katana, logger),
  __target_object( "" ),
  __target_traj( 0 ),
  __cfg_robot_file( robot_file ),
  __cfg_arm_model( arm_model ),
  __cfg_autoload_IK( autoload_IK ),
  __cfg_use_viewer( use_viewer ),
  __is_target_object( 0 ),
  __has_target_quaternion( 0 ),
  __move_straight( 0 ),
  __is_arm_extension( 0 ),
  __plannerparams( "default" ),
  __plannerparams_straight( "default" ),
  _openrave( openrave ),
  __theta_error( 0 )
{
}


/** Set target position.
 * @param x X coordinate relative to base
 * @param y Y coordinate relative to base
 * @param z Z coordinate relative to base
 * @param phi Phi Euler angle of tool
 * @param theta Theta Euler angle of tool
 * @param psi Psi Euler angle of tool
 */
void
KatanaGotoOpenRaveThread::set_target(float x, float y, float z,
                                     float phi, float theta, float psi)
{
  __x     = x;
  __y     = y;
  __z     = z;
  __phi   = (phi);
  __theta = (theta);
  __psi   = (psi);

  __has_target_quaternion = false;
  __is_target_object = false;
  __move_straight = false;
  __is_arm_extension = false;
}

/** Set target position.
 * @param x X coordinate relative to base
 * @param y Y coordinate relative to base
 * @param z Z coordinate relative to base
 * @param quat_x x value of quaternion for tool's rotation
 * @param quat_y y value of quaternion for tool's rotation
 * @param quat_z z value of quaternion for tool's rotation
 * @param quat_w w value of quaternion for tool's rotation
 */
void
KatanaGotoOpenRaveThread::set_target(float x, float y, float z,
                                     float quat_x, float quat_y, float quat_z, float quat_w)
{
  __x      = x;
  __y      = y;
  __z      = z;
  __quat_x = quat_x;
  __quat_y = quat_y;
  __quat_z = quat_z;
  __quat_w = quat_w;

  __has_target_quaternion = true;
  __is_target_object = false;
  __move_straight = false;
  __is_arm_extension = false;
}

/** Set target position.
 * @param object_name name of the object (kinbody) in OpenRaveEnvironment
 */
void
KatanaGotoOpenRaveThread::set_target(const std::string& object_name, float rot_x)
{
  __target_object = object_name;

  __is_target_object = true;
}

/** Set theta error
 * @param error error in radians
 */
void
KatanaGotoOpenRaveThread::set_theta_error(float error)
{
  __theta_error = error;
}

/** Set if arm should move straight.
 * Make sure to call this after(!) a "set_target" method, as they
 * set "__move_straight" attribute to its default value.
 * @param move_straight true, if arm should move straight
 */
void
KatanaGotoOpenRaveThread::set_move_straight(bool move_straight)
{
  __move_straight = move_straight;
}

/** Set if target is taken as arm extension.
 * Make sure to call this after(!) a "set_target" method, as they
 * set "__move_straight" attribute to its default value.
 * @param arm_extension true, if target is regarded as arm extension
 */
void
KatanaGotoOpenRaveThread::set_arm_extension(bool arm_extension)
{
  __is_arm_extension = arm_extension;
}

/** Set plannerparams.
 * @param params plannerparameters. For further information, check openrave plugin, or OpenRAVE documentaiton.
 * @param straight true, if these params are for straight movement
 */
void
KatanaGotoOpenRaveThread::set_plannerparams(std::string& params, bool straight)
{
  if( straight ) {
    __plannerparams_straight = params;
  } else {
    __plannerparams = params;
  }
}

/** Set plannerparams.
 * @param params plannerparameters. For further information, check openrave plugin, or OpenRAVE documentaiton.
 * @param straight true, if these params are for straight movement
 */
void
KatanaGotoOpenRaveThread::set_plannerparams(const char* params, bool straight)
{
  if( straight ) {
    __plannerparams_straight = params;
  } else {
    __plannerparams = params;
  }
}

void
KatanaGotoOpenRaveThread::init()
{
  try {
    __OR_robot = _openrave->add_robot(__cfg_robot_file, false);
  } catch (Exception& e) {
    throw fawkes::Exception("Could not add robot '%s' to openrave environment", __cfg_robot_file.c_str());
  }

  try {
    // configure manipulator
    // TODO: from config parameters? neccessary?
    if( __cfg_arm_model == "5dof" ) {
      __OR_manip = new OpenRaveManipulatorNeuronicsKatana(5, 5);
      __OR_manip->add_motor(0,0);
      __OR_manip->add_motor(1,1);
      __OR_manip->add_motor(2,2);
      __OR_manip->add_motor(3,3);
      __OR_manip->add_motor(4,4);

      // Set manipulator and offsets.
      // offsetZ: katana.kinbody is 0.165 above ground; coordinate system of real katana has origin in intersection of j1 and j2 (i.e. start of link L2: 0.2015 on z-axis)
      // offsetX: katana.kinbody is setup 0.0725 on +x axis
      _openrave->set_manipulator(__OR_robot, __OR_manip, 0.f, 0.f, 0.f);
      __OR_robot->get_robot_ptr()->SetActiveManipulator("arm_kni");

      if( __cfg_autoload_IK ) {
        _openrave->get_environment()->load_IK_solver(__OR_robot, OpenRAVE::IKP_TranslationDirection5D);
      }
    } else if ( __cfg_arm_model == "6dof_dummy" ) {
      __OR_manip = new OpenRaveManipulatorKatana6M180(6, 5);
      __OR_manip->add_motor(0,0);
      __OR_manip->add_motor(1,1);
      __OR_manip->add_motor(2,2);
      __OR_manip->add_motor(4,3);
      __OR_manip->add_motor(5,4);

      // Set manipulator and offsets.
      // offsetZ: katana.kinbody is 0.165 above ground; coordinate system of real katana has origin in intersection of j1 and j2 (i.e. start of link L2: 0.2015 on z-axis)
      // offsetX: katana.kinbody is setup 0.0725 on +x axis
      _openrave->set_manipulator(__OR_robot, __OR_manip, 0.f, 0.f, 0.f);

      if( __cfg_autoload_IK ) {
        _openrave->get_environment()->load_IK_solver(__OR_robot, OpenRAVE::IKP_Transform6D);
      }
    } else {
      throw fawkes::Exception("Unknown entry for 'arm_model':%s", __cfg_arm_model.c_str());
    }

  } catch (Exception& e) {
    finalize();
    throw;
  }

  if( __cfg_use_viewer)
    _openrave->start_viewer();
}

void
KatanaGotoOpenRaveThread::finalize()
{
  _openrave->set_active_robot( NULL );
  __OR_robot = NULL;
  __OR_manip = NULL;
}

void
KatanaGotoOpenRaveThread::once()
{
#ifndef EARLY_PLANNING
  if( !plan_target() ) {
    _finished = true;
    return;
  }
#else
  if( _error_code != fawkes::KatanaInterface::ERROR_NONE ) {
    _finished = true;
    return;
  }
#endif

  // Get trajectories and move katana along them
  __target_traj = __OR_robot->get_trajectory_device();
  Time time_now, time_last = Time();
  try {
    bool final = false;
    __it = __target_traj->begin();
    while (!final) {
      time_last.stamp_systime();
      final = move_katana();
      update_openrave_data();
      time_now.stamp_systime();

      // Wait before sending next command. W.it until 5ms before reached time for next traj point
      // CAUTION! In order for this to work correctly, you need to assure that OpenRAVE model of the
      //   arm and the real device have the same velocity, i.e. need the same amount of time to complete
      //   a movement. Otherwise sampling over time and waiting does not make much sense.
      //   Disable the following line if requirement not fulfilled.

      //usleep(1000*1000*(sampling + time_last.in_sec() - time_now.in_sec() - 0.005f));
    }

    // check if encoders are close enough to target position
    final = false;
    while (!final) {
      final = true;
      update_openrave_data();
      try {
        final = _katana->final();
      } catch (fawkes::KatanaMotorCrashedException &e) {
        _logger->log_warn("KatanaGotoThread", e.what());
        _error_code = fawkes::KatanaInterface::ERROR_MOTOR_CRASHED;
        break;
      }
    }

  } catch (fawkes::Exception &e) {
    _logger->log_warn("KatanaGotoThread", "Moving along trajectory failed (ignoring): %s", e.what());
    _finished = true;
    _error_code = fawkes::KatanaInterface::ERROR_CMD_START_FAILED;
    _katana->stop();
    return;
  }


  // TODO: Check for finished motion
  // Can't use current version like in goto_thread, because target is
  // not set in libkni!  can check endeffector position instead, or
  // check last angle values from __target_traj with katana-arm values

  _finished = true;
}


/** Peform path-planning on target.
 * @return true if ik solvable and path planned, false otherwise
 */
bool
KatanaGotoOpenRaveThread::plan_target()
{
  // Fetch motor encoder values
  if( !update_motor_data() ) {
    _logger->log_warn("KatanaGotoThread", "Fetching current motor values failed");
    _error_code = fawkes::KatanaInterface::ERROR_COMMUNICATION;
    return false;
  }

  // Set starting point for planner, convert encoder values to angles if necessary
  if( !_katana->joint_angles() ) {
    encToRad(__motor_encoders, __motor_angles);
  }
  __OR_manip->set_angles_device(__motor_angles);

  // Checking if target has IK solution
  if( __plannerparams.compare("default") == 0 ) {
    __plannerparams = DEFAULT_PLANNERPARAMS;
  }
  if( __is_target_object) {
    _logger->log_debug(name(), "Check IK for object (%s)", __target_object.c_str());

    if( !_openrave->set_target_object(__target_object, __OR_robot) ) {
      _logger->log_warn("KatanaGotoThread", "Initiating goto failed, no IK solution found");
      _error_code = fawkes::KatanaInterface::ERROR_NO_SOLUTION;
      return false;
    }
  }
  else {
    bool success = false;
    try {
      if( __has_target_quaternion ) {
        _logger->log_debug(name(), "Check IK(%f,%f,%f  |  %f,%f,%f,%f)",
                           __x, __y, __z, __quat_x, __quat_y, __quat_z, __quat_w);
        success = __OR_robot->set_target_quat(__x, __y, __z, __quat_w, __quat_x, __quat_y, __quat_z);
      } else if( __move_straight ) {
        _logger->log_debug(name(), "Check IK(%f,%f,%f), straight movement",
                           __x, __y, __z);
        if( __is_arm_extension ) {
          success = __OR_robot->set_target_rel(__x, __y, __z, true);
        } else {
          success = __OR_robot->set_target_straight(__x, __y, __z);
        }
        if( __plannerparams_straight.compare("default") == 0 ) {
          __plannerparams_straight = DEFAULT_PLANNERPARAMS_STRAIGHT;
        }
      } else {
        float theta_error = 0.0f;
        while( !success && (theta_error <= __theta_error)) {
          _logger->log_debug(name(), "Check IK(%f,%f,%f  |  %f,%f,%f)",
                             __x, __y, __z, __phi, __theta+theta_error, __psi);
          success = __OR_robot->set_target_euler(EULER_ZXZ, __x, __y, __z, __phi, __theta+theta_error, __psi);
          if( !success ) {
            _logger->log_debug(name(), "Check IK(%f,%f,%f  |  %f,%f,%f)",
                               __x, __y, __z, __phi, __theta-theta_error, __psi);
            success = __OR_robot->set_target_euler(EULER_ZXZ, __x, __y, __z, __phi, __theta-theta_error, __psi);
          }

          theta_error += 0.01;
        }
      }
    } catch(OpenRAVE::openrave_exception &e) {
      _logger->log_debug(name(), "OpenRAVE exception:%s", e.what());
    }

    if( !success ) {
      _logger->log_warn("KatanaGotoThread", "Initiating goto failed, no IK solution found");
      _error_code = fawkes::KatanaInterface::ERROR_NO_SOLUTION;
      return false;
    }
  }
  if( __move_straight ) {
    __OR_robot->set_target_plannerparams(__plannerparams_straight);
  } else {
    __OR_robot->set_target_plannerparams(__plannerparams);
  }

  // Run planner
  float sampling = 0.04f; //maybe catch from config? or "learning" depending on performance?
  try {
    _openrave->run_planner(__OR_robot, sampling);
  } catch (fawkes::Exception &e) {
    _logger->log_warn("KatanaGotoThread", "Planner failed (ignoring): %s", e.what());
    _error_code = fawkes::KatanaInterface::ERROR_UNSPECIFIC;
    return false;
  }

  _error_code = fawkes::KatanaInterface::ERROR_NONE;
  return true;
}

/** Update data of arm in OpenRAVE model */
void
KatanaGotoOpenRaveThread::update_openrave_data()
{
  // Fetch motor encoder values
  if( !update_motor_data() ) {
    _logger->log_warn("KatanaGotoThread", "Fetching current motor values failed");
    _finished = true;
    return;
  }

  // Convert encoder values to angles if necessary
  if( !_katana->joint_angles() ) {
    encToRad(__motor_encoders, __motor_angles);
  }
  __OR_manip->set_angles_device(__motor_angles);

  std::vector<OpenRAVE::dReal> angles;
  __OR_manip->get_angles(angles);

  {
    OpenRAVE::EnvironmentMutex::scoped_lock lock(__OR_robot->get_robot_ptr()->GetEnv()->GetMutex());
    __OR_robot->get_robot_ptr()->SetActiveDOFValues(angles);
  }
}

/** Update motors and fetch current encoder values.
 * @return true if succesful, false otherwise
 */
bool
KatanaGotoOpenRaveThread::update_motor_data()
{
  short num_errors  = 0;

  // update motors
  while (1) {
    //usleep(__poll_interval_usec);
    try {
      _katana->read_sensor_data(); // update sensor data
      _katana->read_motor_data(); // update motor data
    } catch (Exception &e) {
      if (++num_errors <= 10) {
        _logger->log_warn("KatanaGotoThread", "Receiving motor data failed, retrying");
        continue;
      } else {
        _logger->log_warn("KatanaGotoThread", "Receiving motor data failed too often, aborting");
        _error_code = fawkes::KatanaInterface::ERROR_COMMUNICATION;
        return 0;
      }
    }
    break;
  }

  // fetch joint values
  num_errors = 0;
  while (1) {
    //usleep(__poll_interval_usec);
    try {
      if( _katana->joint_angles()) {
        _katana->get_angles(__motor_angles, false);    //fetch encoder values, param refreshEncoders=false
      } else {
        _katana->get_encoders(__motor_encoders, false);    //fetch encoder values, param refreshEncoders=false
      }
    } catch (Exception &e) {
      if (++num_errors <= 10) {
        _logger->log_warn("KatanaGotoThread", "Receiving motor %s failed, retrying", _katana->joint_angles() ? "angles" : "encoders");
        continue;
      } else {
        _logger->log_warn("KatanaGotoThread", "Receiving motor %s failed, aborting", _katana->joint_angles() ? "angles" : "encoders");
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
KatanaGotoOpenRaveThread::move_katana()
{
  if( _katana->joint_angles() ) {
    _katana->move_to(*__it, /*blocking*/false);
  } else {
    std::vector<int> enc;
    _katana->get_encoders(enc);
    _katana->move_to(enc, /*blocking*/false);
  }

  return (++__it == __target_traj->end());
}

#endif
