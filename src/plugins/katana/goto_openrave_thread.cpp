
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
                                   const std::string& robot_file,
                                   const std::string& arm_model,
                                   bool autoload_IK,
                                   bool use_viewer)
: KatanaMotionThread("KatanaGotoOpenRaveThread", katana, logger),
  target_object_( "" ),
  target_traj_( 0 ),
  cfg_robot_file_( robot_file ),
  cfg_arm_model_( arm_model ),
  cfg_autoload_IK_( autoload_IK ),
  cfg_use_viewer_( use_viewer ),
  is_target_object_( 0 ),
  has_target_quaternion_( 0 ),
  move_straight_( 0 ),
  is_arm_extension_( 0 ),
  plannerparams_( "default" ),
  plannerparams_straight_( "default" ),
  _openrave( openrave ),
  theta_error_( 0 )
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
  x_     = x;
  y_     = y;
  z_     = z;
  phi_   = (phi);
  theta_ = (theta);
  psi_   = (psi);

  has_target_quaternion_ = false;
  is_target_object_ = false;
  move_straight_ = false;
  is_arm_extension_ = false;
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
  x_      = x;
  y_      = y;
  z_      = z;
  quat_x_ = quat_x;
  quat_y_ = quat_y;
  quat_z_ = quat_z;
  quat_w_ = quat_w;

  has_target_quaternion_ = true;
  is_target_object_ = false;
  move_straight_ = false;
  is_arm_extension_ = false;
}

/** Set target position.
 * @param object_name name of the object (kinbody) in OpenRaveEnvironment
 */
void
KatanaGotoOpenRaveThread::set_target(const std::string& object_name, float rot_x)
{
  target_object_ = object_name;

  is_target_object_ = true;
}

/** Set theta error
 * @param error error in radians
 */
void
KatanaGotoOpenRaveThread::set_theta_error(float error)
{
  theta_error_ = error;
}

/** Set if arm should move straight.
 * Make sure to call this after(!) a "set_target" method, as they
 * set "move_straight_" attribute to its default value.
 * @param move_straight true, if arm should move straight
 */
void
KatanaGotoOpenRaveThread::set_move_straight(bool move_straight)
{
  move_straight_ = move_straight;
}

/** Set if target is taken as arm extension.
 * Make sure to call this after(!) a "set_target" method, as they
 * set "move_straight_" attribute to its default value.
 * @param arm_extension true, if target is regarded as arm extension
 */
void
KatanaGotoOpenRaveThread::set_arm_extension(bool arm_extension)
{
  is_arm_extension_ = arm_extension;
}

/** Set plannerparams.
 * @param params plannerparameters. For further information, check openrave plugin, or OpenRAVE documentaiton.
 * @param straight true, if these params are for straight movement
 */
void
KatanaGotoOpenRaveThread::set_plannerparams(std::string& params, bool straight)
{
  if( straight ) {
    plannerparams_straight_ = params;
  } else {
    plannerparams_ = params;
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
    plannerparams_straight_ = params;
  } else {
    plannerparams_ = params;
  }
}

void
KatanaGotoOpenRaveThread::init()
{
  try {
    OR_robot_ = _openrave->add_robot(cfg_robot_file_, false);
  } catch (Exception& e) {
    throw fawkes::Exception("Could not add robot '%s' to openrave environment", cfg_robot_file_.c_str());
  }

  try {
    // configure manipulator
    // TODO: from config parameters? neccessary?
    if( cfg_arm_model_ == "5dof" ) {
      OR_manip_ = new OpenRaveManipulatorNeuronicsKatana(5, 5);
      OR_manip_->add_motor(0,0);
      OR_manip_->add_motor(1,1);
      OR_manip_->add_motor(2,2);
      OR_manip_->add_motor(3,3);
      OR_manip_->add_motor(4,4);

      // Set manipulator and offsets.
      // offsetZ: katana.kinbody is 0.165 above ground; coordinate system of real katana has origin in intersection of j1 and j2 (i.e. start of link L2: 0.2015 on z-axis)
      // offsetX: katana.kinbody is setup 0.0725 on +x axis
      _openrave->set_manipulator(OR_robot_, OR_manip_, 0.f, 0.f, 0.f);
      OR_robot_->get_robot_ptr()->SetActiveManipulator("arm_kni");

      if( cfg_autoload_IK_ ) {
        _openrave->get_environment()->load_IK_solver(OR_robot_, OpenRAVE::IKP_TranslationDirection5D);
      }
    } else if ( cfg_arm_model_ == "6dof_dummy" ) {
      OR_manip_ = new OpenRaveManipulatorKatana6M180(6, 5);
      OR_manip_->add_motor(0,0);
      OR_manip_->add_motor(1,1);
      OR_manip_->add_motor(2,2);
      OR_manip_->add_motor(4,3);
      OR_manip_->add_motor(5,4);

      // Set manipulator and offsets.
      // offsetZ: katana.kinbody is 0.165 above ground; coordinate system of real katana has origin in intersection of j1 and j2 (i.e. start of link L2: 0.2015 on z-axis)
      // offsetX: katana.kinbody is setup 0.0725 on +x axis
      _openrave->set_manipulator(OR_robot_, OR_manip_, 0.f, 0.f, 0.f);

      if( cfg_autoload_IK_ ) {
        _openrave->get_environment()->load_IK_solver(OR_robot_, OpenRAVE::IKP_Transform6D);
      }
    } else {
      throw fawkes::Exception("Unknown entry for 'arm_model':%s", cfg_arm_model_.c_str());
    }

  } catch (Exception& e) {
    finalize();
    throw;
  }

  if( cfg_use_viewer_)
    _openrave->start_viewer();
}

void
KatanaGotoOpenRaveThread::finalize()
{
  _openrave->set_active_robot( NULL );
  OR_robot_ = NULL;
  OR_manip_ = NULL;
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
  target_traj_ = OR_robot_->get_trajectory_device();
  Time time_now, time_last = Time();
  try {
    bool final = false;
    it_ = target_traj_->begin();
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
        _logger->log_warn("KatanaGotoThread", "Motor crashed: %s", e.what());
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
  // check last angle values from target_traj_ with katana-arm values

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
    encToRad(motor_encoders_, motor_angles_);
  }
  OR_manip_->set_angles_device(motor_angles_);

  // Checking if target has IK solution
  if( plannerparams_.compare("default") == 0 ) {
    plannerparams_ = DEFAULT_PLANNERPARAMS;
  }
  if( is_target_object_) {
    _logger->log_debug(name(), "Check IK for object (%s)", target_object_.c_str());

    if( !_openrave->set_target_object(target_object_, OR_robot_) ) {
      _logger->log_warn("KatanaGotoThread", "Initiating goto failed, no IK solution found");
      _error_code = fawkes::KatanaInterface::ERROR_NO_SOLUTION;
      return false;
    }
  }
  else {
    bool success = false;
    try {
      if( has_target_quaternion_ ) {
        _logger->log_debug(name(), "Check IK(%f,%f,%f  |  %f,%f,%f,%f)",
                           x_, y_, z_, quat_x_, quat_y_, quat_z_, quat_w_);
        success = OR_robot_->set_target_quat(x_, y_, z_, quat_w_, quat_x_, quat_y_, quat_z_);
      } else if( move_straight_ ) {
        _logger->log_debug(name(), "Check IK(%f,%f,%f), straight movement",
                           x_, y_, z_);
        if( is_arm_extension_ ) {
          success = OR_robot_->set_target_rel(x_, y_, z_, true);
        } else {
          success = OR_robot_->set_target_straight(x_, y_, z_);
        }
        if( plannerparams_straight_.compare("default") == 0 ) {
          plannerparams_straight_ = DEFAULT_PLANNERPARAMS_STRAIGHT;
        }
      } else {
        float theta_error = 0.0f;
        while( !success && (theta_error <= theta_error_)) {
          _logger->log_debug(name(), "Check IK(%f,%f,%f  |  %f,%f,%f)",
                             x_, y_, z_, phi_, theta_+theta_error, psi_);
          success = OR_robot_->set_target_euler(EULER_ZXZ, x_, y_, z_, phi_, theta_+theta_error, psi_);
          if( !success ) {
            _logger->log_debug(name(), "Check IK(%f,%f,%f  |  %f,%f,%f)",
                               x_, y_, z_, phi_, theta_-theta_error, psi_);
            success = OR_robot_->set_target_euler(EULER_ZXZ, x_, y_, z_, phi_, theta_-theta_error, psi_);
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
  if( move_straight_ ) {
    OR_robot_->set_target_plannerparams(plannerparams_straight_);
  } else {
    OR_robot_->set_target_plannerparams(plannerparams_);
  }

  // Run planner
  try {
	  float sampling = 0.04f; //maybe catch from config? or "learning" depending on performance?
    _openrave->run_planner(OR_robot_, sampling);
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
    encToRad(motor_encoders_, motor_angles_);
  }
  OR_manip_->set_angles_device(motor_angles_);

  std::vector<OpenRAVE::dReal> angles;
  OR_manip_->get_angles(angles);

  {
    OpenRAVE::EnvironmentMutex::scoped_lock lock(OR_robot_->get_robot_ptr()->GetEnv()->GetMutex());
    OR_robot_->get_robot_ptr()->SetActiveDOFValues(angles);
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
    //usleep(poll_interval_usec_);
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
    //usleep(poll_interval_usec_);
    try {
      if( _katana->joint_angles()) {
        _katana->get_angles(motor_angles_, false);    //fetch encoder values, param refreshEncoders=false
      } else {
        _katana->get_encoders(motor_encoders_, false);    //fetch encoder values, param refreshEncoders=false
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
 * Take the next point from the current trajectory target_traj_, set its
 * joint values and advance the iterator.
 * @return true if the trajectory is finished, i.e. there are no more points
 * left, false otherwise.
 */
bool
KatanaGotoOpenRaveThread::move_katana()
{
  if( _katana->joint_angles() ) {
    _katana->move_to(*it_, /*blocking*/false);
  } else {
    std::vector<int> enc;
    _katana->get_encoders(enc);
    _katana->move_to(enc, /*blocking*/false);
  }

  return (++it_ == target_traj_->end());
}

#endif
