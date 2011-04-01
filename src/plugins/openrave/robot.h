
/***************************************************************************
 *  robot.h - Fawkes to OpenRAVE Robot Handler
 *
 *  Created: Mon Sep 20 14:50:34 2010
 *  Copyright  2010  Bahram Maleki-Fard, AllemaniACs RoboCup Team
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

#ifndef __PLUGINS_OPENRAVE_ROBOT_H
#define __PLUGINS_OPENRAVE_ROBOT_H

#include "types.h"

#include <rave/rave.h>
#include <vector>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class Logger;
class OpenRAVEManipulator;
class OpenRAVEEnvironment;

/** OpenRAVE Robot class */
class OpenRAVERobot
{
 public:
  OpenRAVERobot(fawkes::Logger* logger = 0);
  OpenRAVERobot(const std::string& filename, fawkes::OpenRAVEEnvironment* env, fawkes::Logger* logger = 0);
  virtual ~OpenRAVERobot();

  // build/load robot parts
  virtual void load(const std::string& filename, fawkes::OpenRAVEEnvironment* env);
  virtual void set_ready();
  virtual void set_offset(float trans_x, float trans_y, float trans_z);
  virtual void calibrate(float device_trans_x, float device_trans_y, float device_trans_z);
  virtual void set_manipulator(fawkes::OpenRAVEManipulator* manip);
  virtual void update_manipulator(); // not needed

  virtual bool attach_object(OpenRAVE::KinBodyPtr object);
  virtual bool attach_object(const std::string& name, fawkes::OpenRAVEEnvironment* env);
  virtual bool release_object(OpenRAVE::KinBodyPtr object);
  virtual bool release_object(const std::string& name, fawkes::OpenRAVEEnvironment* env);
  virtual bool release_all_objects();

  virtual bool set_target_quat	 (float trans_x, float trans_y, float trans_z, float quat_w, float quat_x, float quat_y, float quat_z, bool no_offset = false);
  virtual bool set_target_axis_angle(float trans_x, float trans_y, float trans_z, float angle, float axisX, float axisY, float axisZ, bool no_offset = false);
  virtual bool set_target_euler(euler_rotation_t type, float trans_x, float trans_y, float trans_z, float phi, float theta, float psi, bool no_offset = false);
  virtual void set_target_angles( std::vector<float>& angles ); // just temporary

  virtual bool set_target_object_position(float trans_x, float trans_y, float trans_z, float rot_x);

  virtual void get_target_angles(std::vector<float>& to); // not needed
  virtual OpenRAVE::RobotBasePtr get_robot_ptr() const;
  virtual OpenRAVE::PlannerBase::PlannerParametersPtr get_planner_params() const;
  virtual std::vector< std::vector<float> >* get_trajectory() const;
  virtual std::vector< std::vector<float> >* get_trajectory_device() const;

 private:
  void init();
  bool set_target_transform(OpenRAVE::Vector& trans, OpenRAVE::Vector& rotQuat, bool no_offset = false);
  bool set_target_euler(OpenRAVE::Vector& trans, std::vector<float>& rotations, bool no_offset = false);

  fawkes::Logger*	                __logger;

  OpenRAVE::RobotBasePtr                __robot;
  std::string                           __name;
  OpenRAVEManipulator*	                __manip;
  OpenRAVEManipulator*	                __manip_goal;
  OpenRAVE::RobotBase::ManipulatorPtr   __arm;

  OpenRAVE::PlannerBase::PlannerParametersPtr   __planner_params;
  std::vector< std::vector<float> >*            __traj;
  std::vector<float>                            __angles_target;

  float         __trans_offset_x;
  float         __trans_offset_y;
  float         __trans_offset_z;
};

} // end of namespace fawkes

#endif