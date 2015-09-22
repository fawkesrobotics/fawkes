
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

#include <openrave/openrave.h>
#include <vector>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class Logger;

/** OpenRAVE Robot class */
class OpenRaveRobot
{
 public:
  OpenRaveRobot(fawkes::Logger* logger = 0);
  OpenRaveRobot(const std::string& filename, fawkes::OpenRaveEnvironmentPtr& env, fawkes::Logger* logger = 0);
  OpenRaveRobot(const fawkes::OpenRaveRobot& src, const fawkes::OpenRaveEnvironmentPtr& new_env);
  virtual ~OpenRaveRobot();

  // build/load robot parts
  virtual void load(const std::string& filename, fawkes::OpenRaveEnvironmentPtr& env);
  virtual void set_ready();
  virtual void set_offset(float trans_x, float trans_y, float trans_z);
  virtual void calibrate(float device_trans_x, float device_trans_y, float device_trans_z);
  virtual void set_manipulator(fawkes::OpenRaveManipulatorPtr& manip, bool display_movements = false);
  virtual void update_manipulator();
  virtual void update_model();

  virtual bool attach_object(OpenRAVE::KinBodyPtr object, const char* manip_name = NULL);
  virtual bool attach_object(const char* name, fawkes::OpenRaveEnvironmentPtr& env, const char* manip_name = NULL);
  virtual bool release_object(OpenRAVE::KinBodyPtr object);
  virtual bool release_object(const std::string& name, fawkes::OpenRaveEnvironmentPtr& env);
  virtual bool release_all_objects();

  virtual bool set_target_rel(float trans_x, float trans_y, float trans_z, bool is_extension=false);
  virtual bool set_target_straight(float trans_x, float trans_y, float trans_z);
  virtual bool set_target_quat(float trans_x, float trans_y, float trans_z,
                               float quat_w, float quat_x, float quat_y, float quat_z,
                               OpenRAVE::IkFilterOptions filter = OpenRAVE::IKFO_CheckEnvCollisions, bool no_offset = false);
  virtual bool set_target_axis_angle(float trans_x, float trans_y, float trans_z,
                                     float angle, float axisX, float axisY, float axisZ,
                                     OpenRAVE::IkFilterOptions filter = OpenRAVE::IKFO_CheckEnvCollisions, bool no_offset = false);
  virtual bool set_target_euler(euler_rotation_t type,
                                float trans_x, float trans_y, float trans_z,
                                float phi, float theta, float psi,
                                OpenRAVE::IkFilterOptions filter = OpenRAVE::IKFO_CheckEnvCollisions, bool no_offset = false);
  virtual bool set_target_object_position(float trans_x, float trans_y, float trans_z, float rot_x, OpenRAVE::IkFilterOptions filter = OpenRAVE::IKFO_CheckEnvCollisions);
  virtual bool set_target_ikparam(OpenRAVE::IkParameterization ik_param, OpenRAVE::IkFilterOptions filter = OpenRAVE::IKFO_CheckEnvCollisions);
  virtual void set_target_plannerparams(std::string& params);
  virtual void set_target_plannerparams(const char* params);
  virtual void set_target_raw(std::string& cmd);
  virtual void set_target_raw(const char* cmd);
  virtual void set_target_angles( std::vector<float>& angles );

  virtual void enable_ik_comparison(bool enable);

  virtual OpenRAVE::RobotBasePtr get_robot_ptr() const;
  virtual target_t get_target() const;
  virtual OpenRaveManipulatorPtr get_manipulator() const;
  virtual OpenRAVE::PlannerBase::PlannerParametersPtr get_planner_params() const;
  virtual std::vector< std::vector<OpenRAVE::dReal> >* get_trajectory() const;
  virtual std::vector< std::vector<float> >* get_trajectory_device() const;

  virtual bool display_planned_movements() const;

  virtual OpenRAVE::ModuleBasePtr get_basemanip() const;

 private:
  void init();
  void build_name_str();
  const char* name() const;

  bool set_target_transform(OpenRAVE::Vector& trans, OpenRAVE::Vector& rotQuat, OpenRAVE::IkFilterOptions filter, bool no_offset = false);
  bool set_target_euler(OpenRAVE::Vector& trans, std::vector<float>& rotations, OpenRAVE::IkFilterOptions filter, bool no_offset = false);
  OpenRAVE::IkParameterization get_5dof_ikparam(OpenRAVE::Transform& trans);
  bool solve_ik(OpenRAVE::IkFilterOptions filter);

  fawkes::Logger*                 __logger;

  std::string                           __name;
  std::string                           __name_str;
  OpenRAVE::RobotBasePtr                __robot;
  OpenRAVE::RobotBase::ManipulatorPtr   __arm;
  OpenRaveManipulatorPtr                __manip;
  target_t                              __target;


  OpenRAVE::ModuleBasePtr               __mod_basemanip;

  OpenRAVE::PlannerBase::PlannerParametersPtr   __planner_params;
  std::vector< std::vector<OpenRAVE::dReal> >*  __traj;

  float         __trans_offset_x;
  float         __trans_offset_y;
  float         __trans_offset_z;

  bool          __display_planned_movements;
  bool          __find_best_ik;
};

} // end of namespace fawkes

#endif
