
/***************************************************************************
 *  robot.cpp - Fawkes to OpenRAVE Robot Handler
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

#include "robot.h"
#include "manipulator.h"
#include "environment.h"

#include <openrave-core.h>
#include <logging/logger.h>
#include <core/exceptions/software.h>

using namespace OpenRAVE;
namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class OpenRaveRobot <plugins/openrave/robot.h>
* Class handling interaction with the OpenRAVE::RobotBase class.
* This class mainly handles robot specific tasks, like setting a
* target, looking for IK solutions and handling planning parameters
* for the robot.
* @author Bahram Maleki-Fard
*/

/** Constructor
 * @param logger pointer to fawkes logger
 */
OpenRaveRobot::OpenRaveRobot(fawkes::Logger* logger) :
  __logger( logger ),
  __name( "" ),
  __manip( 0 ),
  __find_best_ik( 1 )
{
  init();
}
/** Constructor
 * @param filename path to robot's xml file
 * @param env pointer to OpenRaveEnvironment object
 * @param logger pointer to fawkes logger
 */
OpenRaveRobot::OpenRaveRobot(const std::string& filename, fawkes::OpenRaveEnvironmentPtr& env, fawkes::Logger* logger) :
  __logger( logger ),
  __name( "" ),
  __manip( 0 ),
  __find_best_ik( 1 )
{
  init();
  this->load(filename, env);
}

/** Copy Constructor.
 * @param src The OpenRaveRobot to clone
 * @param new_env Pointer to the new OpenRaveEnvironment. We need this to set __robot
 *  to the correct robot in the new OpenRAVE-environment.
 */
OpenRaveRobot::OpenRaveRobot(const OpenRaveRobot& src, const fawkes::OpenRaveEnvironmentPtr& new_env) :
  __logger( src.__logger ),
  __name( src.__name ),
  __find_best_ik( src.__find_best_ik )
{
  build_name_str();
  __traj = new std::vector< std::vector<dReal> >();

  __trans_offset_x = src.__trans_offset_x;
  __trans_offset_y = src.__trans_offset_y;
  __trans_offset_z = src.__trans_offset_z;

  // Get correct robot from environment.
  EnvironmentMutex::scoped_lock lock(new_env->get_env_ptr()->GetMutex());
  std::string name = src.get_robot_ptr()->GetName();
  __robot = new_env->get_env_ptr()->GetRobot( name );

  if(!__robot)
    throw fawkes::IllegalArgumentException("%s: Robot '%s' could not be loaded. Check name.", this->name(), name.c_str());

  build_name_str();
  if(__logger)
    __logger->log_debug(this->name(), "Robot loaded.");

  // Initialize robot
  set_ready();

  // Set the same manipulator "active" as it was in previous environment
  // "set_ready()" just takes the first manipulator it finds
  {
    EnvironmentMutex::scoped_lock lock(src.get_robot_ptr()->GetEnv()->GetMutex());
    __arm = __robot->SetActiveManipulator( src.get_robot_ptr()->GetActiveManipulator()->GetName() );
  }
  __robot->SetActiveDOFs(__arm->GetArmIndices());

  __manip = src.get_manipulator()->copy();
  __target.manip = __manip->copy();
  __display_planned_movements = false;

  if(__logger)
    __logger->log_debug(this->name(), "Robot '%s' cloned.", name.c_str());
}

/** Destructor */
OpenRaveRobot::~OpenRaveRobot()
{
  __target.manip = NULL;

  //unload everything related to this robot from environment
  try {
    EnvironmentBasePtr env = __robot->GetEnv();
    EnvironmentMutex::scoped_lock lock(env->GetMutex());
    env->Remove(__mod_basemanip);
    env->Remove(__robot);
    if(__logger)
      __logger->log_warn(name(), "Robot unloaded from environment");
  } catch(const openrave_exception &e) {
    if(__logger)
      __logger->log_warn(name(), "Could not unload robot properly from environment. Ex:%s", e.what());
  }
}

/** Build name string to use in logging messages.
 * Nothing important, but helpful for debugging etc.
 */
void
OpenRaveRobot::build_name_str()
{
  std::stringstream n;
  n << "OpenRaveRobot" << "[";
  if( __robot )
    n << RaveGetEnvironmentId(__robot->GetEnv()) << ":";
  n << __name << "]";
  __name_str = n.str();
}

/** Get the name string to use in logging messages etc. */
const char*
OpenRaveRobot::name() const
{
  return __name_str.c_str();
}

/** Inittialize object attributes */
void
OpenRaveRobot::init()
{
  __traj = new std::vector< std::vector<dReal> >();

  __trans_offset_x = 0.f;
  __trans_offset_y = 0.f;
  __trans_offset_z = 0.f;

  build_name_str();
}


/** Load robot from xml file
 * @param filename path to robot's xml file
 * @param env pointer to OpenRaveEnvironment object
 */
void
OpenRaveRobot::load(const std::string& filename, fawkes::OpenRaveEnvironmentPtr& env)
{
  EnvironmentMutex::scoped_lock lock(env->get_env_ptr()->GetMutex());

  // TODO: implementing without usage of 'environment'
  // openrave_exception handling is done in OpenRAVE (see environment-core.h)
  __robot = env->get_env_ptr()->ReadRobotXMLFile(filename);

  if(!__robot)
    throw fawkes::IllegalArgumentException("%s: Robot could not be loaded. Check xml file/path '%s'.", name(), filename.c_str());

  __name = __robot->GetName();
  build_name_str();

  if(__logger)
    __logger->log_debug(name(), "Robot loaded.");
}

/** Set robot ready for usage.
 * Here: Set active DOFs and create plannerParameters.
 * CAUTION: Only successful after added to environment. Otherwise no active DOF will be recognized. */
void
OpenRaveRobot::set_ready()
{
  if(!__robot)
    throw fawkes::Exception("%s: Robot not loaded properly yet.", name());

  EnvironmentMutex::scoped_lock lock(__robot->GetEnv()->GetMutex());

  __robot->SetActiveManipulator(__robot->GetManipulators().at(0)->GetName());
  __arm = __robot->GetActiveManipulator();
  __robot->SetActiveDOFs(__arm->GetArmIndices());

  if(__robot->GetActiveDOF() == 0)
    throw fawkes::Exception("%s: Robot not added to environment yet. Need to do that first, otherwise planner will fail.", name());

  // create planner parameters
  try {
    PlannerBase::PlannerParametersPtr params(new PlannerBase::PlannerParameters());
    __planner_params = params;
    __planner_params->_nMaxIterations = 4000; // max iterations before failure
    __planner_params->SetRobotActiveJoints(__robot); // set planning configuration space to current active dofs
    __planner_params->vgoalconfig.resize(__robot->GetActiveDOF());
  } catch(const openrave_exception &e) {
    throw fawkes::Exception("%s: Could not create PlannerParameters. Ex:%s", name(), e.what());
  }

  // create and load BaseManipulation module
  try {
    __mod_basemanip = RaveCreateModule(__robot->GetEnv(), "basemanipulation");
    __robot->GetEnv()->AddModule( __mod_basemanip, __robot->GetName());
  } catch(const openrave_exception &e) {
    throw fawkes::Exception("%s: Cannot load BaseManipulation Module. Ex:%s", name(), e.what());
  }

  if(__logger)
    __logger->log_debug(name(), "Robot ready.");
}

/** Directly set transition offset between coordinate systems
 * of real device and OpenRAVE model.
 * @param trans_x transition offset on x-axis
 * @param trans_y transition offset on y-axis
 * @param trans_z transition offset on z-axis
 */
 void
 OpenRaveRobot::set_offset(float trans_x, float trans_y, float trans_z)
 {
  __trans_offset_x = trans_x;
  __trans_offset_y = trans_y;
  __trans_offset_z = trans_z;
 }

/** Calculate transition offset between coordinate systems
 * of real device and OpenRAVE model.
 * Sets model's angles to current device's angles (from __manip),
 * and compares transitions.
 * @param device_trans_x transition on x-axis (real device)
 * @param device_trans_y transition on y-axis (real device)
 * @param device_trans_z transition on z-axis (real device)
 */
void
OpenRaveRobot::calibrate(float device_trans_x, float device_trans_y, float device_trans_z)
{
  EnvironmentMutex::scoped_lock lock(__robot->GetEnv()->GetMutex());
  // get device's current angles, and set them for OpenRAVE model
  std::vector<dReal> angles;
  __manip->get_angles(angles);
  __robot->SetActiveDOFValues(angles);

  // get model's current transition and compare
  __arm = __robot->GetActiveManipulator();
  Transform trans = __arm->GetEndEffectorTransform();
  __trans_offset_x = trans.trans[0] - device_trans_x;
  __trans_offset_y = trans.trans[1] - device_trans_y;
  __trans_offset_z = trans.trans[2] - device_trans_z;
}

/** Set pointer to OpenRaveManipulator object.
 *  Make sure this is called AFTER all manipulator settings have
 *  been set (assures that __target.manip has the same settings).
 * @param manip pointer to OpenRaveManipulator object
 * @param display_movements true, if movements should be displayed in viewer.
 *  Better be "false" if want to sync OpenRAVE models with device
 */
void
OpenRaveRobot::set_manipulator(fawkes::OpenRaveManipulatorPtr& manip, bool display_movements)
{
  __manip = manip;
  __target.manip = __manip->copy();

  __display_planned_movements = display_movements;
}

/** Update motor values from OpenRAVE model.
 * Can be used to sync real device with OpenRAVE model*/
void
OpenRaveRobot::update_manipulator()
{
  EnvironmentMutex::scoped_lock lock(__robot->GetEnv()->GetMutex());
  std::vector<dReal> angles;
  __robot->GetActiveDOFValues(angles);
  __manip->set_angles(angles);
}

/** Update/Set OpenRAVE motor angles */
void
OpenRaveRobot::update_model()
{
  EnvironmentMutex::scoped_lock lock(__robot->GetEnv()->GetMutex());
  std::vector<dReal> angles;
  __manip->get_angles(angles);
  __robot->SetActiveDOFValues(angles);
}

/** Getter for __display_planned_movements.
 * @return return value
 */
bool
OpenRaveRobot::display_planned_movements() const
{
  return __display_planned_movements;
}

/** Activate/Deactive IK comparison.
 * When activated, we don't just take the first returned IK solution, but
 * compare them all to find the best, i.e. the one that is "closest" to our
 * current configuration.
 * @param enable Sets the state of the comparison. Enabled by default.
 */
void
OpenRaveRobot::enable_ik_comparison(bool enable)
{
  __find_best_ik = enable;
}

/** Set target, given relative transition.
 * This is the prefered method to set a target for straight manipulator movement.
 * @param trans_x x-transition
 * @param trans_y y-transition
 * @param trans_z z-transition
 * @param is_extension true, if base coordination system lies in arm extension
 * @return true if solvable, false otherwise
 */
bool
OpenRaveRobot::set_target_rel(float trans_x, float trans_y, float trans_z, bool is_extension)
{
  if( is_extension ) {
    __target.type = TARGET_RELATIVE_EXT;
  } else {
    __target.type = TARGET_RELATIVE;
  }
  __target.x = trans_x;
  __target.y = trans_y;
  __target.z = trans_z;

  // Not sure how to check IK solvability yet. Would be nice to have this
  // checked before planning a path.
  __target.solvable = true;

  return __target.solvable;
}

/** Set target for a straight movement, given transition.
 * This is the a wrapper for "set_target_rel", to be able to call for a
 * straight arm movement by giving non-relative transition.
 * @param trans_x x-transition
 * @param trans_y y-transition
 * @param trans_z z-transition
 * @return true if solvable, false otherwise
 */
bool
OpenRaveRobot::set_target_straight(float trans_x, float trans_y, float trans_z)
{
  Transform trans;
  {
    EnvironmentMutex::scoped_lock lock(__robot->GetEnv()->GetMutex());
    __arm = __robot->GetActiveManipulator();
    trans = __arm->GetEndEffectorTransform();
  }

  return set_target_rel( trans_x - trans.trans[0],
                         trans_y - trans.trans[1],
                         trans_z - trans.trans[2]);
}

/** Set target, given transition, and rotation as quaternion.
 * @param trans_x x-transition
 * @param trans_y y-transition
 * @param trans_z z-transition
 * @param quat_w quaternion skalar
 * @param quat_x quaternion 1st value
 * @param quat_y quaternion 2nd value
 * @param quat_z quaternion 3rd value
 * @param filter IK filter options (see OpenRAVE doc for details)
 * @param no_offset if true, do not include manipulator offset (default: false)
 * @return true if solvable, false otherwise
 */
bool
OpenRaveRobot::set_target_quat(float trans_x, float trans_y, float trans_z,
                               float quat_w, float quat_x, float quat_y, float quat_z,
                               IkFilterOptions filter, bool no_offset)
{
  Vector trans(trans_x, trans_y, trans_z);
  Vector   rot(quat_w, quat_x, quat_y, quat_z);

  return set_target_transform(trans, rot, filter, no_offset);
}

/** Set target, given transition, and rotation as axis-angle.
 * @param trans_x x-transition
 * @param trans_y y-transition
 * @param trans_z z-transition
 * @param angle axis-angle angle
 * @param axisX axis-angle x-axis value
 * @param axisY axis-angle y-axis value
 * @param axisZ axis-angle z-axis value
 * @param filter IK filter options (see OpenRAVE doc for details)
 * @param no_offset if true, do not include manipulator offset (default: false)
 * @return true if solvable, false otherwise
 */
bool
OpenRaveRobot::set_target_axis_angle(float trans_x, float trans_y, float trans_z,
                                     float angle, float axisX, float axisY, float axisZ,
                                     IkFilterOptions filter, bool no_offset)
{
  Vector trans(trans_x, trans_y, trans_z);
  Vector aa(angle, axisX, axisY, axisZ);
  Vector rot = quatFromAxisAngle(aa);

  return set_target_transform(trans, rot, filter, no_offset);
}

/** Set target, given transition, and Euler-rotation.
 * @param type Euler-rotation type (ZXZ, ZYZ, ...)
 * @param trans_x x-transition
 * @param trans_y y-transition
 * @param trans_z z-transition
 * @param phi 1st rotation
 * @param theta 2nd rotation
 * @param psi 3rd rotation
 * @param filter IK filter options (see OpenRAVE doc for details)
 * @param no_offset if true, do not include manipulator offset (default: false)
 * @return true if solvable, false otherwise
 */
bool
OpenRaveRobot::set_target_euler(euler_rotation_t type,
                                float trans_x, float trans_y, float trans_z,
                                float phi, float theta, float psi,
                                OpenRAVE::IkFilterOptions filter, bool no_offset)
{
  Vector trans(trans_x, trans_y, trans_z);
  std::vector<float> rot(9, 0.f); //rotations vector

  switch(type) {
    case (EULER_ZXZ) :
        if(__logger)
          __logger->log_debug(name(), "Target EULER_ZXZ: %f %f %f %f %f %f", trans_x, trans_y, trans_z, phi, theta, psi);
        rot.at(2) = phi;   //1st row, 3rd value; rotation on z-axis
        rot.at(3) = theta; //2nd row, 1st value; rotation on x-axis
        rot.at(8) = psi;   //3rd row, 3rd value; rotation on z-axis
        break;

    case (EULER_ZYZ) :
        if(__logger)
          __logger->log_debug(name(), "Target EULER_ZYZ:", "%f %f %f %f %f %f", trans_x, trans_y, trans_z, phi, theta, psi);
        rot.at(2) = phi;   //1st row, 3rd value; rotation on z-axis
        rot.at(4) = theta; //2nd row, 2nd value; rotation on y-axis
        rot.at(8) = psi;   //3rd row, 3rd value; rotation on z-axis
        break;

    case (EULER_ZYX) :
        if(__logger)
          __logger->log_debug(name(), "Target EULER_ZYX:", "%f %f %f %f %f %f", trans_x, trans_y, trans_z, phi, theta, psi);
        rot.at(2) = phi;   //1st row, 3rd value; rotation on z-axis
        rot.at(4) = theta; //2nd row, 2nd value; rotation on y-axis
        rot.at(6) = psi;   //3rd row, 1st value; rotation on x-axis
        break;

    default :
        __target.type = TARGET_NONE;
        __target.solvable = false;
        return false;
  }

  return set_target_euler(trans, rot, filter, no_offset);
}

/** Set target by giving position of an object.
 * Currently the object should be cylindric, and stand upright. It may
 * also be rotated on its x-axis, but that rotation needs to be given in an argument
 * to calculate correct position for end-effector. This is only temporary until
 * proper grasp planning for 5DOF in OpenRAVE is provided.
 * @param trans_x x-transition of object
 * @param trans_y y-transition of object
 * @param trans_z z-transition of object
 * @param rot_x rotation of object on x-axis (radians) (default: 0.f, i.e. upright)
 * @param filter IK filter options (see OpenRAVE doc for details)
 * @return true if solvable, false otherwise
 */
bool
OpenRaveRobot::set_target_object_position(float trans_x, float trans_y, float trans_z, float rot_x, IkFilterOptions filter)
{
  // This is about 2 times faster than using setTargetEuler each time, especially when it comes
  // to the while loop (whole loop: ~56ms vs ~99ms)

  // release all attached/grabbed bodys
  {
    EnvironmentMutex::scoped_lock lock(__robot->GetEnv()->GetMutex());
    __robot->ReleaseAllGrabbed();
  }

  // quaternion defining consecutiv rotations on axis
  float alpha = atan2(trans_y - __trans_offset_y, trans_x - __trans_offset_x);      //angle to rotate left/right when manipulator points to +x
  Vector quat_y = quatFromAxisAngle(Vector(0.f, M_PI/2, 0.f));           //1st, rotate down -> manipulator points to +x
  Vector quat_x = quatFromAxisAngle(Vector(-alpha, 0.f, 0.f));           //2nd, rotate left/right -> manipulator points to object
  Vector quat_z = quatFromAxisAngle(Vector(0.f, 0.f, rot_x));             //last, rotate wrist -> manipulator ready to grab

  Vector quat_xY =  quatMultiply (quat_y, quat_x);
  Vector quat_xYZ = quatMultiply (quat_xY, quat_z);

  Vector trans(trans_x, trans_y, trans_z);

  if( set_target_transform(trans, quat_xYZ, filter, true) )
    return true;

  //try varying 2nd rotation (quat_y) until a valid IK is found. Max angle: 45° (~0.79 rad)
  Vector quatPosY=quatFromAxisAngle(Vector(0.f, 0.017f, 0.f));          //rotate up for 1°
  Vector quatNegY=quatFromAxisAngle(Vector(0.f, -0.017f, 0.f));         //rotate down for 1°

  Vector quatPos(quat_xY);       //starting position, after first 2 rotations
  Vector quatNeg(quat_xY);

  unsigned int count = 0;
  bool foundIK = false;

  while( (!foundIK) && (count <= 45)) {
    count++;

    quatPos = quatMultiply(quatPos, quatPosY);  //move up ~1°
    quatNeg = quatMultiply(quatNeg, quatNegY);  //move down ~1°

    quat_xYZ = quatMultiply(quatPos, quat_z);     //apply wrist rotation
    foundIK = set_target_transform(trans, quat_xYZ, filter, true);
    if( !foundIK ) {
      quat_xYZ = quatMultiply(quatNeg, quat_z);
      foundIK = set_target_transform(trans, quat_xYZ, filter, true);
    }
  }

  return foundIK;
}

/** Set target by giving IkParameterizaion of target.
 * OpenRAVE::IkParameterization is the desired type to be calculated with
 * by OpenRAVE. Each oter type (i.e. Transform) is implicitly transformed
 * to an IkParameterization before continuing to check for Ik solution and
 * planning, i.e. by the BaseManipulation module.
 * @param ik_param the OpenRAVE::IkParameterization of the target
 * @param filter IK filter options (see OpenRAVE doc for details)
 * @return true if solvable, false otherwise
 */
bool
OpenRaveRobot::set_target_ikparam(IkParameterization ik_param, IkFilterOptions filter)
{
  EnvironmentMutex::scoped_lock lock(__robot->GetEnv()->GetMutex());
  __arm = __robot->GetActiveManipulator();

  __target.ikparam = ik_param;
  __target.type = TARGET_IKPARAM;
  solve_ik(filter);

  return __target.solvable;
}

/** Set additional planner parameters.
 * BaseManipulation module accepts many arguments that can be passed.
 * Planner parameters can be important to plan a path according to ones
 * needs, e.g. set deviations, optimizer iterations, etc.
 * Do not mistake it with the single argument "plannerparams" of BaseManipulation.
 * @param params complete string of additional arguments.
 */
void
OpenRaveRobot::set_target_plannerparams(std::string& params)
{
  __target.plannerparams = params;
}

/** Set additional planner parameters.
 * @param params complete string of additional arguments.
 */
void
OpenRaveRobot::set_target_plannerparams(const char* params)
{
  __target.plannerparams = params;
}

/** Set raw command for BaseManipulation module.
 * BaseManipulation module accepts many arguments that can be passed.
 * Basic commands are covered by the other set_target_ methods. In case something
 * is not covered, or you want to send a custom command, use this method.
 * Remember that plannerparams set by "set_target_plannerparams" are still added
 * to the planner, so make sure you don't send duplicate entries both in plannerparams
 * and in the raw command string.
 * @param cmd complete command string.
 */
void
OpenRaveRobot::set_target_raw(std::string& cmd)
{
  __target.raw_cmd = cmd;
}

/** Set raw command for BaseManipulation module.
 * @param cmd complete command string.
 */
void
OpenRaveRobot::set_target_raw(const char* cmd)
{
  __target.raw_cmd = cmd;
}

/** Set target angles directly.
 * @param angles vector with angle values
 */
void
OpenRaveRobot::set_target_angles( std::vector<float>& angles )
{
  __target.manip->set_angles(angles);
  __target.type = TARGET_JOINTS;
  __target.solvable = true; //no IK check done though!
}




/* ################### getters ##################*/
/** Returns RobotBasePtr for uses in other classes.
 * @return RobotBasePtr of current robot
 */
OpenRAVE::RobotBasePtr
OpenRaveRobot::get_robot_ptr() const
{
  return __robot;
}

/** Get target.
 * @return target struct
 */
target_t
OpenRaveRobot::get_target() const
{
  return __target;
}

/** Get manipulator.
 * @return pointer to currentl used OpenRaveManipulator
 */
OpenRaveManipulatorPtr
OpenRaveRobot::get_manipulator() const
{
  return __manip;
}

/** Updates planner parameters and return pointer to it
 * @return PlannerParametersPtr or robot's planner params
 */
OpenRAVE::PlannerBase::PlannerParametersPtr
OpenRaveRobot::get_planner_params() const
{
  EnvironmentMutex::scoped_lock lock(__robot->GetEnv()->GetMutex());
  // set planning configuration space to current active dofs
  __planner_params->SetRobotActiveJoints(__robot);
  __planner_params->vgoalconfig.resize(__robot->GetActiveDOF());

  __manip->get_angles(__planner_params->vinitialconfig);
  __target.manip->get_angles(__planner_params->vgoalconfig);

  __robot->SetActiveDOFValues(__planner_params->vinitialconfig);

  return __planner_params;
}

/** Return pointer to trajectory of motion from
 * __manip to __target.manip with OpenRAVE-model angle format
 * @return pointer to trajectory
 */
std::vector< std::vector<dReal> >*
OpenRaveRobot::get_trajectory() const
{
  return __traj;
}

/** Return pointer to trajectory of motion from
 * __manip to __target.manip with device angle format
 * @return pointer to trajectory
 */
std::vector< std::vector<float> >*
OpenRaveRobot::get_trajectory_device() const
{
  std::vector< std::vector<float> >* traj = new std::vector< std::vector<float> >();

  std::vector<float> v;

  for(unsigned int i=0; i<__traj->size(); i++) {
    __manip->angles_or_to_device(__traj->at(i), v);
    traj->push_back(v);
  }

  return traj;
}

/** Return BaseManipulation Module-Pointer.
 * @return ModuleBasePtr
 */
OpenRAVE::ModuleBasePtr
OpenRaveRobot::get_basemanip() const
{
  return __mod_basemanip;
}


/* ###### attach / release kinbodys ###### */
/** Attach a kinbody to the robot.
 * @param object KinbodyPtr of object to be attached
 * @param manip_name name of the manipulator to attach the object to.
 *  If non given, the currently active manipulator is taken.
 * @return true if successful
 */
bool
OpenRaveRobot::attach_object(OpenRAVE::KinBodyPtr object, const char* manip_name)
{
  EnvironmentMutex::scoped_lock lock(__robot->GetEnv()->GetMutex());

  bool success = false;
  try{
    if( manip_name ) {
      // try attaching to given manipulator
      RobotBase::ManipulatorPtr manip = __robot->SetActiveManipulator(manip_name);
      if( !manip ) {
        if(__logger)
          __logger->log_warn(name(), "Could not attach Object, could not get manipulator '%s'", manip_name);
        return false;

      } else {
        success = __robot->Grab(object, manip->GetEndEffector());
      }

    } else {
      // use currently active manipulator
      success = __robot->Grab(object);
    }
  } catch(const OpenRAVE::openrave_exception &e) {
    if(__logger)
      __logger->log_warn(name(), "Could not attach Object. Ex:%s", e.what());
    return false;
  }

  return success;
}
/** Attach a kinbody to the robot.
 * @param name name of the object
 * @param env pointer to OpenRaveEnvironment object
 * @param manip_name name of the manipulator to attach the object to
 * @return true if successful
 */
bool
OpenRaveRobot::attach_object(const char* name, fawkes::OpenRaveEnvironmentPtr& env, const char* manip_name)
{
  OpenRAVE::KinBodyPtr body;
  {
    EnvironmentMutex::scoped_lock lock(env->get_env_ptr()->GetMutex());
    body = env->get_env_ptr()->GetKinBody(name);
  }

  return attach_object(body, manip_name);
}

/** Release a kinbody from the robot.
 * @param object KinbodyPtr of object to be released
 * @return true if successful
 */
bool
OpenRaveRobot::release_object(OpenRAVE::KinBodyPtr object)
{
  try{
    EnvironmentMutex::scoped_lock lock(__robot->GetEnv()->GetMutex());
    __robot->Release(object);
  } catch(const OpenRAVE::openrave_exception &e) {
    if(__logger)
      __logger->log_warn(name(), "Could not release Object. Ex:%s", e.what());
    return false;
  }

  return true;
}
/** Release a kinbody from the robot.
 * @param name name of the object
 * @param env pointer to OpenRaveEnvironment object
 * @return true if successful
 */
bool
OpenRaveRobot::release_object(const std::string& name, fawkes::OpenRaveEnvironmentPtr& env)
{
  OpenRAVE::KinBodyPtr body;
  {
    EnvironmentMutex::scoped_lock lock(env->get_env_ptr()->GetMutex());
    body = env->get_env_ptr()->GetKinBody(name);
  }

  return release_object(body);
}

/** Release all grabbed kinbodys from the robot.
 * @return true if successful
 */
bool
OpenRaveRobot::release_all_objects()
{
  try{
    EnvironmentMutex::scoped_lock lock(__robot->GetEnv()->GetMutex());
    __robot->ReleaseAllGrabbed();
  } catch(const OpenRAVE::openrave_exception &e) {
    if(__logger)
      __logger->log_warn(name(), "Could not release all objects. Ex:%s", e.what());
    return false;
  }

  return true;
}




/* ########################################
   ###------------- private ------------###
   ########################################*/

/** Set target, given transformation (transition, and rotation as quaternion).
 * Check IK solvability for target Transform. If solvable,
 * then set target angles to manipulator configuration __target.manip
 * @param trans transformation vector
 * @param rotQuat rotation vector; a quaternion
 * @param no_offset if true, do not include manipulator offset (default: false)
 * @return true if solvable, false otherwise
 */
bool
OpenRaveRobot::set_target_transform(Vector& trans, OpenRAVE::Vector& rotQuat, IkFilterOptions filter, bool no_offset)
{
  Transform target;
  target.trans = trans;
  target.rot = rotQuat;

  if( !no_offset ) {
    target.trans[0] += __trans_offset_x;
    target.trans[1] += __trans_offset_y;
    target.trans[2] += __trans_offset_z;
  }

  __target.type = TARGET_TRANSFORM;
  __target.x  = target.trans[0];
  __target.y  = target.trans[1];
  __target.z  = target.trans[2];
  __target.qw = target.rot[0];
  __target.qx = target.rot[1];
  __target.qy = target.rot[2];
  __target.qz = target.rot[3];

  // check for supported IK types
  EnvironmentMutex::scoped_lock lock(__robot->GetEnv()->GetMutex());
  __arm = __robot->GetActiveManipulator();
  if( __arm->GetIkSolver()->Supports(IKP_Transform6D) ) {
    if(__logger)
      __logger->log_debug(name(), "6D suppport for arm %s", __arm->GetName().c_str());
    // arm supports 6D ik. Perfect!
    __target.ikparam = IkParameterization(target);
    solve_ik(filter);

  } else if( __arm->GetIkSolver()->Supports(IKP_TranslationDirection5D) ) {
    if(__logger)
      __logger->log_debug(name(), "5D suppport");
    // arm has only 5 DOF.
    __target.ikparam = get_5dof_ikparam(target);
    __target.solvable = set_target_ikparam(__target.ikparam, filter);

  } else {
    if(__logger)
      __logger->log_debug(name(), "No IK suppport");
    //other IK types not supported yet
    __target.solvable = false;
  }

  return __target.solvable;
}

/** Set target, given 3 consecutive axis rotations.
 * Axis rotations are given as 1 vector representing a 3x3 matrix,
 * (left to right, top to bottom) where each row represents
 * one rotation over one axis (axis-angle notation).
 * See public setTargetEuler methods to get a better understanding.
 *
 * Check IK solvability for target Transform. If solvable,
 * then set target angles to manipulator configuration __target.manip
 * @param rotations 3x3 matrix given as one row.
 * @param filter IK filter options (see OpenRAVE doc for details)
 * @param no_offset if true, do not include manipulator offset (default: false)
 * @return true if solvable, false otherwise
 */
bool
OpenRaveRobot::set_target_euler(OpenRAVE::Vector& trans, std::vector<float>& rotations, OpenRAVE::IkFilterOptions filter, bool no_offset)
{
  if( rotations.size() != 9 ) {
    __target.type = TARGET_NONE;
    __target.solvable = false;

    if(__logger)
      __logger->log_error(name(), "Bad size of rotations vector. Is %i, expected 9", rotations.size());
    return false;
  }

  Vector r1(rotations.at(0), rotations.at(1), rotations.at(2));
  Vector r2(rotations.at(3), rotations.at(4), rotations.at(5));
  Vector r3(rotations.at(6), rotations.at(7), rotations.at(8));

  if(__logger) {
    __logger->log_debug(name(), "TEST Rot1: %f %f %f", r1[0], r1[1], r1[2]);
    __logger->log_debug(name(), "TEST Rot2: %f %f %f", r2[0], r2[1], r2[2]);
    __logger->log_debug(name(), "TEST Rot3: %f %f %f", r3[0], r3[1], r3[2]);
  }

  Vector q1 = quatFromAxisAngle(r1);
  Vector q2 = quatFromAxisAngle(r2);
  Vector q3 = quatFromAxisAngle(r3);

  Vector q12  = quatMultiply (q1, q2);
  Vector quat = quatMultiply (q12, q3);

  return set_target_transform(trans, quat, filter, no_offset);
}

/** Get IkParameterization for a 5DOF arm given a 6D Transform.
 * @param trans The 6D OpenRAVE::Transform
 * @return the calculated 5DOF IkParameterization
 */
OpenRAVE::IkParameterization
OpenRaveRobot::get_5dof_ikparam(OpenRAVE::Transform& trans)
{
  /* The initial pose (that means NOT all joints=0, but the manipulator's coordinate-system
     matching the world-coordinate-system) of an arm in OpenRAVE has its gripper pointing to the z-axis.
     Imagine a tube between the grippers. That tube lies on the y-axis.
     For 5DOF-IK one needs another manipulator definition, that has it's z-axis lying on that
     'tube', i.e. it needs to be lying between the fingers. That is achieved by rotating the
     coordinate-system first by +-90° around z-axis, then +90° on the rotated x-axis.
  */

  // get direction vector for TranslationDirection5D
  /* Rotate Vector(0, +-1, 0) by target.rot. First need to figure out which of "+-"
     Now if the first rotation on z-axis was +90°, we need a (0,-1,0) direction vector.
     If it was -90°, we need (0, 1, 0). So just take the inverse of the first rotation
     and apply it to (1,0,0)
  */
  EnvironmentMutex::scoped_lock lock(__robot->GetEnv()->GetMutex());
  Vector dir(1,0,0);
  {
    RobotBasePtr tmp_robot = __robot;
    RobotBase::RobotStateSaver saver(tmp_robot); // save the state, do not modifiy currently active robot!

    //reset robot joints
    std::vector<dReal> zero_joints(tmp_robot->GetActiveDOF(), (dReal)0.0);
    tmp_robot->SetActiveDOFValues(zero_joints);

    // revert the rotations for the 5DOF manipulator specifition. See long comment above.
    // First rotate back -90° on x-axis (revert 2nd rotation)
    Transform cur_pos = __arm->GetEndEffectorTransform();
    Vector v1 = quatFromAxisAngle(Vector(-M_PI/2, 0, 0));
    v1 = quatMultiply(cur_pos.rot, v1);

    // Now get the inverse of 1st rotation and get our (0, +-1, 0) direction
    v1 = quatInverse(v1);
    TransformMatrix mat = matrixFromQuat(v1);
    dir = mat.rotate(dir);
  }  // robot state is restored

  // now rotate direction by target
  TransformMatrix mat = matrixFromQuat(trans.rot);
  dir = mat.rotate(dir);

  IkParameterization ikparam = __arm->GetIkParameterization(IKP_TranslationDirection5D);
  ikparam.SetTranslationDirection5D(RAY(trans.trans, dir));

  return ikparam;
}

/** Find IK solution that is closest to current configuration.
 * This method checks and updates the internal __target variable.
 * @return true if solvable, false otherwise.
 */
bool
OpenRaveRobot::solve_ik(IkFilterOptions filter)
{
  if( !__find_best_ik ) {
    std::vector<dReal> solution;
    __target.solvable = __arm->FindIKSolution(__target.ikparam,solution,filter);
    __target.manip->set_angles(solution);

  } else {
    std::vector< std::vector<dReal> > solutions;

    // get all IK solutions
    __target.solvable = __arm->FindIKSolutions(__target.ikparam,solutions,filter);
    if(!__target.solvable)
      return false;

    // pick closest solution to current configuration
    std::vector< std::vector<dReal> >::iterator sol;
    std::vector<dReal> cur;
    std::vector<dReal> diff;
    float dist = 100.f;
    __arm->GetArmDOFValues(cur);

    for( sol=solutions.begin(); sol!=solutions.end(); ++sol ) {
      diff = cur;
      __robot->SubtractActiveDOFValues(diff, *sol);

      float sol_dist = 0.f;
      for( unsigned int i=0; i<diff.size(); ++i ) {
        sol_dist += fabs(diff[i]);
        // use cur+diff instead of sol, to have better angles
        // for circular joints. Otherwise planner might have problems
        (*sol)[i] = cur[i] - diff[i];
      }

      if( sol_dist < dist ) {
        // found a solution that is closer
        dist = sol_dist;
        __target.manip->set_angles(*sol);
      }
    }
  }

  return __target.solvable;
}

} // end of namespace fawkes
