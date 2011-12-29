
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
  __manip( 0 )
{
  init();
}
/** Constructor
 * @param filename path to robot's xml file
 * @param env pointer to OpenRaveEnvironment object
 * @param logger pointer to fawkes logger
 */
OpenRaveRobot::OpenRaveRobot(const std::string& filename, fawkes::OpenRaveEnvironment* env, fawkes::Logger* logger) :
  __logger( logger ),
  __name( "" ),
  __manip( 0 )
{
  init();
  this->load(filename, env);
}

/** Destructor */
OpenRaveRobot::~OpenRaveRobot()
{
}

/** Inittialize object attributes */
void
OpenRaveRobot::init()
{
  __traj = new std::vector< std::vector<dReal> >();

  __trans_offset_x = 0.f;
  __trans_offset_y = 0.f;
  __trans_offset_z = 0.f;
}


/** Load robot from xml file
 * @param filename path to robot's xml file
 * @param env pointer to OpenRaveEnvironment object
 */
void
OpenRaveRobot::load(const std::string& filename, fawkes::OpenRaveEnvironment* env)
{
  // TODO: implementing without usage of 'environment'
  // openrave_exception handling is done in OpenRAVE (see environment-core.h)
  __robot = env->get_env_ptr()->ReadRobotXMLFile(filename);

  if(!__robot)
    {throw fawkes::IllegalArgumentException("OpenRAVE Robot: Robot could not be loaded. Check xml file/path.");}
  else if(__logger)
    {__logger->log_debug("OpenRAVE Robot", "Robot loaded.");}
}

/** Set robot ready for usage.
 * Here: Set active DOFs and create plannerParameters.
 * CAUTION: Only successful after added to environment. Otherwise no active DOF will be recognized. */
void
OpenRaveRobot::set_ready()
{
  if(!__robot)
    {throw fawkes::Exception("OpenRAVE Robot: Robot not loaded properly yet.");}

  __name = __robot->GetName();
  __robot->SetActiveManipulator(__robot->GetManipulators().at(0)->GetName());
  __arm = __robot->GetActiveManipulator();
  __robot->SetActiveDOFs(__arm->GetArmIndices());

  if(__robot->GetActiveDOF() == 0)
    {throw fawkes::Exception("OpenRAVE Robot: Robot not added to environment yet. Need to do that first, otherwise planner will fail.");}

  // create planner parameters
  try {
    PlannerBase::PlannerParametersPtr params(new PlannerBase::PlannerParameters());
    __planner_params = params;
    __planner_params->_nMaxIterations = 4000; // max iterations before failure
    __planner_params->SetRobotActiveJoints(__robot); // set planning configuration space to current active dofs
    __planner_params->vgoalconfig.resize(__robot->GetActiveDOF());
  } catch(const openrave_exception &e) {
    throw fawkes::Exception("OpenRAVE Robot: Could not create PlannerParameters. Ex:%s", e.what());
  }

  // create and load BaseManipulation module
  try {
    __mod_basemanip = RaveCreateModule(__robot->GetEnv(), "basemanipulation");
    __robot->GetEnv()->AddModule( __mod_basemanip, __robot->GetName());
  } catch(const openrave_exception &e) {
    throw fawkes::Exception("OpenRAVE Robot: Cannot load BaseManipulation Module. Ex:%s", e.what());
  }

  if(__logger)
    {__logger->log_debug("OpenRAVE Robot", "Robot ready.");}
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
  // get device's current angles, and set them for OpenRAVE model
  std::vector<dReal> angles;
  __manip->get_angles(angles);
  __robot->SetActiveDOFValues(angles);

  // get model's current transition and compare
  Transform trans = __arm->GetEndEffectorTransform();
  __trans_offset_x = trans.trans[0] - device_trans_x;
  __trans_offset_y = trans.trans[1] - device_trans_y;
  __trans_offset_z = trans.trans[2] - device_trans_z;
}

/** Set pointer to OpenRaveManipulator object.
 *  Make sure this is called AFTER all manipulator settings have
 *  been set (assures that __manip_goal has the same settings).
 * @param manip pointer to OpenRaveManipulator object
 * @param display_movements true, if movements should be displayed in viewer.
 *  Better be "false" if want to sync OpenRAVE models with device
 */
void
OpenRaveRobot::set_manipulator(fawkes::OpenRaveManipulator* manip, bool display_movements)
{
  __manip = manip;
  __manip_goal = new OpenRaveManipulator(*__manip);

  __display_planned_movements = display_movements;
}

/** Update motor values from OpenRAVE model.
 * Can be used to sync real device with OpenRAVE model*/
void
OpenRaveRobot::update_manipulator()
{
  std::vector<dReal> angles;
  __robot->GetActiveDOFValues(angles);
  __manip->set_angles(angles);
}

/** Update/Set OpenRAVE motor angles */
void
OpenRaveRobot::update_model()
{
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

/** Set target, given transition, and rotation as quaternion.
 * @param trans_x x-transition
 * @param trans_y y-transition
 * @param trans_z z-transition
 * @param quat_w quaternion skalar
 * @param quat_x quaternion 1st value
 * @param quat_y quaternion 2nd value
 * @param quat_z quaternion 3rd value
 * @param no_offset if true, do not include manipulator offset (default: false)
 * @return true if solvable, false otherwise
 */
bool
OpenRaveRobot::set_target_quat(float trans_x, float trans_y, float trans_z, float quat_w, float quat_x, float quat_y, float quat_z, bool no_offset)
{
  Vector trans(trans_x, trans_y, trans_z);
  Vector   rot(quat_w, quat_x, quat_y, quat_z);

  return set_target_transform(trans, rot, no_offset);
}

/** Set target, given transition, and rotation as axis-angle.
 * @param trans_x x-transition
 * @param trans_y y-transition
 * @param trans_z z-transition
 * @param angle axis-angle angle
 * @param axisX axis-angle x-axis value
 * @param axisY axis-angle y-axis value
 * @param axisZ axis-angle z-axis value
 * @param no_offset if true, do not include manipulator offset (default: false)
 * @return true if solvable, false otherwise
 */
bool
OpenRaveRobot::set_target_axis_angle(float trans_x, float trans_y, float trans_z, float angle, float axisX, float axisY, float axisZ, bool no_offset)
{
  Vector trans(trans_x, trans_y, trans_z);
  Vector aa(angle, axisX, axisY, axisZ);
  Vector rot = quatFromAxisAngle(aa);

  return set_target_transform(trans, rot, no_offset);
}

/** Set target, given transition, and Euler-rotation.
 * @param type Euler-rotation type (ZXZ, ZYZ, ...)
 * @param trans_x x-transition
 * @param trans_y y-transition
 * @param trans_z z-transition
 * @param phi 1st rotation
 * @param theta 2nd rotation
 * @param psi 3rd rotation
 * @param no_offset if true, do not include manipulator offset (default: false)
 * @return true if solvable, false otherwise
 */
bool
OpenRaveRobot::set_target_euler(euler_rotation_t type, float trans_x, float trans_y, float trans_z, float phi, float theta, float psi, bool no_offset)
{
  Vector trans(trans_x, trans_y, trans_z);
  std::vector<float> rot(9, 0.f); //rotations vector

  switch(type) {
    case (EULER_ZXZ) :
        __logger->log_debug("TEST ZXZ", "%f %f %f %f %f %f", trans_x, trans_y, trans_z, phi, theta, psi);
        rot.at(2) = phi;   //1st row, 3rd value; rotation on z-axis
        rot.at(3) = theta; //2nd row, 1st value; rotation on x-axis
        rot.at(8) = psi;   //3rd row, 3rd value; rotation on z-axis
        break;

    case (EULER_ZYZ) :
        __logger->log_debug("TEST ZYZ", "%f %f %f %f %f %f", trans_x, trans_y, trans_z, phi, theta, psi);
        rot.at(2) = phi;   //1st row, 3rd value; rotation on z-axis
        rot.at(4) = theta; //2nd row, 2nd value; rotation on y-axis
        rot.at(8) = psi;   //3rd row, 3rd value; rotation on z-axis
        break;

    case (EULER_ZYX) :
        rot.at(2) = phi;   //1st row, 3rd value; rotation on z-axis
        rot.at(4) = theta; //2nd row, 2nd value; rotation on y-axis
        rot.at(6) = psi;   //3rd row, 1st value; rotation on x-axis
        break;

    default :
        return false;
  }

  return set_target_euler(trans, rot, no_offset);
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
 * @return true if solvable, false otherwise
 */
bool
OpenRaveRobot::set_target_object_position(float trans_x, float trans_y, float trans_z, float rot_x)
{
  // This is about 2 times faster than using setTargetEuler each time, especially when it comes
  // to the while loop (whole loop: ~56ms vs ~99ms)

  // release all attached/grabbed bodys
  __robot->ReleaseAllGrabbed();

  // quaternion defining consecutiv rotations on axis
  float alpha = atan2(trans_y - __trans_offset_y, trans_x - __trans_offset_x);      //angle to rotate left/right when manipulator points to +x
  Vector quat_y = quatFromAxisAngle(Vector(0.f, M_PI/2, 0.f));           //1st, rotate down -> manipulator points to +x
  Vector quat_x = quatFromAxisAngle(Vector(-alpha, 0.f, 0.f));           //2nd, rotate left/right -> manipulator points to object
  Vector quat_z = quatFromAxisAngle(Vector(0.f, 0.f, rot_x));             //last, rotate wrist -> manipulator ready to grab

  Vector quat_xY =  quatMultiply (quat_y, quat_x);
  Vector quat_xYZ = quatMultiply (quat_xY, quat_z);

  Vector trans(trans_x, trans_y, trans_z);

  if( set_target_transform(trans, quat_xYZ, true) )
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
    foundIK = set_target_transform(trans, quat_xYZ, true);
    if( !foundIK ) {
      quat_xYZ = quatMultiply(quatNeg, quat_z);
      foundIK = set_target_transform(trans, quat_xYZ, true);
    }
  }

  return foundIK;
}


// just temporary! no IK check etc involved
/** Set target angles directly.
 * @param angles vector with angle values
 */
void
OpenRaveRobot::set_target_angles( std::vector<float>& angles )
{
  __manip_goal->set_angles(angles);
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

// not needed
/** Get target angles.
 * @param to vector that should be filled with angles
 */
void
OpenRaveRobot::get_target_angles(std::vector<OpenRAVE::dReal>& to)
{
  to = __angles_target;
}

/** Updates planner parameters and return pointer to it
 * @return PlannerParametersPtr or robot's planner params
 */
OpenRAVE::PlannerBase::PlannerParametersPtr
OpenRaveRobot::get_planner_params() const
{
  __manip_goal->get_angles(__planner_params->vgoalconfig);
  __manip->get_angles(__planner_params->vinitialconfig);

  __robot->SetActiveDOFValues(__planner_params->vinitialconfig);

  return __planner_params;
}

/** Return pointer to trajectory of motion from
 * __manip to __manip_goal with OpenRAVE-model angle format
 * @return pointer to trajectory
 */
std::vector< std::vector<dReal> >*
OpenRaveRobot::get_trajectory() const
{
  return __traj;
}

/** Return pointer to trajectory of motion from
 * __manip to __manip_goal with device angle format
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
 * @return true if successful
 */
bool
OpenRaveRobot::attach_object(OpenRAVE::KinBodyPtr object)
{
  bool success = false;
  try{
    success = __robot->Grab(object);
  } catch(const OpenRAVE::openrave_exception &e) {
    if(__logger)
      __logger->log_warn("OpenRAVE Robot", "Could not attach Object. Ex:%s", e.what());
    return false;
  }

  return success;
}
/** Attach a kinbody to the robot.
 * @param name name of the object
 * @param env pointer to OpenRaveEnvironment object
 * @return true if successful
 */
bool
OpenRaveRobot::attach_object(const std::string& name, fawkes::OpenRaveEnvironment* env)
{
  OpenRAVE::KinBodyPtr body = env->get_env_ptr()->GetKinBody(name);

  return attach_object(body);
}

/** Release a kinbody from the robot.
 * @param object KinbodyPtr of object to be released
 * @return true if successful
 */
bool
OpenRaveRobot::release_object(OpenRAVE::KinBodyPtr object)
{
  try{
    __robot->Release(object);
  } catch(const OpenRAVE::openrave_exception &e) {
    if(__logger)
      __logger->log_warn("OpenRAVE Robot", "Could not release Object. Ex:%s", e.what());
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
OpenRaveRobot::release_object(const std::string& name, fawkes::OpenRaveEnvironment* env)
{
  OpenRAVE::KinBodyPtr body = env->get_env_ptr()->GetKinBody(name);

  return release_object(body);
}

/** Release all grabbed kinbodys from the robot.
 * @return true if successful
 */
bool
OpenRaveRobot::release_all_objects()
{
  try{
    __robot->ReleaseAllGrabbed();
  } catch(const OpenRAVE::openrave_exception &e) {
    if(__logger)
      __logger->log_warn("OpenRAVE Robot", "Could not release all objects. Ex:%s", e.what());
    return false;
  }

  return true;
}




/* ########################################
   ###------------- private ------------###
   ########################################*/

/** Set target, given transformation (transition, and rotation as quaternion).
 * Check IK solvability for target Transform. If solvable,
 * then set target angles to manipulator configuration __manip_goal
 * @param trans transformation vector
 * @param rotQuat rotation vector; a quaternion
 * @param no_offset if true, do not include manipulator offset (default: false)
 * @return true if solvable, false otherwise
 */
bool
OpenRaveRobot::set_target_transform(OpenRAVE::Vector& trans, OpenRAVE::Vector& rotQuat, bool no_offset)
{
  Transform target;
  target.trans = trans;
  target.rot = rotQuat;

  if( !no_offset ) {
    target.trans[0] += __trans_offset_x;
    target.trans[1] += __trans_offset_y;
    target.trans[2] += __trans_offset_z;
  }

  bool success = __arm->FindIKSolution(IkParameterization(target),__angles_target,true);
  __manip_goal->set_angles(__angles_target);

  return success;
}

/** Set target, given 3 consecutive axis rotations.
 * Axis rotations are given as 1 vector representing a 3x3 matrix,
 * (left to right, top to bottom) where each row represents
 * one rotation over one axis (axis-angle notation).
 * See public setTargetEuler methods to get a better understanding.
 *
 * Check IK solvability for target Transform. If solvable,
 * then set target angles to manipulator configuration __manip_goal
 * @param rotations 3x3 matrix given as one row.
 * @param no_offset if true, do not include manipulator offset (default: false)
 * @return true if solvable, false otherwise
 */
bool
OpenRaveRobot::set_target_euler(OpenRAVE::Vector& trans, std::vector<float>& rotations, bool no_offset)
{
  if( rotations.size() != 9 ) {
    if(__logger)
      {__logger->log_error("OpenRAVE Robot", "Bad size of rotations vector. Is %i, expected 9", rotations.size());}
    return false;
  }

  Vector r1(rotations.at(0), rotations.at(1), rotations.at(2));
  Vector r2(rotations.at(3), rotations.at(4), rotations.at(5));
  Vector r3(rotations.at(6), rotations.at(7), rotations.at(8));

  __logger->log_debug("TEST", "Rot1: %f %f %f", r1[0], r1[1], r1[2]);
  __logger->log_debug("TEST", "Rot2: %f %f %f", r2[0], r2[1], r2[2]);
  __logger->log_debug("TEST", "Rot3: %f %f %f", r3[0], r3[1], r3[2]);

  Vector q1 = quatFromAxisAngle(r1);
  Vector q2 = quatFromAxisAngle(r2);
  Vector q3 = quatFromAxisAngle(r3);

  Vector q12  = quatMultiply (q1, q2);
  Vector quat = quatMultiply (q12, q3);

  return set_target_transform(trans, quat, no_offset);
}

} // end of namespace fawkes
