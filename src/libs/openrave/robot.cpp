
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
#include <utils/logging/logger.h>
#include <core/exceptions/software.h>

using namespace OpenRAVE;
namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** Constructor */
OpenRAVERobot::OpenRAVERobot(fawkes::Logger* logger) :
  __logger( logger ),
  __name( "" ),
  __manip( 0 )
{
  init();
}
OpenRAVERobot::OpenRAVERobot(const std::string& filename, fawkes::OpenRAVEEnvironment* env, fawkes::Logger* logger) :
  __logger( logger ),
  __name( "" ),
  __manip( 0 )
{
  init();
  this->load(filename, env);
}

/** Destructor */
OpenRAVERobot::~OpenRAVERobot()
{
}

/** Inittialize object attributes */
void
OpenRAVERobot::init()
{
  __traj = new std::vector< std::vector<float> >();
}


/** Load robot from xml file
 * @param filename path to robot's xml file
 * @param env pointer to OpenRAVEEnvironment object
 */
void
OpenRAVERobot::load(const std::string& filename, fawkes::OpenRAVEEnvironment* env)
{
  // TODO: implementing without usage of 'environment'
  // openrave_exception handling is done in OpenRAVE (see environment-core.h)
  __robot = env->getEnvPtr()->ReadRobotXMLFile(filename);

  if(!__robot)
    {throw fawkes::IllegalArgumentException("OpenRAVE Robot: Robot could not be loaded. Check xml file/path.");}
  else if(__logger)
    {__logger->log_debug("OpenRAVE Robot", "Robot loaded.");}
}

/** Set robot ready for usage.
 * Here: Set active DOFs and create plannerParameters.
 * CAUTION: Only successful after added to environment. Otherwise no active DOF will be recognized. */
void
OpenRAVERobot::setReady()
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
    __plannerParams = params;
    __plannerParams->_nMaxIterations = 4000; // max iterations before failure
    __plannerParams->SetRobotActiveJoints(__robot); // set planning configuration space to current active dofs
    __plannerParams->vgoalconfig.resize(__robot->GetActiveDOF());
  } catch(const openrave_exception &e) {
    throw fawkes::Exception("OpenRAVE Robot: Could not create PlannerParameters. Ex:%s", e.what());
  }

  if(__logger)
    {__logger->log_debug("OpenRAVE Robot", "Robot ready.");}
}

/** Set pointer to OpenRAVEManipulator object.
 *  Make sure this is called AFTER all manipulator settings have
 *  been set (assures that __manipGoal has the same settings) .*/
void
OpenRAVERobot::setManipulator(fawkes::OpenRAVEManipulator* manip)
{
  __manip = manip;
  __manipGoal = new OpenRAVEManipulator(*__manip);
}

/** Update motor values from OpenRAVE model.
 * TODO: why would we need this??? */
void
OpenRAVERobot::updateManipulator()
{
  std::vector<float> angles;
  __robot->GetDOFValues(angles);
  __manip->setAngles(angles);
}


/** Set target, given transition, and rotation as quaternion.
 * @param transX x-transition
 * @param transY y-transition
 * @param transZ z-transition
 * @param quatW quaternion skalar
 * @param quatX quaternion 1st value
 * @param quatY quaternion 2nd value
 * @param quatZ quaternion 3rd value
 * @return true if solvable, false otherwise
 */
bool
OpenRAVERobot::setTargetQuat(float& transX, float& transY, float& transZ, float& quatW, float& quatX, float& quatY, float& quatZ)
{
  Vector trans(transX, transY, transZ);
  Vector   rot(quatW, quatX, quatY, quatZ);

  return setTargetTransform(trans, rot);
}

/** Set target, given transition, and rotation as axis-angle.
 * @param transX x-transition
 * @param transY y-transition
 * @param transZ z-transition
 * @param angle axis-angle angle
 * @param axisX axis-angle x-axis value
 * @param axisY axis-angle y-axis value
 * @param axisZ axis-angle z-axis value
 * @return true if solvable, false otherwise
 */
bool
OpenRAVERobot::setTargetAxisAngle(float& transX, float& transY, float& transZ, float& angle, float& axisX, float& axisY, float& axisZ)
{
  Vector trans(transX, transY, transZ);
  Vector aa(angle, axisX, axisY, axisZ);
  Vector rot = quatFromAxisAngle(aa);

  return setTargetTransform(trans, rot);
}

/** Set target, given transition, and rotation as ZXZ Euler-rotation.
 * @param transX x-transition
 * @param transY y-transition
 * @param transZ z-transition
 * @param phi 1st rotaion (on z-axis)
 * @param theta 2nd rotaion (on x-axis)
 * @param psi 3rd rotaion (on z-axis)
 * @return true if solvable, false otherwise
 */
bool
OpenRAVERobot::setTargetEulerZXZ (float& transX, float& transY, float& transZ, float& phi, float& theta, float& psi)
{
  Vector trans(transX, transY, transZ);

  std::vector<float> rot(9, 0.f); //rotations vector
  rot.at(2) = phi;   //1st row, 3rd value; rotation on z-axis
  rot.at(3) = theta; //2nd row, 1st value; rotation on x-axis
  rot.at(8) = psi;   //3rd row, 3rd value; rotation on z-axis

  return setTargetEuler(trans, rot);
}

/** Set target, given transition, and rotation as ZYZ Euler-rotation.
 * @param transX x-transition
 * @param transY y-transition
 * @param transZ z-transition
 * @param phi 1st rotaion (on z-axis)
 * @param theta 2nd rotaion (on y-axis)
 * @param psi 3rd rotaion (on z-axis)
 * @return true if solvable, false otherwise
 */
bool
OpenRAVERobot::setTargetEulerZYZ (float& transX, float& transY, float& transZ, float& phi, float& theta, float& psi)
{
  Vector trans(transX, transY, transZ);

  std::vector<float> rot(9, 0.f); //rotations vector
  rot.at(2) = phi;   //1st row, 3rd value; rotation on z-axis
  rot.at(4) = theta; //2nd row, 2nd value; rotation on y-axis
  rot.at(8) = psi;   //3rd row, 3rd value; rotation on z-axis

  return setTargetEuler(trans, rot);
}

/** Set target, given transition, and rotation as ZYX Euler-rotation.
 * @param transX x-transition
 * @param transY y-transition
 * @param transZ z-transition
 * @param phi 1st rotaion (on z-axis)
 * @param theta 2nd rotaion (on y-axis)
 * @param psi 3rd rotaion (on x-axis)
 * @return true if solvable, false otherwise
 */
bool
OpenRAVERobot::setTargetEulerZYX (float& transX, float& transY, float& transZ, float& phi, float& theta, float& psi)
{
  Vector trans(transX, transY, transZ);

  std::vector<float> rot(9, 0.f); //rotations vector
  rot.at(2) = phi;   //1st row, 3rd value; rotation on z-axis
  rot.at(4) = theta; //2nd row, 2nd value; rotation on y-axis
  rot.at(6) = psi;   //3rd row, 1st value; rotation on x-axis

  return setTargetEuler(trans, rot);
}

// just temporary! no IK check etc involved
void
OpenRAVERobot::setTargetAngles( std::vector<float>& angles )
{
  __manipGoal->setAngles(angles);
}





/* ################### getters ##################*/
/** Returns RobotBasePtr for uses in other classes.
 * @return RobotBasePtr of current robot
 */
OpenRAVE::RobotBasePtr
OpenRAVERobot::getRobotPtr() const
{
  return __robot;
}

// not needed
void
OpenRAVERobot::getTargetAngles(std::vector<float>& to)
{
  to = __anglesTarget;
}

/** Updates planner parameters and return pointer to it
 * @return PlannerParametersPtr or robot's planner params
 */
OpenRAVE::PlannerBase::PlannerParametersPtr
OpenRAVERobot::getPlannerParams() const
{
  __manipGoal->getAngles(__plannerParams->vgoalconfig);
  __manip->getAngles(__plannerParams->vinitialconfig);

  return __plannerParams;
}

/** Return pointer to trajectory of motion from
 * __manip to __manipGoal with OpenRAVE-model angle format
 * @return pointer to trajectory
 */
std::vector< std::vector<float> >*
OpenRAVERobot::getTrajectory() const
{
  return __traj;
}

/** Return pointer to trajectory of motion from
 * __manip to __manipGoal with device angle format
 * @return pointer to trajectory
 */
std::vector< std::vector<float> >*
OpenRAVERobot::getTrajectoryDevice() const
{
  std::vector< std::vector<float> >* traj = new std::vector< std::vector<float> >();

  for(unsigned int i=0; i<__traj->size(); i++) {
    traj->push_back(__manip->anglesOR2Device(__traj->at(i)));
  }

  return traj;
}


/* ########################################
   ###------------- private ------------###
   ########################################*/

/** Set target, given transformation (transition, and rotation as quaternion).
 * Check IK solvability for target Transform. If solvable,
 * then set target angles to manipulator configuration __manipGoal
 * @param trans transformation vector
 * @param rotQuat rotation vector; a quaternion
 * @return true if solvable, false otherwise
 */
bool
OpenRAVERobot::setTargetTransform(OpenRAVE::Vector& trans, OpenRAVE::Vector& rotQuat)
{
  Transform target;
  target.trans = trans;
  target.rot = rotQuat;

  bool success = __arm->FindIKSolution(IkParameterization(target),__anglesTarget,true);
  __manipGoal->setAngles(__anglesTarget);

  return success;
}

/** Set target, given 3 consecutive axis rotations.
 * Axis rotations are given as 1 vector representing a 3x3 matrix,
 * (left to right, top to bottom) where each row represents
 * one rotation over one axis (axis-angle notation).
 * See public setTargetEuler methods to get a better understanding.
 *
 * Check IK solvability for target Transform. If solvable,
 * then set target angles to manipulator configuration __manipGoal
 * @param rotations 3x3 matrix given as one row.
 * @return true if solvable, false otherwise
 */
bool
OpenRAVERobot::setTargetEuler(OpenRAVE::Vector& trans, std::vector<float>& rotations)
{
  if( rotations.size() != 9 ) {
    if(__logger)
      {__logger->log_error("OpenRAVE Robot", "Bad size of rotations vector. Is %i, expected 9", rotations.size());}
    return false;
  }

  Vector r1(rotations.at(0), rotations.at(1), rotations.at(2));
  Vector r2(rotations.at(3), rotations.at(4), rotations.at(5));
  Vector r3(rotations.at(6), rotations.at(7), rotations.at(8));

  Vector q1 = quatFromAxisAngle(r1);
  Vector q2 = quatFromAxisAngle(r2);
  Vector q3 = quatFromAxisAngle(r3);

  Vector q12  = quatMultiply (q1, q2);
  Vector quat = quatMultiply (q12, q3);

  return setTargetTransform(trans, quat);
}

} // end of namespace fawkes