
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
 *@param filename path to robot's xml file
 */
bool
OpenRAVERobot::load(const std::string& filename, fawkes::OpenRAVEEnvironment* env)
{
  // load the robot
  // TODO: implementing without usage of 'environment'
  try {
    __robot = env->getEnvPtr()->ReadRobotXMLFile(filename);
  } catch(const openrave_exception &e) {
    if(__logger)
      __logger->log_error("OpenRAVE Robot", "Robot could not be loaded. Ex:%s", e.what());
    return 0;
  }

  if(!__robot) {
    if(__logger)
      __logger->log_error("OpenRAVE Robot", "Robot could not be loaded.");
    return 0;
  }

  return 1;
}

/** Set robot ready for usage.
 *  Here: Set active DOFs and create plannerParameters.
 * Only successful after added to environment */
bool
OpenRAVERobot::setReady()
{
  __name = __robot->GetName();
  __robot->SetActiveManipulator(__robot->GetManipulators().at(0)->GetName());
  __arm = __robot->GetActiveManipulator();
  __robot->SetActiveDOFs(__arm->GetArmIndices());

  // create planner parameters
  try {
    PlannerBase::PlannerParametersPtr params(new PlannerBase::PlannerParameters());
    __plannerParams = params;
    __plannerParams->_nMaxIterations = 4000; // max iterations before failure
    __plannerParams->SetRobotActiveJoints(__robot); // set planning configuration space to current active dofs
    __plannerParams->vgoalconfig.resize(__robot->GetActiveDOF());
  } catch(const openrave_exception &e) {
    if(__logger)
      __logger->log_error("OpenRAVE Robot", "Could not create PlannerParameters. Ex:%s", e.what());
    return 0;
  }

  return 1;
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

/** Check IK solvability for target Transform. If solvable,
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


std::vector<float>*
OpenRAVERobot::getTargetAngles()
{
  return &__anglesTarget;
}

/** Updates planner parameters and return pointer to it
 * @return PlannerParametersPtr or robot's planner params
 */
OpenRAVE::PlannerBase::PlannerParametersPtr
OpenRAVERobot::getPlannerParams() const
{
  __plannerParams->vgoalconfig = __manipGoal->getAngles();
  __plannerParams->vinitialconfig = __manip->getAngles();

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
} // end of namespace fawkes