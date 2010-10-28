
/***************************************************************************
 *  environment.cpp - Fawkes to OpenRAVE Environment
 *
 *  Created: Sun Sep 19 14:50:34 2010
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
#include "environment.h"
#include "robot.h"

#include <openrave-core.h>
#include <utils/logging/logger.h>

using namespace OpenRAVE;
namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif


void
SetViewer(OpenRAVE::EnvironmentBasePtr env, const std::string& viewername)
{
  ViewerBasePtr viewer = RaveCreateViewer(env, viewername);
  BOOST_ASSERT(!!viewer);

  // attach it to the environment:
  env->AttachViewer(viewer);

  // finally you call the viewer's infinite loop (this is why you need a separate thread):
  viewer->main(/*showGUI=*/true);
}


/** @class OpenRAVEEnvironment <openrave/environment.h>
 * Class handling interaction with the OpenRAVE::EnvironmentBase class.
 * This class loads a scene and adds robots/objects etc to it. All calculations
 * in OpenRAVE (IK, planning, etc) are done based on the current scene.
 * @author Bahram Maleki-Fard
 */

/** Constructor
 * @param logger pointer to fawkes logger
 */
OpenRAVEEnvironment::OpenRAVEEnvironment(fawkes::Logger* logger) :
  __name( "OpenRAVE Environment" ),
  __logger( logger )
{
}

/** Destructor. */
OpenRAVEEnvironment::~OpenRAVEEnvironment()
{
  this->destroy();
}

/** Create and lock the environment */
void
OpenRAVEEnvironment::create()
{
  // create environment
  try {
    __env = RaveCreateEnvironment();
    if(__logger)
      __logger->log_debug(__name, "Environment created");
  } catch(const openrave_exception &e) {
    if(__logger)
      __logger->log_warn(__name, "Could not create Environment. Ex:%s", e.what());
    throw;
  }

  // create planner
  try {
    __planner = RaveCreatePlanner(__env,"birrt");
  } catch(const openrave_exception &e) {
    if(__logger)
      __logger->log_warn(__name, "Could not create Planner. Ex:%s", e.what());
    throw;
  }
}

/** Destroy the environment */
void
OpenRAVEEnvironment::destroy()
{
  try {
    __env->Destroy();
    if(__logger)
      __logger->log_debug(__name, "Environment destroyed");
  } catch(const openrave_exception& e) {
    __logger->log_warn(__name, "Could not destroy Environment. Ex:%s", e.what());
  }
}

/** Lock the environment to prevent changes */
void
OpenRAVEEnvironment::lock()
{
  EnvironmentMutex::scoped_lock lock(__env->GetMutex());
}

/** Enable debugging messages of OpenRAVE */
void
OpenRAVEEnvironment::enableDebug()
{
  RaveSetDebugLevel(Level_Debug);
}

/** Disable debugging messages of OpenRAVE */
void
OpenRAVEEnvironment::disableDebug()
{
  RaveSetDebugLevel(Level_Fatal);
}

/** Add a robot into the scene
 * @param robot RobotBasePtr of robot to add
 * @return 1 if succeeded, 0 if not able to add robot
 */
bool
OpenRAVEEnvironment::addRobot(RobotBasePtr robot)
{
    return __env->AddRobot(robot);
}

/** Add a robot into the scene
 * @param filename path to robot's xml file
 * @return 1 if succeeded, 0 if not able to load file
 */
bool
OpenRAVEEnvironment::addRobot(const std::string& filename)
{
  // load the robot
  RobotBasePtr robot;
  try {
    robot = __env->ReadRobotXMLFile(filename);
  } catch(const openrave_exception &e) {
    if(__logger)
      __logger->log_error(__name, "Robot could not be loaded. Ex:%s", e.what());
    return 0;
  }

  if( !robot ) {
    // could not load robot file. Check file path, and test file itself for correct syntax and semantics
    // by loading it directly into openrave with "openrave robotfile.xml"
    if(__logger)
      __logger->log_error(__name, "Robot could not be loaded.");
    return 0;
  } else {
    return addRobot(robot);
  }
}

/** Add a robot into the scene
 * @param robot pointer to OpenRAVERobot object of robot to add
 * @return 1 if succeeded, 0 if not able to add robot
 */
bool
OpenRAVEEnvironment::addRobot(OpenRAVERobot* robot)
{
    return addRobot(robot->getRobotPtr());
}


/** Get EnvironmentBasePtr
 * @return EnvironmentBasePtr in use
 */
OpenRAVE::EnvironmentBasePtr
OpenRAVEEnvironment::getEnvPtr() const
{
  return __env;
}

/** Starts the  qt viewer in a separate thread.
 *  Use this mainly for debugging purposes, as it uses
 *  a lot of CPU/GPU resources.
 */
void
OpenRAVEEnvironment::startViewer()
{
  try {
    boost::thread thviewer(boost::bind(SetViewer,__env,"qtcoin"));
  } catch( const openrave_exception &e) {
    if(__logger)
      __logger->log_error(__name, "Could not load viewr. Ex:%s", e.what());
    throw;
  }
}

/** Plan collision-free path for current and target manipulator
 * configuration of a OpenRAVERobot robot.
 * @param robot pointer to OpenRAVERobot object of robot to use
 */
bool
OpenRAVEEnvironment::runPlanner(OpenRAVERobot* robot)
{
  // init planner
  if( !__planner->InitPlan(robot->getRobotPtr(),robot->getPlannerParams()) ) {
    if(__logger)
      __logger->log_error(__name, "Planner: init failed");
    return 0;
  }

  // plan path
  boost::shared_ptr<Trajectory> traj(RaveCreateTrajectory(__env, robot->getRobotPtr()->GetActiveDOF()));
  traj->Clear();
  if( !__planner->PlanPath(traj) ) {
    if(__logger)
    __logger->log_error(__name, "Planner: plan failed");
    return 0;
  }

  // re-timing the trajectory with cubic interpolation
  traj->CalcTrajTiming(robot->getRobotPtr(),TrajectoryBase::CUBIC,true,true);

  // setting robots trajectory
  std::vector<TrajectoryBase::TPOINT> points = traj->GetPoints();
  std::vector< std::vector<float> >* trajRobot = robot->getTrajectory();
  trajRobot->clear();

  for(std::vector<TrajectoryBase::TPOINT>::iterator it = points.begin(); it!=points.end(); ++it) {
    trajRobot->push_back((*it).q);
  }

  return 1;
}


} // end of namespace fawkes
