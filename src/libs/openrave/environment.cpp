
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
#include <core/exceptions/software.h>

#include <sstream>

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
  __logger( logger ),
  __viewerEnabled( 0 )
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
  __env = RaveCreateEnvironment();
  if(!__env)
    {throw fawkes::Exception("OpenRAVE Environment: Could not create environment. Error in OpenRAVE.");}
  else if (__logger)
    {__logger->log_debug("OpenRAVE Environment", "Environment created");}

  // create planner
  __planner = RaveCreatePlanner(__env,"birrt");
  if(!__planner)
    {throw fawkes::Exception("OpenRAVE Environment: Could not create planner. Error in OpenRAVE.");}
}

/** Destroy the environment */
void
OpenRAVEEnvironment::destroy()
{
  try {
    __env->Destroy();
    if(__logger)
      {__logger->log_debug("OpenRAVE Environment", "Environment destroyed");}
  } catch(const openrave_exception& e) {
    if(__logger)
      {__logger->log_warn("OpenRAVE Environment", "Could not destroy Environment. Ex:%s", e.what());}
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
void
OpenRAVEEnvironment::addRobot(RobotBasePtr robot)
{
  if(!__env->AddRobot(robot))
    {throw fawkes::Exception("OpenRAVE Environment: Could not add robot to environment. Error in OpenRAVE.");}
  else if(__logger)
    {__logger->log_debug("OpenRAVE Environment", "Robot added to environment.");}
}

/** Add a robot into the scene
 * @param filename path to robot's xml file
 * @return 1 if succeeded, 0 if not able to load file
 */
void
OpenRAVEEnvironment::addRobot(const std::string& filename)
{
  // load the robot
  RobotBasePtr robot = __env->ReadRobotXMLFile(filename);

  // if could not load robot file: Check file path, and test file itself for correct syntax and semantics
  // by loading it directly into openrave with "openrave robotfile.xml"
  if( !robot )
    {throw fawkes::IllegalArgumentException("OpenRAVE Environment: Robot could not be loaded. Check xml file/path.");}
  else if(__logger)
    {__logger->log_debug("OpenRAVE Environment", "Robot loaded.");}

  addRobot(robot);
}

/** Add a robot into the scene
 * @param robot pointer to OpenRAVERobot object of robot to add
 * @return 1 if succeeded, 0 if not able to add robot
 */
void
OpenRAVEEnvironment::addRobot(OpenRAVERobot* robot)
{
  addRobot(robot->getRobotPtr());
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
      {__logger->log_error("OpenRAVE Environment", "Could not load viewr. Ex:%s", e.what());}
    throw;
  }
}

/** Autogenerate IKfast IK solver for robot
 * @param robot pointer to OpenRAVERobot object
 */
void
OpenRAVEEnvironment::loadIKSolver(OpenRAVERobot* robot)
{
  ProblemInstancePtr ikfast = RaveCreateProblem(__env,"ikfast");
  RobotBasePtr robotBase = robot->getRobotPtr();
  __env->LoadProblem(ikfast,"");

  std::stringstream ssin,ssout;
  ssin << "LoadIKFastSolver " << robotBase->GetName() << " " << (int)IkParameterization::Type_Transform6D;
  // if necessary, add free inc for degrees of freedom
  //ssin << " " << 0.04f;
  if( !ikfast->SendCommand(ssout,ssin) )
    {throw fawkes::Exception("OpenRAVE Environment: Could not load ik solver");}
}

/** Plan collision-free path for current and target manipulator
 * configuration of a OpenRAVERobot robot.
 * @param robot pointer to OpenRAVERobot object of robot to use
 */
void
OpenRAVEEnvironment::runPlanner(OpenRAVERobot* robot)
{
  bool success;

  // init planner
  success = __planner->InitPlan(robot->getRobotPtr(),robot->getPlannerParams());
  if(!success)
    {throw fawkes::Exception("OpenRAVE Environment: Planner: init failed");}
  else if(__logger)
    {__logger->log_debug("OpenRAVE Environment", "Planner: initialized");}

  // plan path
  boost::shared_ptr<Trajectory> traj(RaveCreateTrajectory(__env, robot->getRobotPtr()->GetActiveDOF()));
  traj->Clear();
  success = __planner->PlanPath(traj);
  if(!success)
    {throw fawkes::Exception("OpenRAVE Environment: Planner: planning failed");}
  else if(__logger)
    {__logger->log_debug("OpenRAVE Environment", "Planner: path planned");}

  // re-timing the trajectory with cubic interpolation
  traj->CalcTrajTiming(robot->getRobotPtr(),TrajectoryBase::CUBIC,true,true);

  // setting robots trajectory
  std::vector<TrajectoryBase::TPOINT> points = traj->GetPoints();
  std::vector< std::vector<float> >* trajRobot = robot->getTrajectory();
  trajRobot->clear();

  for(std::vector<TrajectoryBase::TPOINT>::iterator it = points.begin(); it!=points.end(); ++it) {
    trajRobot->push_back((*it).q);
  }

  // display motion in viewer
  if( __viewerEnabled )
    {robot->getRobotPtr()->SetActiveMotion(traj);}
}


} // end of namespace fawkes
