
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

#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

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
  __viewer_enabled( 0 )
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
OpenRAVEEnvironment::enable_debug()
{
  RaveSetDebugLevel(Level_Debug);
}

/** Disable debugging messages of OpenRAVE */
void
OpenRAVEEnvironment::disable_debug()
{
  RaveSetDebugLevel(Level_Fatal);
}

/** Add a robot into the scene
 * @param robot RobotBasePtr of robot to add
 * @return 1 if succeeded, 0 if not able to add robot
 */
void
OpenRAVEEnvironment::add_robot(RobotBasePtr robot)
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
OpenRAVEEnvironment::add_robot(const std::string& filename)
{
  // load the robot
  RobotBasePtr robot = __env->ReadRobotXMLFile(filename);

  // if could not load robot file: Check file path, and test file itself for correct syntax and semantics
  // by loading it directly into openrave with "openrave robotfile.xml"
  if( !robot )
    {throw fawkes::IllegalArgumentException("OpenRAVE Environment: Robot could not be loaded. Check xml file/path.");}
  else if(__logger)
    {__logger->log_debug("OpenRAVE Environment", "Robot loaded.");}

  add_robot(robot);
}

/** Add a robot into the scene
 * @param robot pointer to OpenRAVERobot object of robot to add
 * @return 1 if succeeded, 0 if not able to add robot
 */
void
OpenRAVEEnvironment::add_robot(OpenRAVERobot* robot)
{
  add_robot(robot->get_robot_ptr());
}


/** Get EnvironmentBasePtr
 * @return EnvironmentBasePtr in use
 */
OpenRAVE::EnvironmentBasePtr
OpenRAVEEnvironment::get_env_ptr() const
{
  return __env;
}

/** Starts the  qt viewer in a separate thread.
 *  Use this mainly for debugging purposes, as it uses
 *  a lot of CPU/GPU resources.
 */
void
OpenRAVEEnvironment::start_viewer()
{
  try {
    boost::thread thviewer(boost::bind(SetViewer,__env,"qtcoin"));
  } catch( const openrave_exception &e) {
    if(__logger)
      {__logger->log_error("OpenRAVE Environment", "Could not load viewr. Ex:%s", e.what());}
    throw;
  }

  __viewer_enabled = true;
}

/** Autogenerate IKfast IK solver for robot
 * @param robot pointer to OpenRAVERobot object
 */
void
OpenRAVEEnvironment::load_IK_solver(OpenRAVERobot* robot)
{
  ProblemInstancePtr ikfast = RaveCreateProblem(__env,"ikfast");
  RobotBasePtr robotBase = robot->get_robot_ptr();
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
OpenRAVEEnvironment::run_planner(OpenRAVERobot* robot)
{
  bool success;

  // init planner
  success = __planner->InitPlan(robot->get_robot_ptr(),robot->get_planner_params());
  if(!success)
    {throw fawkes::Exception("OpenRAVE Environment: Planner: init failed");}
  else if(__logger)
    {__logger->log_debug("OpenRAVE Environment", "Planner: initialized");}

  // plan path
  boost::shared_ptr<Trajectory> traj(RaveCreateTrajectory(__env, robot->get_robot_ptr()->GetActiveDOF()));
  traj->Clear();
  success = __planner->PlanPath(traj);
  if(!success)
    {throw fawkes::Exception("OpenRAVE Environment: Planner: planning failed");}
  else if(__logger)
    {__logger->log_debug("OpenRAVE Environment", "Planner: path planned");}

  // re-timing the trajectory with cubic interpolation
  traj->CalcTrajTiming(robot->get_robot_ptr(),TrajectoryBase::CUBIC,true,true);

  // setting robots trajectory
  std::vector<TrajectoryBase::TPOINT> points = traj->GetPoints();
  std::vector< std::vector<float> >* trajRobot = robot->get_trajectory();
  trajRobot->clear();

  for(std::vector<TrajectoryBase::TPOINT>::iterator it = points.begin(); it!=points.end(); ++it) {
    trajRobot->push_back((*it).q);
  }

  // display motion in viewer
  if( __viewer_enabled )
    {robot->get_robot_ptr()->SetActiveMotion(traj);}
}


/** Add an object to the environment.
 * @param name name that should be given to that object
 * @param filename path to xml file of that object (KinBody)
 */
bool
OpenRAVEEnvironment::add_object(const std::string& name, const std::string& filename)
{
  try {
    KinBodyPtr kb = __env->ReadKinBodyXMLFile(filename);
    kb->SetName(name);
    __env->AddKinBody(kb);
  } catch(const OpenRAVE::openrave_exception &e) {
    if(__logger)
      __logger->log_warn("OpenRAVE Environment", "Could not add Object '%s'. Ex:%s", name.c_str(), e.what());
    return false;
  }

  return true;
}

/** Remove object from environment.
 * @param name name of the object
 */
bool
OpenRAVEEnvironment::delete_object(const std::string& name)
{
  try {
    KinBodyPtr kb = __env->GetKinBody(name);
    __env->Remove(kb);
  } catch(const OpenRAVE::openrave_exception &e) {
    if(__logger)
      __logger->log_warn("OpenRAVE Environment", "Could not delete Object '%s'. Ex:%s", name.c_str(), e.what());
    return false;
  }

  return true;
}

/** Rename object.
 * @param name current name of the object
 * @param new_name new name of the object
 */
bool
OpenRAVEEnvironment::rename_object(const std::string& name, const std::string& new_name)
{
  try {
    KinBodyPtr kb = __env->GetKinBody(name);
    kb->SetName(new_name);
  } catch(const OpenRAVE::openrave_exception &e) {
    if(__logger)
      __logger->log_warn("OpenRAVE Environment", "Could not rename Object '%s' to '%s'. Ex:%s", name.c_str(), new_name.c_str(), e.what());
    return false;
  }

  return true;
}

/** Move object in the environment.
 * Distances are given in meters
 * @param name name of the object
 * @param trans_x transition along x-axis
 * @param trans_y transition along y-axis
 * @param trans_z transition along z-axis
 * @param robot if given, move relatively to robot (in most simple cases robot is at position (0,0,0) anyway, so this has no effect)
 */
bool
OpenRAVEEnvironment::move_object(const std::string& name, float trans_x, float trans_y, float trans_z, OpenRAVERobot* robot)
{
  try {
    KinBodyPtr kb = __env->GetKinBody(name);

    Transform transform = kb->GetTransform();
    transform.trans = Vector(trans_x, trans_y, trans_z);

    if( robot ) {
      Transform robotTrans = robot->get_robot_ptr()->GetTransform();
      transform.trans += robotTrans.trans;
    }

    kb->SetTransform(transform);
  } catch(const OpenRAVE::openrave_exception &e) {
    if(__logger)
      __logger->log_warn("OpenRAVE Environment", "Could not move Object '%s'. Ex:%s", name.c_str(), e.what());
    return false;
  }

  return true;
}

/** Rotate object along its axis.
 * Rotation angles should be given in radians.
 * @param name name of the object
 * @param rot_x 1st rotation, along x-axis
 * @param rot_y 2nd rotation, along y-axis
 * @param rot_z 3rd rotation, along z-axis
 */
bool
OpenRAVEEnvironment::rotate_object(const std::string& name, float rot_x, float rot_y, float rot_z)
{
  try {
    KinBodyPtr kb = __env->GetKinBody(name);

    Vector q1 = quatFromAxisAngle(Vector(rot_x, 0.f, 0.f));
    Vector q2 = quatFromAxisAngle(Vector(0.f, rot_y, 0.f));
    Vector q3 = quatFromAxisAngle(Vector(0.f, 0.f, rot_z));

    Vector q12  = quatMultiply (q1, q2);
    Vector quat = quatMultiply (q12, q3);

    Transform transform = kb->GetTransform();
    transform.rot = quat;

    kb->SetTransform(transform);
  } catch(const OpenRAVE::openrave_exception &e) {
    if(__logger)
      __logger->log_warn("OpenRAVE Environment", "Could not rotate Object '%s'. Ex:%s", name.c_str(), e.what());
    return false;
  }

  return true;
}

} // end of namespace fawkes
