
/***************************************************************************
 *  openrave_thread.cpp - OpenRAVE Thread
 *
 *  Created: Fri Feb 25 15:08:00 2011
 *  Copyright  2011  Bahram Maleki-Fard
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

#include "openrave_thread.h"

#include "environment.h"
#include "robot.h"
#include "manipulator.h"

#include <openrave-core.h>
#include <core/exceptions/software.h>

using namespace fawkes;

/** @class OpenRaveThread "openrave_thread.h"
 * OpenRAVE Thread.
 * This thread maintains an active connection to OpenRAVE and provides an
 * aspect to access OpenRAVE to make it convenient for other threads to use
 * OpenRAVE.
 *
 * @author Bahram Maleki-Fard
 */

/** Constructor. */
OpenRaveThread::OpenRaveThread()
  : Thread("OpenRaveThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT),
    AspectProviderAspect(&__or_aspectIniFin),
  __or_aspectIniFin( this ),
  __OR_env( 0 ),
  __OR_robot( 0 )

{
}


/** Destructor. */
OpenRaveThread::~OpenRaveThread()
{
}


void
OpenRaveThread::init()
{
  try {
    OpenRAVE::RaveInitialize(true);
  } catch(const OpenRAVE::openrave_exception &e) {
    logger->log_error(name(), "Could not initialize OpenRAVE. Ex:%s", e.what());
  }

  __OR_env = new OpenRaveEnvironment(logger);

  __OR_env->create();
  __OR_env->enable_debug(); // TODO: cfg

  __OR_env->set_name("viewer");
}


void
OpenRaveThread::finalize()
{
  __OR_env->destroy();
}


void
OpenRaveThread::loop()
{
}

/* ##########################################################
 * #    methods form OpenRaveConnector (openrave_connector.h)     #
 * ########################################################*/
/** Clone basically everything
 * We pass pointers to pointer as parameters, so the pointers we create before calling this clone()
 *  method will point to the new objects.
 * @param env Pointer to pointer of the copied environment
 * @param robot Pointer to pointer of the copied robot
 * @param manip Pointer to pointer of the copied manipulator
 */
void
OpenRaveThread::clone(OpenRaveEnvironmentPtr& env, OpenRaveRobotPtr& robot, OpenRaveManipulatorPtr& manip) const
{
  env = new OpenRaveEnvironment(**__OR_env);
  robot = new OpenRaveRobot(**__OR_robot, env);
  manip = robot->get_manipulator(); // cloned by OpenRaveRobot copy-ctor

  env->load_IK_solver(robot);
}

/** Get pointer to OpenRaveEnvironment object.
 * @return pointer
 */
OpenRaveEnvironmentPtr
OpenRaveThread::get_environment() const
{
  return __OR_env;
}

/** Get RefPtr to currently used OpenRaveRobot object.
 * @return RefPtr
 */
OpenRaveRobotPtr
OpenRaveThread::get_active_robot() const
{
  return __OR_robot;
}

/** Set robot to be used
 * @param robot OpenRaveRobot that should be used implicitly in other methods
 */
void
OpenRaveThread::set_active_robot(OpenRaveRobotPtr robot)
{
  __OR_robot = robot;
}

/** Set robot to be used
 * @param robot OpenRaveRobot that should be used implicitly in other methods
 */
void
OpenRaveThread::set_active_robot(OpenRaveRobot* robot)
{
  __OR_robot = robot;
}

/** Set OpenRaveManipulator object for robot, and calculate
 * coordinate-system offsets or set them directly.
 * Make sure to update manip angles before calibrating!
 * @param robot pointer to OpenRaveRobot object, explicitly set
 * @param manip pointer to OpenRAVManipulator that is set for robot
 * @param trans_x transition offset on x-axis
 * @param trans_y transition offset on y-axis
 * @param trans_z transition offset on z-axis
 * @param calibrate decides whether to calculate offset (true )or set them directly (false; default)
 */
void
OpenRaveThread::set_manipulator(OpenRaveRobotPtr& robot, OpenRaveManipulatorPtr& manip, float trans_x, float trans_y, float trans_z, bool calibrate)
{
  robot->set_manipulator(manip);
  if( calibrate )
    {robot->calibrate(trans_x, trans_y, trans_z);}
  else
    {robot->set_offset(trans_x, trans_y, trans_z);}
}
/** Set OpenRaveManipulator object for robot, and calculate
 * coordinate-system offsets or set them directly.
 * Make sure to update manip angles before calibrating!
 * Uses default OpenRaveRobot object.
 * @param manip pointer to OpenRAVManipulator that is set for robot
 * @param trans_x transition offset on x-axis
 * @param trans_y transition offset on y-axis
 * @param trans_z transition offset on z-axis
 * @param calibrate decides whether to calculate offset (true )or set them directly (false; default)
 */
void
OpenRaveThread::set_manipulator(OpenRaveManipulatorPtr& manip, float trans_x, float trans_y, float trans_z, bool calibrate)
{
  set_manipulator(__OR_robot, manip, trans_x, trans_y, trans_z, calibrate);
}


/** Start Viewer */
void
OpenRaveThread::start_viewer() const
{
  __OR_env->start_viewer();
}

/** Run planner on previously set target.
 * @param robot robot to use planner on. If none is given, the currently used robot is taken
 * @param sampling sampling time between each trajectory point (in seconds)
 */
void
OpenRaveThread::run_planner(OpenRaveRobotPtr& robot, float sampling)
{
  __OR_env->run_planner(robot, sampling);
}

/** Run planner on previously set target. Uses currently active robot.
 * @param sampling sampling time between each trajectory point (in seconds)
 */
void
OpenRaveThread::run_planner(float sampling)
{
  run_planner(__OR_robot, sampling);
}

/** Run graspplanning script for a given target.
 * @param target_name name of targeted object (KinBody)
 * @param robot robot to use planner on. If none is given, the currently used robot is taken
 */
void
OpenRaveThread::run_graspplanning(const std::string& target_name, OpenRaveRobotPtr& robot)
{
  __OR_env->run_graspplanning(target_name, robot);
}

/** Run graspplanning script for a given target. Uses currently active robot.
 * @param target_name name of targeted object (KinBody)
 */
void
OpenRaveThread::run_graspplanning(const std::string& target_name)
{
  run_graspplanning(target_name, __OR_robot);
}

/** Add a new robot to the environment, and set it as the currently active one.
 * @param filename_robot path to robot's xml file
 * @param autogenerate_IK if true: autogenerate IKfast IK solver for robot
 * @return pointer to new OpenRaveRobot object
 */
OpenRaveRobotPtr
OpenRaveThread::add_robot(const std::string& filename_robot, bool autogenerate_IK)
{
  OpenRaveRobotPtr robot( new OpenRaveRobot(logger));
  robot->load(filename_robot, __OR_env);
  __OR_env->add_robot(robot);
  robot->set_ready();

  if( autogenerate_IK )
    __OR_env->load_IK_solver(robot);

  set_active_robot(robot);

  return robot;
}

/* ##########################################################
 * #    object handling (mainly from environment.h)        #
* ########################################################*/
/** Add an object to the environment.
 * @param name name that should be given to that object
 * @param filename path to xml file of that object (KinBody)
 * @return true if successful */
bool OpenRaveThread::add_object(const std::string& name, const std::string& filename) {
  return __OR_env->add_object(name, filename); }

/** Remove object from environment.
 * @param name name of the object
 * @return true if successful */
bool OpenRaveThread::delete_object(const std::string& name) {
  return __OR_env->delete_object(name); }

/** Remove all objects from environment.
 * @return true if successful */
bool OpenRaveThread::delete_all_objects() {
  return __OR_env->delete_all_objects(); }

/** Rename object.
 * @param name current name of the object
 * @param new_name new name of the object
 * @return true if successful */
bool OpenRaveThread::rename_object(const std::string& name, const std::string& new_name) {
  return __OR_env->rename_object(name, new_name); }

/** Move object in the environment.
 * Distances are given in meters
 * @param name name of the object
 * @param trans_x transition along x-axis
 * @param trans_y transition along y-axis
 * @param trans_z transition along z-axis
 * @return true if successful */
bool OpenRaveThread::move_object(const std::string& name, float trans_x, float trans_y, float trans_z) {
  return __OR_env->move_object(name, trans_x, trans_y, trans_z); }

/** Move object in the environment, relatively to robot.
 * Distances are given in meters
 * @param name name of the object
 * @param trans_x transition along x-axis
 * @param trans_y transition along y-axis
 * @param trans_z transition along z-axis
 * @param robot move relatively to robot (in most simple cases robot is at position (0,0,0) anyway, so this has no effect)
 * @return true if successful */
bool OpenRaveThread::move_object(const std::string& name, float trans_x, float trans_y, float trans_z, OpenRaveRobotPtr& robot) {
  return __OR_env->move_object(name, trans_x, trans_y, trans_z, robot); }

/** Rotate object by a quaternion.
 * @param name name of the object
 * @param quat_x x value of quaternion
 * @param quat_y y value of quaternion
 * @param quat_z z value of quaternion
 * @param quat_w w value of quaternion
 * @return true if successful */
bool OpenRaveThread::rotate_object(const std::string& name, float quat_x, float quat_y, float quat_z, float quat_w) {
  return __OR_env->rotate_object(name, quat_x, quat_y, quat_z, quat_w); }

/** Rotate object along its axis.
 * Rotation angles should be given in radians.
 * @param name name of the object
 * @param rot_x 1st rotation, along x-axis
 * @param rot_y 2nd rotation, along y-axis
 * @param rot_z 3rd rotation, along z-axis
 * @return true if successful */
bool OpenRaveThread::rotate_object(const std::string& name, float rot_x, float rot_y, float rot_z) {
  return __OR_env->rotate_object(name, rot_x, rot_y, rot_z); }

/** Set an object as the target.
 * Currently the object should be cylindric, and stand upright. It may
 * also be rotated on its x-axis, but that rotation needs to be given in an argument
 * to calculate correct position for endeffecto. This is only temporary until
 * proper graps planning for 5DOF in OpenRAVE is provided.
 * @param name name of the object
 * @param robot pointer to OpenRaveRobot that the target is set for
 * @param rot_x rotation of object on x-axis (radians)
 * @return true if IK solvable
 */
bool
OpenRaveThread::set_target_object(const std::string& name, OpenRaveRobotPtr& robot, float rot_x)
{
  OpenRAVE::Transform transform = __OR_env->get_env_ptr()->GetKinBody(name)->GetTransform();

  return robot->set_target_object_position(transform.trans[0], transform.trans[1], transform.trans[2], rot_x);
}

/** Attach a kinbody to the robot.
 * @param name name of the object
 * @param robot pointer to OpenRaveRobot that object is attached to
 * @param manip_name name of the manipulator to attach the object to
 * @return true if successfull
 */
bool
OpenRaveThread::attach_object(const char* name, OpenRaveRobotPtr& robot, const char* manip_name)
{
  return robot->attach_object(name, __OR_env, manip_name);
}

/** Attach a kinbody to the robot. Uses currently active robot.
 * @param name name of the object
 * @param manip_name name of the manipulator to attach the object to
 * @return true if successfull
 */
bool
OpenRaveThread::attach_object(const char* name, const char* manip_name)
{
  return attach_object(name, __OR_robot, manip_name);
}

/** Release a kinbody from the robot.
 * @param name name of the object
 * @param robot pointer to OpenRaveRobot that object is released from
 * @return true if successfull
 */
bool
OpenRaveThread::release_object(const std::string& name, OpenRaveRobotPtr& robot)
{
  return robot->release_object(name, __OR_env);
}

/** Release a kinbody from the robot. Uses currently active robot.
 * @param name name of the object
 * @return true if successfull
 */
bool
OpenRaveThread::release_object(const std::string& name)
{
  return release_object(name, __OR_robot);
}

/** Release all grabbed kinbodys from the robot.
 * @param robot pointer to OpenRaveRobot that objects are released from
 * @return true if successfull
 */
bool
OpenRaveThread::release_all_objects(OpenRaveRobotPtr& robot)
{
  return robot->release_all_objects();
}

/** Release all grabbed kinbodys from the robot. Uses currently active robot.
 * @return true if successfull
 */
bool
OpenRaveThread::release_all_objects()
{
  return release_all_objects(__OR_robot);
}
