
/***************************************************************************
 *  openrave_connector.h - Fawkes OpenRave connector interface
 *
 *  Created: Fri Feb 25 15:08:00 2011
 *  Copyright  2011  Bahram Maleki-Fard
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#ifndef __PLUGINS_OPENRAVE_ASPECT_OPENRAVE_CONNECTOR_H_
#define __PLUGINS_OPENRAVE_ASPECT_OPENRAVE_CONNECTOR_H_

#include <string>
#include <core/utils/refptr.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class OpenRaveEnvironment;
class OpenRaveRobot;
class OpenRaveManipulator;

///\brief RefPtr to OpenRaveEnvironment
typedef RefPtr<OpenRaveEnvironment> OpenRaveEnvironmentPtr;
///\brief RefPtr to OpenRaveRobot
typedef RefPtr<OpenRaveRobot>       OpenRaveRobotPtr;
///\brief RefPtr to OpenRaveManipulator
typedef RefPtr<OpenRaveManipulator> OpenRaveManipulatorPtr;


/** @class OpenRaveConnector <plugins/openrave/aspect/openrave_connector.h>
 * Interface for a OpenRave connection creator.
 * @author Bahram Maleki-Fard
 */
class OpenRaveConnector
{
 public:
  /** Virtual empty destructor. */
  virtual ~OpenRaveConnector() {}

  /** Clone basically everything
   * We pass pointers to pointer as parameters, so the pointers we create before calling this clone()
   *  method will point to the new objects.
   * @param env Pointer to pointer of the copied environment
   * @param robot Pointer to pointer of the copied robot
   * @param manip Pointer to pointer of the copied manipulator
   */
  virtual void clone(OpenRaveEnvironmentPtr& env, OpenRaveRobotPtr& robot, OpenRaveManipulatorPtr& manip) const = 0;

  /** Start OpenRave viewer */
  virtual void start_viewer() const = 0;

  /** Run planner on previously set target.
  * @param robot robot to use planner on. If none is given, the currently used robot is taken
  * @param sampling sampling time between each trajectory point (in seconds)
  */
  virtual void run_planner(OpenRaveRobotPtr& robot, float sampling=0.01f) = 0;

  /** Run planner on previously set target. Uses currently active robot.
  * @param sampling sampling time between each trajectory point (in seconds)
  */
  virtual void run_planner(float sampling=0.01f) = 0;

  /** Run graspplanning script for a given target.
  * @param target_name name of targeted object (KinBody)
  * @param robot robot to use planner on. If none is given, the currently used robot is taken
  */
  virtual void run_graspplanning(const std::string& target_name, OpenRaveRobotPtr& robot) = 0;

  /** Run graspplanning script for a given target. Uses currently active robot.
  * @param target_name name of targeted object (KinBody)
  */
  virtual void run_graspplanning(const std::string& target_name) = 0;

  /** Get pointer to OpenRaveEnvironment object.
  * @return pointer
  */
  virtual OpenRaveEnvironmentPtr get_environment() const = 0;

  /** Get pointer to currently used OpenRaveRobot object.
  * @return pointer
  */
  virtual OpenRaveRobotPtr get_active_robot() const = 0;

  /** Set robot to be used
  * @param robot OpenRaveRobot that should be used implicitly in other methods
  */
  virtual void set_active_robot(OpenRaveRobotPtr robot) = 0;

  /** Set robot to be used
  * @param robot OpenRaveRobot that should be used implicitly in other methods
  */
  virtual void set_active_robot(OpenRaveRobot* robot) = 0;

  /** Add a new robot to the environment, and set it as the currently active one.
  * @param filename_robot path to robot's xml file
  * @param autogenerate_IK if true: autogenerate IKfast IK solver for robot
  * @return pointer to new OpenRaveRobot object
  */
  virtual OpenRaveRobotPtr add_robot(const std::string& filename_robot, bool autogenerate_IK)  = 0;

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
  virtual void set_manipulator(OpenRaveRobotPtr& robot, OpenRaveManipulatorPtr& manip, float trans_x=0.f, float trans_y=0.f, float trans_z=0.f, bool calibrate=0)  = 0;

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
  virtual void set_manipulator(OpenRaveManipulatorPtr& manip, float trans_x=0.f, float trans_y=0.f, float trans_z=0.f, bool calibrate=0)  = 0;

  // object handling methods
  /** Add an object to the environment.
  * @param name name that should be given to that object
  * @param filename path to xml file of that object (KinBody)
  * @return true if successful
  */
  virtual bool add_object(const std::string& name, const std::string& filename) = 0;

  /** Remove object from environment.
  * @param name name of the object
  * @return true if successful
  */
  virtual bool delete_object(const std::string& name) = 0;

  /** Rename object.
  * @param name current name of the object
  * @param new_name new name of the object
  * @return true if successful
  */
  virtual bool rename_object(const std::string& name, const std::string& new_name) = 0;

  /** Move object in the environment.
  * Distances are given in meters
  * @param name name of the object
  * @param trans_x transition along x-axis
  * @param trans_y transition along y-axis
  * @param trans_z transition along z-axis
  * @param robot if given, move relatively to robot (in most simple cases robot is at position (0,0,0) anyway, so this has no effect)
  * @return true if successful
  */
  virtual bool move_object(const std::string& name, float trans_x, float trans_y, float trans_z, OpenRaveRobotPtr& robot) = 0;

  /** Move object in the environment. Uses currently active robot.
  * Distances are given in meters
  * @param name name of the object
  * @param trans_x transition along x-axis
  * @param trans_y transition along y-axis
  * @param trans_z transition along z-axis
  * @return true if successful
  */
  virtual bool move_object(const std::string& name, float trans_x, float trans_y, float trans_z) = 0;

  /** Rotate object by a quaternion.
  * @param name name of the object
  * @param quat_x x value of quaternion
  * @param quat_y y value of quaternion
  * @param quat_z z value of quaternion
  * @param quat_w w value of quaternion
  * @return true if successful
  */
  virtual bool rotate_object(const std::string& name, float quat_x, float quat_y, float quat_z, float quat_w) = 0;

  /** Rotate object along its axis.
  * Rotation angles should be given in radians.
  * @param name name of the object
  * @param rot_x 1st rotation, along x-axis
  * @param rot_y 2nd rotation, along y-axis
  * @param rot_z 3rd rotation, along z-axis
  * @return true if successful
  */
  virtual bool rotate_object(const std::string& name, float rot_x, float rot_y, float rot_z) = 0;

  /** Attach a kinbody to the robot.
  * @param name name of the object
  * @param robot pointer to OpenRaveRobot that the target is set for
  * @param manip_name name of the manipulator to attach the object to
  * @return true if successful
  */
  virtual bool attach_object(const char* name, OpenRaveRobotPtr& robot, const char* manip_name) = 0;

  /** Attach a kinbody to the robot. Uses currently active robot.
  * @param name name of the object
  * @param manip_name name of the manipulator to attach the object to
  * @return true if successful
  */
  virtual bool attach_object(const char* name, const char* manip_name) = 0;

  /** Release a kinbody from the robot.
  * @param name name of the object
  * @param robot pointer to OpenRaveRobot that object is released from
  * @return true if successful
  */
  virtual bool release_object(const std::string& name, OpenRaveRobotPtr& robot) = 0;

  /** Release a kinbody from the robot. Uses currently active robot.
  * @param name name of the object
  * @return true if successful
  */
  virtual bool release_object(const std::string& name) = 0;

  /** Release all grabbed kinbodys from the robot.
  * @param robot pointer to OpenRaveRobot that objects are released from
  * @return true if successful
  */
  virtual bool release_all_objects(OpenRaveRobotPtr& robot) = 0;

  /** Release all grabbed kinbodys from the robot. Uses currently active robot.
  * @return true if successful
  */
  virtual bool release_all_objects() = 0;

  /** Set an object as the target.
  * Currently the object should be cylindric, and stand upright. It may
  * also be rotated on its x-axis, but that rotation needs to be given in an argument
  * to calculate correct position for endeffecto. This is only temporary until
  * proper graps planning for 5DOF in OpenRave is provided.
  * @param name name of the object
  * @param robot pointer to OpenRaveRobot that the target is set for
  * @param rot_x rotation of object on x-axis (radians)
  * @return true if IK solvable
  */
  virtual bool set_target_object(const std::string& name, OpenRaveRobotPtr& robot, float rot_x = 0) = 0;
};

} // end namespace fawkes

#endif
