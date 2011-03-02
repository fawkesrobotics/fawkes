
/***************************************************************************
 *  or_connector.h - Fawkes OpenRAVE connector interface
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

#ifndef __PLUGINS_OPENRAVE_ASPECT_OR_CONNECTOR_H_
#define __PLUGINS_OPENRAVE_ASPECT_OR_CONNECTOR_H_

//#include <plugins/openrave/aspect/or_descriptions.h>
#include <string>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class OpenRAVEEnvironment;
class OpenRAVERobot;
class OpenRAVEManipulator;

/** @class OpenRAVEConnector <plugins/openrave/aspect/or_manager.h>
 * Interface for a OpenRAVE connection creator.
 * @author Bahram Maleki-Fard
 */
class OpenRAVEConnector
{
 public:
  /** Virtual empty destructor. */
  virtual ~OpenRAVEConnector() {}

  /** Start OpenRAVE viewer */
  virtual void start_viewer() const = 0;

  /** Run planner on previously set target.
  * @param robot robot to use planner on. If none is given, the currently used robot is taken
  */
  virtual void run_planner(OpenRAVERobot* = NULL) = 0;

  /** Get pointer to OpenRAVEEnvironment object.
  * @return pointer
  */
  virtual OpenRAVEEnvironment* get_environment() const = 0;

  /** Get pointer to currently used OpenRAVERobot object.
  * @return pointer
  */
  virtual OpenRAVERobot* get_active_robot() const = 0;

  /** Set robot to be used
  * @param robot OpenRAVERobot that should be used implicitly in other methods
  */
  virtual void set_active_robot(OpenRAVERobot* robot) = 0;

  /** Add a new robot to the environment, and set it as the currently active one.
  * @param filename_robot path to robot's xml file
  * @param autogenerate_IK if true: autogenerate IKfast IK solver for robot
  * @return pointer to new OpenRAVERobot object
  */
  virtual OpenRAVERobot* add_robot(const std::string& filename_robot, bool autogenerate_IK)  = 0;

/** Set OpenRAVEManipulator object for given robot, and calculate
 * coordinate-system offsets or set them directly.
 * Make sure to update manip angles before calibrating!
 * @param calibrate decides whether to calculate offset (true )or set them directly (false; default)
 */
  virtual void set_manipulator(OpenRAVERobot* robot, OpenRAVEManipulator* manip, float trans_x=0.f, float trans_y=0.f, float trans_z=0.f, bool calibrate=0)  = 0;
  virtual void set_manipulator(OpenRAVEManipulator* manip, float trans_x=0.f, float trans_y=0.f, float trans_z=0.f, bool calibrate=0)  = 0;

  // object handling methods
  /** Add an object to the environment.
  * @param name name that should be given to that object
  * @param filename path to xml file of that object (KinBody)
  */
  virtual bool add_object(const std::string& name, const std::string& filename) = 0;

  /** Remove object from environment.
  * @param name name of the object
  */
  virtual bool delete_object(const std::string& name) = 0;

  /** Rename object.
  * @param name current name of the object
  * @param new_name new name of the object
  */
  virtual bool rename_object(const std::string& name, const std::string& new_name) = 0;

  /** Move object in the environment.
  * Distances are given in meters
  * @param name name of the object
  * @param trans_x transition along x-axis
  * @param trans_y transition along y-axis
  * @param trans_z transition along z-axis
  * @param robot if given, move relatively to robot (in most simple cases robot is at position (0,0,0) anyway, so this has no effect)
  */
  virtual bool move_object(const std::string& name, float trans_x, float trans_y, float trans_z, OpenRAVERobot* robot=NULL) = 0;

  /** Rotate object along its axis.
  * Rotation angles should be given in radians.
  * @param name name of the object
  * @param rot_x 1st rotation, along x-axis
  * @param rot_y 2nd rotation, along y-axis
  * @param rot_z 3rd rotation, along z-axis
  */
  virtual bool rotate_object(const std::string& name, float rot_x, float rot_y, float rot_z) = 0;

  /** Set an object as the target.
  * Currently the object should be cylindric, and stand upright. It may
  * also be rotated on its x-axis, but that rotation needs to be given in an argument
  * to calculate correct position for endeffecto. This is only temporary until
  * proper graps planning for 5DOF in OpenRAVE is provided.
  * @param name name of the object
  * @param rot_x rotation of object on x-axis (radians)
  */
  virtual bool set_target_object(const std::string& name, OpenRAVERobot* robot, float rot_x = 0) = 0;
};

} // end namespace fawkes

#endif
