
/***************************************************************************
 *  types.h - Definition of simple types
 *
 *  Created: Thu Dec 02 13:51:46 2010
 *  Copyright  2010  Bahram Maleki-Fard
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

#ifndef __PLUGINS_OPENRAVE_TYPES_H_
#define __PLUGINS_OPENRAVE_TYPES_H_

#include <openrave/openrave.h>
#include <core/utils/refptr.h>

#include <vector>
#include <string>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class OpenRaveEnvironment;
class OpenRaveRobot;
class OpenRaveManipulator;

typedef RefPtr<OpenRaveEnvironment> OpenRaveEnvironmentPtr;
typedef RefPtr<OpenRaveRobot>       OpenRaveRobotPtr;
typedef RefPtr<OpenRaveManipulator> OpenRaveManipulatorPtr;

/** Euler rotations. */
typedef enum {
  EULER_ZXZ,            /**< ZXZ rotation */
  EULER_ZYZ,            /**< ZYZ rotation */
  EULER_ZYX             /**< ZYX rotation */
} euler_rotation_t;

/** Target types. */
typedef enum {
  TARGET_NONE,          /**< No valid target */
  TARGET_JOINTS,        /**< Target: motor joint values */
  TARGET_TRANSFORM,     /**< Target: absolute endeffector translation and rotation */
  TARGET_RELATIVE,      /**< Target: relative endeffector translation, based on robot's coordinate system */
  TARGET_RELATIVE_EXT,  /**< Target: relative endeffector translation, based on arm extension */
  TARGET_IKPARAM,       /**< Target: OpenRAVE::IkParameterization string */
  TARGET_RAW            /**< Target: Raw string, passed to OpenRAVE's BaseManipulation module */
} target_type_t;


/** Struct containing angle of current motor, its number in OpenRAVE and
 * corresponding motor number of real devices. */
typedef struct {
  unsigned int  no;         /**< motor number in OpenRAVE */
  unsigned int  no_device;  /**< motor number of real device */
  float         angle;      /**< radian angle */
} motor_t;


/** Struct containing information about the current target. */
typedef struct {
  float x;   /**< translation on x-axis */
  float y;   /**< translation on y-axis */
  float z;   /**< translation on z-axis */
  float qx;  /**< x value of quaternion */
  float qy;  /**< y value of quaternion */
  float qz;  /**< z value of quaternion */
  float qw;  /**< w value of quaternion */
  bool solvable;                /**< target IK solvable */
  OpenRaveManipulatorPtr manip; /**< target manipulator configuration */
  target_type_t          type;  /**< target type */
  OpenRAVE::IkParameterization ikparam;  /**< OpenRAVE::IkParameterization; each target is implicitly transformed to one by OpenRAVE */
  std::string plannerparams;    /**< additional string to be passed to planner, i.e. BaseManipulation module */
  std::string raw_cmd;          /**< raw command passed to the BaseManipulator module, e.g. for anything that is not covered */
} target_t;

} // end namespace firevision

#endif
