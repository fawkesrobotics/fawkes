
/***************************************************************************
 *  types.h - Definition of types for Kinova Jaco Plugin
 *
 *  Created: Thu Jun 13 19:14:20 2013
 *  Copyright  2013  Bahram Maleki-Fard
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

#ifndef __PLUGINS_JACO_TYPES_H_
#define __PLUGINS_JACO_TYPES_H_

#include <core/utils/refptr.h>

#include <string>
#include <vector>
#include <list>

class JacoGotoThread;
class JacoOpenraveThread;
class JacoBimanualGotoThread;
class JacoBimanualOpenraveThread;

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class Mutex;
class JacoArm;
class JacoInterface;
class JacoBimanualInterface;

typedef std::vector<float>               jaco_trajec_point_t;
typedef std::vector<jaco_trajec_point_t> jaco_trajec_t;

typedef enum jaco_arm_config_enum {
  CONFIG_SINGLE,    /**< we only have one arm. */
  CONFIG_LEFT,      /**< this arm is the left one out of two. */
  CONFIG_RIGHT      /**< this arm is the right one out of two. */
} jaco_arm_config_t;

typedef enum jaco_target_type_enum {
  TARGET_CARTESIAN,     /**< target with cartesian coordinates. */
  TARGET_ANGULAR,       /**< target with angular coordinates. */
  TARGET_GRIPPER,       /**< only gripper movement. */
  TARGET_READY,         /**< target is the READY position of the Jaco arm. */
  TARGET_RETRACT        /**< target is the RETRACT position of the Jaco arm. */
} jaco_target_type_t;

typedef enum jaco_trajec_state_enum {
  TRAJEC_SKIP,          /**< skip trajectory planning for this target. */
  TRAJEC_WAITING,       /**< new trajectory target, wait for planner to process. */
  TRAJEC_PLANNING,      /**< planner is planning the trajectory. */
  TRAJEC_READY,         /**< trajectory has been planned and is ready for execution. */
  TRAJEC_EXECUTING,     /**< trajectory is being executed. */
  TRAJEC_IK_ERROR,      /**< planner could not find IK solution for target */
  TRAJEC_PLANNING_ERROR /**< planner could not plan a collision-free trajectory. */
} jaco_trajec_state_t;

typedef struct jaco_target_struct_t {
  jaco_target_type_t            type;           /**< target type. */
  jaco_trajec_point_t           pos;            /**< target position (interpreted depending on target type). */
  jaco_trajec_point_t           fingers;        /**< target finger values. */
  fawkes::RefPtr<jaco_trajec_t> trajec;         /**< trajectory, if target is TARGET_TRAJEC. */
  jaco_trajec_state_t           trajec_state;   /**< state of the trajectory, if target is TARGET_TRAJEC. */
  bool                          coord;          /**< this target needs to be coordinated with targets of other arms. */
} jaco_target_t;

typedef std::list< RefPtr<jaco_target_t> > jaco_target_queue_t;

typedef struct jaco_arm_struct {
  jaco_arm_config_t config;
  fawkes::JacoArm *arm;
  JacoInterface *iface;

  JacoGotoThread *goto_thread;
  JacoOpenraveThread *openrave_thread;

  RefPtr< Mutex > target_mutex;
  RefPtr< Mutex > trajec_mutex; // very shortly locked mutex

  RefPtr< jaco_target_queue_t > target_queue;

  float trajec_color[4]; // RGBA values, each from 0-1
} jaco_arm_t;

typedef struct jaco_dual_arm_struct {
  jaco_arm_t *left;
  jaco_arm_t *right;
  JacoBimanualInterface      *iface;
  JacoBimanualGotoThread     *goto_thread;
  JacoBimanualOpenraveThread *openrave_thread;
} jaco_dual_arm_t;


} // end namespace fawkes

#endif
