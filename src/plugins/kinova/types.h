
/***************************************************************************
 *  types.h - Definition of types for Kinova Jaco
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

#ifndef __PLUGINS_KINOVA_TYPES_H_
#define __PLUGINS_KINOVA_TYPES_H_

#include <core/utils/refptr.h>

#include <string>
#include <vector>
#include <list>

class KinovaGotoThread;
class KinovaOpenraveBaseThread;

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class Mutex;
class KinovaArm;
class JacoInterface;

typedef std::vector<float>               jaco_trajec_point_t;
typedef std::vector<jaco_trajec_point_t> jaco_trajec_t;

typedef enum jaco_target_type_enum {
  TARGET_CARTESIAN,
  TARGET_ANGULAR,
  TARGET_GRIPPER,
  TARGET_TRAJEC,
  TARGET_READY,
  TARGET_RETRACT
} jaco_target_type_t;

typedef struct jaco_target_struct_t {
  jaco_target_type_t            type;
  jaco_trajec_point_t           pos;
  jaco_trajec_point_t           fingers;
  fawkes::RefPtr<jaco_trajec_t> trajec;
} jaco_target_t;

typedef std::list< fawkes::RefPtr<jaco_target_t> > jaco_target_queue_t;

typedef struct jaco_arm_struct {
  fawkes::KinovaArm *arm;
  fawkes::JacoInterface *iface;

  KinovaGotoThread *goto_thread;
  KinovaOpenraveBaseThread *openrave_thread;

  fawkes::RefPtr< fawkes::Mutex > target_mutex;
  fawkes::RefPtr< fawkes::Mutex > trajec_mutex; // very shortly locked mutex

  fawkes::RefPtr< jaco_target_queue_t > target_queue;

  float trajec_color[4]; // RGBA values, each from 0-1
} jaco_arm_t;

typedef struct jaco_dual_arm_struct {
  jaco_arm_t left;
  jaco_arm_t right;
  KinovaOpenraveBaseThread *openrave_thread;
} jaco_dual_arm_t;


} // end namespace fawkes

#endif
