
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

#include <vector>
#include <string>

class KinovaGotoThread;
class KinovaOpenraveBaseThread;

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class KinovaArm;
class JacoInterface;

typedef enum jaco_target_type_enum {
  TARGET_CARTESIAN,
  TARGET_ANGULAR,
  TARGET_READY,
  TARGET_RETRACT
} jaco_target_type_t;

typedef struct jaco_arm_struct {
  fawkes::KinovaArm *arm;
  fawkes::JacoInterface *iface;
  KinovaGotoThread *goto_thread;
  KinovaOpenraveBaseThread *openrave_thread;
} jaco_arm_t;

typedef struct jaco_dual_arm_struct {
  jaco_arm_t left;
  jaco_arm_t right;
  KinovaOpenraveBaseThread *openrave_thread;
} jaco_dual_arm_t;


} // end namespace fawkes

#endif
