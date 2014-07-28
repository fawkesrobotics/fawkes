
/***************************************************************************
 *  openrave_dual_thread.h - Kinova plugin OpenRAVE thread for dual-arm setup
 *
 *  Created: Mon Jul 28 19:43:20 2014
 *  Copyright  2014  Bahram Maleki-Fard
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

#ifndef __PLUGINS_KINOVA_OPENRAVE_DUAL_THREAD_H_
#define __PLUGINS_KINOVA_OPENRAVE_DUAL_THREAD_H_

#include "openrave_base_thread.h"

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>
#ifdef HAVE_OPENRAVE
 #include <plugins/openrave/aspect/openrave.h>
 #include <openrave/openrave.h>
#endif

#include <string>

class KinovaOpenraveDualThread : public KinovaOpenraveBaseThread
{
 public:
  KinovaOpenraveDualThread();

  virtual void loop();

  virtual void register_arm(fawkes::jaco_arm_t *arm);
  virtual void unregister_arms();

  virtual std::vector<float> set_target(float x, float y, float z, float e1, float e2, float e3, fawkes::jaco_arm_t *arm);

 protected:
  /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
  virtual void run() { Thread::run(); }

 private:
  void _init();
  void _load_robot();

  std::string __cfg_left_arm_name;

  struct {
    fawkes::jaco_arm_t *left;
    fawkes::jaco_arm_t *right;
  } __arms;

#ifdef HAVE_OPENRAVE
  struct {
    OpenRAVE::RobotBase::ManipulatorPtr left;
    OpenRAVE::RobotBase::ManipulatorPtr right;
    std::vector<double> joints_l;
    std::vector<double> joints_r;
  } __manips;
#endif

};


#endif
