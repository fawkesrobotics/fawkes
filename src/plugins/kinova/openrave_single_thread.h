
/***************************************************************************
 *  openrave_single_thread.h - Kinova plugin OpenRAVE thread for single-arm setup
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

#ifndef __PLUGINS_KINOVA_OPENRAVE_SINGLE_THREAD_H_
#define __PLUGINS_KINOVA_OPENRAVE_SINGLE_THREAD_H_

#include "openrave_base_thread.h"

#ifdef HAVE_OPENRAVE
 #include <plugins/openrave/aspect/openrave.h>
 #include <openrave/openrave.h>
#endif

#include <string>
#include <vector>

class KinovaOpenraveSingleThread : public KinovaOpenraveBaseThread
{
 public:
  KinovaOpenraveSingleThread(const char *manipname, bool load_robot=true);
  KinovaOpenraveSingleThread(const char *name, const char *manipname, bool load_robot=true);

  virtual void once();
  virtual void loop();
  virtual void finalize();

  virtual void register_arm(fawkes::jaco_arm_t *arm);
  virtual void unregister_arms();

  virtual void update_openrave();

  virtual bool add_target(float x, float y, float z, float e1, float e2, float e3, bool plan=true);
  virtual bool set_target(float x, float y, float z, float e1, float e2, float e3, bool plan=true);

 protected:
  /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
  virtual void run() { Thread::run(); }

 private:
  void _load_robot();
  void _plan_path(std::vector<float> &target);

  fawkes::jaco_arm_t  *__arm;

  std::string __manipname;
  bool        __load_robot;

#ifdef HAVE_OPENRAVE
  OpenRAVE::RobotBasePtr              __robot;
  OpenRAVE::RobotBase::ManipulatorPtr __manip;
  std::vector<OpenRAVE::dReal>        __joints;
#endif
};


#endif
