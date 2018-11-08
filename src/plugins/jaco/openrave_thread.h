
/***************************************************************************
 *  openrave_thread.h - Jaco plugin OpenRAVE thread for single arm
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

#ifndef _PLUGINS_JACO_OPENRAVE_THREAD_H_
#define _PLUGINS_JACO_OPENRAVE_THREAD_H_

#include "openrave_base_thread.h"

#ifdef HAVE_OPENRAVE
 #include <plugins/openrave/aspect/openrave.h>
 #include <openrave/openrave.h>
#endif

#include <string>
#include <vector>

class JacoOpenraveThread : public JacoOpenraveBaseThread
{
 public:
  JacoOpenraveThread(const char *name, fawkes::jaco_arm_t* arm, bool load_robot=true);

  virtual void loop();
  virtual void finalize();

  virtual void update_openrave();
  virtual void plot_first();

  virtual bool add_target(float x, float y, float z, float e1, float e2, float e3, bool plan=true);
  virtual bool add_target_ang(float j1, float j2, float j3, float j4, float j5, float j6, bool plan=true);
  virtual bool set_target(float x, float y, float z, float e1, float e2, float e3, bool plan=true);
  virtual bool set_target_ang(float j1, float j2, float j3, float j4, float j5, float j6, bool plan=true);

 protected:
  /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
  virtual void run() { Thread::run(); }

 private:
  void _init();
  void _load_robot();
  void _post_init();

  void _plan_path(fawkes::RefPtr<fawkes::jaco_target_t> &from, fawkes::RefPtr<fawkes::jaco_target_t> &to);

  fawkes::jaco_arm_t  *arm_;

  std::string cfg_manipname_;
  bool        load_robot_;

#ifdef HAVE_OPENRAVE
  fawkes::jaco_openrave_set_t planner_env_;

  OpenRAVE::RobotBasePtr              robot_;
  OpenRAVE::RobotBase::ManipulatorPtr manip_;
  std::vector<OpenRAVE::dReal>        joints_;

  std::vector<OpenRAVE::GraphHandlePtr> graph_handle_;
  std::vector<OpenRAVE::GraphHandlePtr> graph_current_;

  std::vector<OpenRAVE::KinBody::LinkPtr> links_;

  bool plotted_current_;   // keep track of plotting
#endif
};


#endif
