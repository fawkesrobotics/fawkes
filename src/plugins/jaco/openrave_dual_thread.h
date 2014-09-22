
/***************************************************************************
 *  openrave_dual_thread.h - Kinova Jaco plugin OpenRAVE thread for dual-arm setup
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

#ifndef __PLUGINS_JACO_OPENRAVE_DUAL_THREAD_H_
#define __PLUGINS_JACO_OPENRAVE_DUAL_THREAD_H_

#include "openrave_base_thread.h"

#ifdef HAVE_OPENRAVE
 #include <openrave/openrave.h>
#endif

#include <string>

class JacoOpenraveDualThread : public JacoOpenraveBaseThread
{
 public:
  JacoOpenraveDualThread(fawkes::jaco_arm_t *arm_l, fawkes::jaco_arm_t *arm_r);

  virtual void finalize();

  virtual void update_openrave();
  virtual void plot_first();

  virtual bool add_target(float x, float y, float z, float e1, float e2, float e3, bool plan=true);
  virtual bool set_target(float x, float y, float z, float e1, float e2, float e3, bool plan=true);

 protected:
  /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
  virtual void run() { Thread::run(); }

 private:
  void _init();
  void _load_robot();

  struct {
    fawkes::jaco_arm_t *left;
    fawkes::jaco_arm_t *right;
  } __arms;

#ifdef HAVE_OPENRAVE
  struct {
    OpenRAVE::RobotBase::ManipulatorPtr left;
    OpenRAVE::RobotBase::ManipulatorPtr right;
    std::vector<OpenRAVE::dReal> joints_l;
    std::vector<OpenRAVE::dReal> joints_r;
  } __manips;
#endif

};


#endif
