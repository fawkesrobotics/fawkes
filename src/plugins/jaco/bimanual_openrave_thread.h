
/***************************************************************************
 *  bimanual_openrave_thread.h - Jaco plugin OpenRAVE thread for bimanual manipulatin
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

#ifndef __PLUGINS_JACO_BIMANUAL_OPENRAVE_THREAD_H_
#define __PLUGINS_JACO_BIMANUAL_OPENRAVE_THREAD_H_

#include "openrave_base_thread.h"

#ifdef HAVE_OPENRAVE
 #include <openrave/openrave.h>
#endif

#include <string>

class JacoBimanualOpenraveThread : public JacoOpenraveBaseThread
{
 public:
  JacoBimanualOpenraveThread(fawkes::jaco_dual_arm_t *arms);

  virtual void loop();
  virtual void finalize();

  virtual void update_openrave();
  virtual void plot_first();

  virtual bool add_target(float l_x, float l_y, float l_z, float l_e1, float l_e2, float l_e3,
                          float r_x, float r_y, float r_z, float r_e1, float r_e2, float r_e3);

  virtual void set_constrained(bool enable);

 protected:
  /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
  virtual void run() { Thread::run(); }

 private:
  void _init();
  void _load_robot();
  void _init_dualmanipulation();

  void _set_trajec_state(fawkes::jaco_trajec_state_t state);
  void _copy_env();
  bool _solve_multi_ik(std::vector<float> &left,
                       std::vector<float> &right);
  bool _plan_path();

  typedef struct arm_struct {
    fawkes::jaco_arm_t                    *arm;
#ifdef HAVE_OPENRAVE
    std::string                           manipname;
    OpenRAVE::RobotBase::ManipulatorPtr   manip;
    fawkes::RefPtr<fawkes::jaco_target_t> target;
#endif
  } arm_struct_t;

  struct {
    arm_struct_t left;
    arm_struct_t right;
  } __arms;

#ifdef HAVE_OPENRAVE
  fawkes::jaco_openrave_set_t __planner_env;
  OpenRAVE::ModuleBasePtr __mod_dualmanip;

  std::set<OpenRAVE::KinBody::LinkPtr> links_left_;
  std::set<OpenRAVE::KinBody::LinkPtr> links_right_;
#endif

  bool __constrained;
};


#endif
