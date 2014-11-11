
/***************************************************************************
 *  openrave_thread.h - Kinova Jaco plugin OpenRAVE base thread
 *
 *  Created: Tue Jun 04 13:13:20 2013
 *  Copyright  2013  Bahram Maleki-Fard
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

#ifndef __PLUGINS_JACO_OPENRAVE_BASE_THREAD_H_
#define __PLUGINS_JACO_OPENRAVE_BASE_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>
#ifdef HAVE_OPENRAVE
 #include <plugins/openrave/aspect/openrave.h>
#endif

#include <core/utils/refptr.h>

#include "types.h"

#include <string>
#include <list>
#include <vector>

namespace fawkes {
  class Mutex;

#ifdef HAVE_OPENRAVE
  typedef struct {
    OpenRaveEnvironmentPtr env;
    OpenRaveRobotPtr       robot;
    OpenRaveManipulatorPtr manip;
  } jaco_openrave_set_t;
#endif
}

class JacoOpenraveBaseThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
#ifdef HAVE_OPENRAVE
  public fawkes::OpenRaveAspect,
#endif
  public fawkes::BlackBoardAspect
{
 public:
  JacoOpenraveBaseThread(const char *name);
  virtual ~JacoOpenraveBaseThread();

  void init();
  virtual void finalize();

  virtual void set_plannerparams(const std::string &params);
  virtual void set_plannerparams(const char* params);

  /** Update the openrave environment to represent the current situation.
   * This includes updating the model and plotting current positions.
   */
  virtual void update_openrave() = 0;

  /** Plot the first target of the target_queue, if it is a trajectory. */
  virtual void plot_first() = 0;

  virtual void plot_current(bool enable);

 protected:
  /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
  virtual void run() { Thread::run(); }

  /** Use this in inheriting classes for additiona initializations. */
  virtual void _init() {}

  /** Use this in inheriting classes to load the OpenRaveRobot */
  virtual void _load_robot() {}

  /** Use this in inheriting classes for post_init stuff, e.g. env-cloning */
  virtual void _post_init() {}

  fawkes::Mutex *__planning_mutex;      /**< mutex, used to lock when planning. */

#ifdef HAVE_OPENRAVE
  fawkes::jaco_openrave_set_t __viewer_env;

  bool          __cfg_OR_use_viewer;
  std::string   __cfg_OR_robot_file;
  bool          __cfg_OR_auto_load_ik;
  float         __cfg_OR_sampling;
  bool          __cfg_OR_plot_traj_manip;
  bool          __cfg_OR_plot_traj_joints;
  bool          __cfg_OR_plot_cur_manip;
  bool          __cfg_OR_plot_cur_joints;

  std::string   __plannerparams;
  bool          __plot_current;
#endif
};


#endif
