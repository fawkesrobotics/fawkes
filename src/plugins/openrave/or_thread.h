
/***************************************************************************
 *  or_thread.h - OpenRAVE Thread
 *
 *  Created: Fri Feb 25 15:08:00 2011
 *  Copyright  2011  Bahram Maleki-Fard
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

#ifndef __PLUGINS_OPENRAVE_OR_THREAD_H_
#define __PLUGINS_OPENRAVE_OR_THREAD_H_

#include <plugins/openrave/aspect/or_connector.h>
#include <plugins/openrave/aspect/or_inifin.h>

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/aspect_provider.h>

namespace fawkes {
  class OpenRAVEEnvironment;
  class OpenRAVERobot;
  class OpenRAVEManipulator;
}

class OpenRAVEThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::AspectProviderAspect,
  public fawkes::OpenRAVEConnector
{
 public:
  OpenRAVEThread();
  virtual ~OpenRAVEThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

  //for OpenRAVEConnector
  //virtual void testDebug();
  virtual fawkes::OpenRAVEEnvironment*	get_environment() const;
  virtual fawkes::OpenRAVERobot*	get_active_robot() const;
  virtual void				set_active_robot(fawkes::OpenRAVERobot* robot);
  virtual fawkes::OpenRAVERobot*	add_robot(const std::string& filename_robot, bool autogenerate_IK);
  virtual void 				set_manipulator(fawkes::OpenRAVEManipulator* manip, float trans_x=0.f, float trans_y=0.f, float trans_z=0.f, bool calibrate=0);
  virtual void 				set_manipulator(fawkes::OpenRAVERobot* robot, fawkes::OpenRAVEManipulator* manip, float trans_x=0.f, float trans_y=0.f, float trans_z=0.f, bool calibrate=0);

  virtual void start_viewer() const;
  virtual void run_planner(fawkes::OpenRAVERobot* = NULL);

  //handling objects; mainly from environment.h
  virtual bool add_object(const std::string& name, const std::string& filename);
  virtual bool delete_object(const std::string& name);
  virtual bool rename_object(const std::string& name, const std::string& new_name);
  virtual bool move_object(const std::string& name, float trans_x, float trans_y, float trans_z, fawkes::OpenRAVERobot* robot=NULL);
  virtual bool rotate_object(const std::string& name, float rot_x, float rot_y, float rot_z);
  virtual bool set_target_object(const std::string& name, fawkes::OpenRAVERobot* robot, float rot_x = 0);

  virtual bool attach_object(const std::string& name, fawkes::OpenRAVERobot* robot=NULL);
  virtual bool release_object(const std::string& name, fawkes::OpenRAVERobot* robot=NULL);
  virtual bool release_all_objects(fawkes::OpenRAVERobot* robot=NULL);

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  fawkes::OpenRAVEAspectIniFin     __or_aspectIniFin;

  fawkes::OpenRAVEEnvironment*  __OR_env;
  fawkes::OpenRAVERobot*        __OR_robot;
};

#endif
