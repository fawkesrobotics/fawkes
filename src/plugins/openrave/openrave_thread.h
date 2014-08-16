
/***************************************************************************
 *  openrave_thread.h - OpenRAVE Thread
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

#ifndef __PLUGINS_OPENRAVE_OPENRAVE_THREAD_H_
#define __PLUGINS_OPENRAVE_OPENRAVE_THREAD_H_

#include <plugins/openrave/aspect/openrave_connector.h>
#include <plugins/openrave/aspect/openrave_inifin.h>

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/aspect_provider.h>

namespace fawkes {
  class OpenRaveEnvironment;
  class OpenRaveRobot;
  class OpenRaveManipulator;
}

class OpenRaveThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::AspectProviderAspect,
  public fawkes::OpenRaveConnector
{
 public:
  OpenRaveThread();
  virtual ~OpenRaveThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

  //for OpenRaveConnector
  //virtual void testDebug();
  virtual void clone(fawkes::OpenRaveEnvironment** env,
                     fawkes::OpenRaveRobot** robot,
                     fawkes::OpenRaveManipulator** manip) const;

  virtual fawkes::OpenRaveEnvironment* get_environment() const;
  virtual fawkes::OpenRaveRobot*       get_active_robot() const;
  virtual void                         set_active_robot(fawkes::OpenRaveRobot* robot);
  virtual fawkes::OpenRaveRobot*       add_robot(const std::string& filename_robot, bool autogenerate_IK);

  virtual void set_manipulator(fawkes::OpenRaveManipulator* manip,
                               float trans_x=0.f, float trans_y=0.f, float trans_z=0.f, bool calibrate=0);
  virtual void set_manipulator(fawkes::OpenRaveRobot* robot, fawkes::OpenRaveManipulator* manip,
                               float trans_x=0.f, float trans_y=0.f, float trans_z=0.f, bool calibrate=0);

  virtual void start_viewer() const;
  virtual void run_planner(fawkes::OpenRaveRobot* = NULL, float sampling=0.01f);
  virtual void run_graspplanning(const std::string& target_name, fawkes::OpenRaveRobot* robot = NULL);

  //handling objects; mainly from environment.h
  virtual bool add_object(const std::string& name, const std::string& filename);
  virtual bool delete_object(const std::string& name);
  virtual bool rename_object(const std::string& name, const std::string& new_name);
  virtual bool move_object(const std::string& name, float trans_x, float trans_y, float trans_z, fawkes::OpenRaveRobot* robot=NULL);
  virtual bool rotate_object(const std::string& name, float quat_x, float quat_y, float quat_z, float quat_w);
  virtual bool rotate_object(const std::string& name, float rot_x, float rot_y, float rot_z);
  virtual bool set_target_object(const std::string& name, fawkes::OpenRaveRobot* robot, float rot_x = 0);

  virtual bool attach_object(const std::string& name, fawkes::OpenRaveRobot* robot=NULL);
  virtual bool release_object(const std::string& name, fawkes::OpenRaveRobot* robot=NULL);
  virtual bool release_all_objects(fawkes::OpenRaveRobot* robot=NULL);

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  fawkes::OpenRaveAspectIniFin     __or_aspectIniFin;

  fawkes::OpenRaveEnvironment*  __OR_env;
  fawkes::OpenRaveRobot*        __OR_robot;
};

#endif
