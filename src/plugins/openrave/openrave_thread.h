
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

#include "types.h"

#include <plugins/openrave/aspect/openrave_connector.h>
#include <plugins/openrave/aspect/openrave_inifin.h>

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/aspect_provider.h>

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
  virtual void clone(fawkes::OpenRaveEnvironmentPtr& env,
                     fawkes::OpenRaveRobotPtr& robot,
                     fawkes::OpenRaveManipulatorPtr& manip) const;

  virtual fawkes::OpenRaveEnvironmentPtr get_environment() const;
  virtual fawkes::OpenRaveRobotPtr       get_active_robot() const;
  virtual void                           set_active_robot(fawkes::OpenRaveRobotPtr robot);
  virtual void                           set_active_robot(fawkes::OpenRaveRobot* robot);
  virtual fawkes::OpenRaveRobotPtr       add_robot(const std::string& filename_robot, bool autogenerate_IK);

  virtual void set_manipulator(fawkes::OpenRaveManipulatorPtr& manip,
                               float trans_x=0.f, float trans_y=0.f, float trans_z=0.f, bool calibrate=0);
  virtual void set_manipulator(fawkes::OpenRaveRobotPtr& robot, fawkes::OpenRaveManipulatorPtr& manip,
                               float trans_x=0.f, float trans_y=0.f, float trans_z=0.f, bool calibrate=0);

  virtual void start_viewer() const;
  virtual void run_planner(fawkes::OpenRaveRobotPtr& robot, float sampling=0.01f);
  virtual void run_planner(float sampling=0.01f);
  virtual void run_graspplanning(const std::string& target_name, fawkes::OpenRaveRobotPtr& robot);
  virtual void run_graspplanning(const std::string& target_name);

  //handling objects; mainly from environment.h
  virtual bool add_object(const std::string& name, const std::string& filename);
  virtual bool delete_object(const std::string& name);
  virtual bool delete_all_objects();
  virtual bool rename_object(const std::string& name, const std::string& new_name);
  virtual bool move_object(const std::string& name, float trans_x, float trans_y, float trans_z, fawkes::OpenRaveRobotPtr& robot);
  virtual bool move_object(const std::string& name, float trans_x, float trans_y, float trans_z);
  virtual bool rotate_object(const std::string& name, float quat_x, float quat_y, float quat_z, float quat_w);
  virtual bool rotate_object(const std::string& name, float rot_x, float rot_y, float rot_z);
  virtual bool set_target_object(const std::string& name, fawkes::OpenRaveRobotPtr& robot, float rot_x = 0);

  virtual bool attach_object(const char* name, fawkes::OpenRaveRobotPtr& robot, const char* manip_name = NULL);
  virtual bool attach_object(const char* name, const char* manip_name = NULL);
  virtual bool release_object(const std::string& name, fawkes::OpenRaveRobotPtr& robot);
  virtual bool release_object(const std::string& name);
  virtual bool release_all_objects(fawkes::OpenRaveRobotPtr& robot);
  virtual bool release_all_objects();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  fawkes::OpenRaveAspectIniFin   __or_aspectIniFin;

  fawkes::OpenRaveEnvironmentPtr __OR_env;
  fawkes::OpenRaveRobotPtr       __OR_robot;
};

#endif
