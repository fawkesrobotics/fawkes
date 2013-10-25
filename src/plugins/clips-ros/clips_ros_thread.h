
/***************************************************************************
 *  clips_ros_thread.h - ROS integration for CLIPS
 *
 *  Created: Tue Oct 22 18:12:19 2013
 *  Copyright  2006-2013  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_CLIPS_ROS_CLIPS_ROS_THREAD_H_
#define __PLUGINS_CLIPS_ROS_CLIPS_ROS_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <plugins/clips/aspect/clips_feature.h>
#include <plugins/ros/aspect/ros.h>

#include <clipsmm.h>

#include <map>
#include <string>


class ClipsROSThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::ROSAspect,
  public fawkes::CLIPSFeature,
  public fawkes::CLIPSFeatureAspect
{
 public:
  ClipsROSThread();
  virtual ~ClipsROSThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

  // for CLIPSFeature
  virtual void clips_context_init(const std::string &env_name,
				  fawkes::LockPtr<CLIPS::Environment> &clips);
  virtual void clips_context_destroyed(const std::string &env_name);

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private: // members
  std::map<std::string, fawkes::LockPtr<CLIPS::Environment> >  envs_;

 private: // methods
  class RosNodeInfo {
   public:
    std::list<std::string> published;
    std::list<std::string> subscribed;
    std::list<std::string> services;
  };

  void clips_ros_get_nodes(std::string env_name);
  void clips_ros_get_topics(std::string env_name);
  void clips_ros_get_topic_connections(std::string env_name);

};

#endif
