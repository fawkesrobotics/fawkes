
/***************************************************************************
 *  visualization_thread.h - Visualization for navgraph-generator via rviz
 *
 *  Created: Fri Mar 27 12:07:00 2015
 *  Copyright  2011-2015  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_NAVGRAPH_GENERATOR_VISUALIZATION_THREAD_H_
#define __PLUGINS_NAVGRAPH_GENERATOR_VISUALIZATION_THREAD_H_

#include "navgraph_generator_thread.h"

#include <core/threading/thread.h>
#include <core/threading/mutex.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <plugins/ros/aspect/ros.h>

#include <ros/publisher.h>
#include <visualization_msgs/MarkerArray.h>


class NavGraphGeneratorVisualizationThread
: public fawkes::Thread,
  public fawkes::ConfigurableAspect,
  public fawkes::LoggingAspect,
  public fawkes::ROSAspect
{
 public:
  NavGraphGeneratorVisualizationThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

  void publish(const NavGraphGeneratorThread::ObstacleMap &obstacles,
	       const NavGraphGeneratorThread::ObstacleMap &map_obstacles,
	       const NavGraphGeneratorThread::PoiMap      &pois);

 private:
  std::string  cfg_global_frame_;

  size_t last_id_num_;
  ros::Publisher vispub_;

  NavGraphGeneratorThread::ObstacleMap obstacles_;
  NavGraphGeneratorThread::ObstacleMap map_obstacles_;
  NavGraphGeneratorThread::PoiMap      pois_;
};

#endif
