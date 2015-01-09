/***************************************************************************
 *  navgraph_clusters_thread.h - block paths based on laser clusters
 *
 *  Created: Sun Jul 13 15:30:03 2014
 *  Copyright  2014  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_NAVGRAPH_CLUSTERS_NAVGRAPH_CLUSTERS_THREAD_H_
#define __PLUGINS_NAVGRAPH_CLUSTERS_NAVGRAPH_CLUSTERS_THREAD_H_

#include <core/threading/thread.h>
#include <core/utils/lock_list.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/tf.h>
#include <aspect/blackboard.h>
#include <navgraph/aspect/navgraph.h>
#include <blackboard/interface_listener.h>
#include <blackboard/interface_observer.h>

#include <Eigen/Geometry>

#include <list>
#include <string>
#include <tuple>

namespace fawkes {
  class Position3DInterface;
  class Time;
  class NavGraphEdgeConstraint;
  class NavGraphEdgeCostConstraint;
}

class NavGraphClustersThread
: public fawkes::Thread,
  public fawkes::ClockAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::TransformAspect,
  public fawkes::NavGraphAspect,
  public fawkes::BlackBoardInterfaceObserver,
  public fawkes::BlackBoardInterfaceListener
{
 public:
  NavGraphClustersThread();
  virtual ~NavGraphClustersThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

  std::list<std::pair<std::string, std::string>> blocked_edges() throw();

  std::list<std::tuple<std::string, std::string, Eigen::Vector2f>>
    blocked_edges_centroids() throw();

  bool robot_pose(Eigen::Vector2f &pose) throw();

  /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run();}

 private:
  // for BlackBoardInterfaceObserver
  virtual void bb_interface_created(const char *type, const char *id) throw();

  // for BlackBoardInterfaceListener
  virtual void bb_interface_writer_removed(fawkes::Interface *interface,
                                           unsigned int instance_serial) throw();
  virtual void bb_interface_reader_removed(fawkes::Interface *interface,
                                           unsigned int instance_serial) throw();

  void conditional_close(fawkes::Interface *interface) throw();

  Eigen::Vector2f fixed_frame_pose(std::string frame, const fawkes::Time &timestamp,
				   float x, float y);

 private:
  std::string  cfg_iface_prefix_;
  float        cfg_close_threshold_;
  std::string  cfg_fixed_frame_;
  std::string  cfg_base_frame_;
  int          cfg_min_vishistory_;
  std::string  cfg_mode_;

  fawkes::LockList<fawkes::Position3DInterface *>  cluster_ifs_;

  fawkes::NavGraphEdgeConstraint *edge_constraint_;
  fawkes::NavGraphEdgeCostConstraint *edge_cost_constraint_;
};

#endif
