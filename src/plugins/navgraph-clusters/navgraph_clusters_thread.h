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
#include <plugins/navgraph/aspect/navgraph.h>
#include <blackboard/interface_listener.h>
#include <blackboard/interface_observer.h>
#include <plugins/navgraph/constraints/edge_constraint.h>

#include <Eigen/Geometry>

#include <list>
#include <string>

namespace fawkes {
  class Position3DInterface;
  class Time;
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
  public fawkes::BlackBoardInterfaceListener,
  public fawkes::NavGraphEdgeConstraint
{
 public:
  NavGraphClustersThread();
  virtual ~NavGraphClustersThread();

  // ambiguous, choose Thread::name() instead of NavGraphEdgeConstraint::name()
  using Thread::name;

  virtual void init();
  virtual void loop();
  virtual void finalize();

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

  // for NavGraphEdgeConstraint
  virtual bool compute(void) throw();
  virtual bool blocks(const fawkes::TopologicalMapNode &from,
		      const fawkes::TopologicalMapNode &to) throw();

  void conditional_close(fawkes::Interface *interface) throw();

  Eigen::Vector2f fixed_frame_pose(std::string frame, const fawkes::Time &timestamp,
				   float x, float y);

 private:
  std::string  cfg_iface_prefix_;
  float        cfg_close_threshold_;
  std::string  cfg_fixed_frame_;
  int          cfg_min_vishistory_;

  fawkes::LockList<fawkes::Position3DInterface *>  cluster_ifs_;
  std::list<std::pair<std::string, std::string>>   blocked_;
};

#endif
