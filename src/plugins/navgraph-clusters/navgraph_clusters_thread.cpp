/***************************************************************************
 *  navgraph_clusters_thread.cpp - block paths based on laser clusters
 *
 *  Created: Sun Jul 13 15:30:08 2014
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

#include "navgraph_clusters_thread.h"
#include "clusters_block_constraint.h"

#include <core/threading/mutex_locker.h>
#include <utils/graph/topological_map_graph.h>
#include <tf/utils.h>
#include <interfaces/Position3DInterface.h>
#include <plugins/navgraph/constraints/constraint_repo.h>

#include <Eigen/StdVector>
#include <algorithm>

using namespace fawkes;

using std::find;
using std::make_pair;

/** @class NavGraphClustersThread "navgraph_clusters_thread.h"
 * Block navgraph paths based on laser clusters.
 * @author Tim Niemueller
 */

/** Constructor. */
NavGraphClustersThread::NavGraphClustersThread()
  : Thread("NavGraphClustersThread", Thread::OPMODE_WAITFORWAKEUP),
    BlackBoardInterfaceListener("NavGraphClustersThread")
{
}

/** Destructor. */
NavGraphClustersThread::~NavGraphClustersThread()
{
}

void
NavGraphClustersThread::init()
{
  cfg_iface_prefix_    = config->get_string("/navgraph-clusters/interface-prefix");
  cfg_close_threshold_ = config->get_float("/navgraph-clusters/close-threshold");
  cfg_fixed_frame_     = config->get_string("/frames/fixed");
  cfg_min_vishistory_  = config->get_int("/navgraph-clusters/min-visibility-history");
  cfg_mode_            = config->get_string("/navgraph-clusters/constraint-mode");

  std::string pattern = cfg_iface_prefix_ + "*";

  cluster_ifs_ =
    blackboard->open_multiple_for_reading<Position3DInterface>(pattern.c_str());

  for (Position3DInterface *pif : cluster_ifs_) {
    bbil_add_reader_interface(pif);
    bbil_add_writer_interface(pif);
  }
  blackboard->register_listener(this);

  bbio_add_observed_create("Position3DInterface", pattern.c_str());
  bbio_add_observed_create("Position3DInterface", pattern.c_str());
  blackboard->register_observer(this);

  edge_constraint_ = NULL;
  edge_cost_constraint_ = NULL;
  if (cfg_mode_ == "block") {
    edge_constraint_ = new NavGraphClustersBlockConstraint("clusters", this);
    constraint_repo->register_constraint(edge_constraint_);
  }
}

void
NavGraphClustersThread::finalize()
{
  if (edge_constraint_) {
    constraint_repo->unregister_constraint(edge_constraint_->name());
    delete edge_constraint_;
  }

  blackboard->unregister_listener(this);
  blackboard->unregister_observer(this);

  for (Position3DInterface *pif : cluster_ifs_) {
    blackboard->close(pif);
  }
  cluster_ifs_.clear();
}

void
NavGraphClustersThread::loop()
{
}

void
NavGraphClustersThread::bb_interface_created(const char *type, const char *id) throw()
{
  Position3DInterface *pif;
  try {
    pif = blackboard->open_for_reading<Position3DInterface>(id);
  } catch (Exception &e) {
    // ignored
    logger->log_warn(name(), "Failed to open %s:%s: %s", type, id, e.what_no_backtrace());
    return;
  }

  try {
    bbil_add_data_interface(pif);
    blackboard->update_listener(this);
  } catch (Exception &e) {
    logger->log_warn(name(), "Failed to register for %s:%s: %s",
                     type, id, e.what());
    try {
      bbil_remove_data_interface(pif);
      blackboard->update_listener(this);
      blackboard->close(pif);
    } catch (Exception &e) {
      logger->log_error(name(), "Failed to deregister %s:%s during error recovery: %s",
                        type, id, e.what());
    }
    return;
  }

  cluster_ifs_.push_back_locked(pif);
}

void
NavGraphClustersThread::bb_interface_writer_removed(fawkes::Interface *interface,
						    unsigned int instance_serial) throw()
{
  conditional_close(interface);
}

void
NavGraphClustersThread::bb_interface_reader_removed(fawkes::Interface *interface,
						    unsigned int instance_serial) throw()
{
  conditional_close(interface);
}

void
NavGraphClustersThread::conditional_close(Interface *interface) throw()
{
  Position3DInterface *pif = dynamic_cast<Position3DInterface *>(interface);

  bool close = false;
  MutexLocker lock(cluster_ifs_.mutex());

  LockList<Position3DInterface *>::iterator c =
    std::find(cluster_ifs_.begin(), cluster_ifs_.end(), pif);

  if (c != cluster_ifs_.end() &&
      (! interface->has_writer() && (interface->num_readers() == 1)))
  {
    // It's only us
    logger->log_info(name(), "Last on %s, closing", interface->uid());
    close = true;
    cluster_ifs_.erase(c);
  }

  lock.unlock();

  if (close) {
    std::string uid = interface->uid();
    try {
      bbil_remove_data_interface(interface);
      blackboard->update_listener(this);
      blackboard->close(interface);
    } catch (Exception &e) {
      logger->log_error(name(), "Failed to unregister or close %s: %s",
                        uid.c_str(), e.what());
    }
  }
}


/** Get a list of edges close to a clusters considered blocked.
 * @return list of pairs of blocked edges' start and end name.
 */
std::list<std::pair<std::string, std::string>>
NavGraphClustersThread::blocked_edges() throw()
{
  MutexLocker lock(cluster_ifs_.mutex());
  std::list<std::pair<std::string, std::string>> blocked;

  const std::vector<TopologicalMapEdge> &graph_edges = navgraph->edges();

  for (Position3DInterface *pif : cluster_ifs_) {
    pif->read();
    if (pif->visibility_history() >= cfg_min_vishistory_) {
      try {
	// Should use *pif->timestamp(), but the way things are timed we
	// would always run into an extrapolation exception
	Eigen::Vector2f centroid(fixed_frame_pose(pif->frame(), fawkes::Time(0,0),
						  pif->translation(0), pif->translation(1)));

	for (const TopologicalMapEdge &edge : graph_edges) {
	  const Eigen::Vector2f origin(edge.from_node().x(), edge.from_node().y());
	  const Eigen::Vector2f target(edge.to_node().x(), edge.to_node().y());
	  const Eigen::Vector2f direction(target - origin);
	  const Eigen::Vector2f direction_norm = direction.normalized();
	  const Eigen::Vector2f diff = centroid - origin;
	  const float t = direction.dot(diff) / direction.squaredNorm();

	  if (t >= 0.0 && t <= 1.0) {
	    // projection of the centroid onto the edge is within the line segment
	    float distance = (diff - direction_norm.dot(diff) * direction_norm).norm();
	    if (distance < cfg_close_threshold_) {
	      blocked.push_back(make_pair(edge.from(), edge.to()));
	    }
	  }
	}
      } catch (Exception &e) {
	//logger->log_info(name(), "Failed to transform %s, ignoring", pif->uid());
      }
    }
  }
  blocked.sort();
  blocked.unique();

  return blocked;
}


Eigen::Vector2f
NavGraphClustersThread::fixed_frame_pose(std::string frame, const fawkes::Time &timestamp,
					 float x, float y)
{
  if (frame == cfg_fixed_frame_) {
    return Eigen::Vector2f(x, y);
  } else {
    //logger->log_debug(name(), "Transforming %s -> %s", frame.c_str(),
    //		      cfg_fixed_frame_.c_str());
    tf::Stamped<tf::Point> tpose;
    tf::Stamped<tf::Point> input(tf::Point(x, y, 0), timestamp, frame);
    try {
      tf_listener->transform_point(cfg_fixed_frame_, input, tpose);
      //logger->log_debug(name(), "Transformed %s -> %s:  (%f,%f) -> (%f,%f)", frame.c_str(),
      //		cfg_fixed_frame_.c_str(), x, y, tpose.x(), tpose.y());
      return Eigen::Vector2f(tpose.x(), tpose.y());

    } catch (Exception &e) {
      //logger->log_warn(name(),
      //	       "Failed to transform cluster pose: %s", e.what_no_backtrace());
      throw;
    }
  }
}
