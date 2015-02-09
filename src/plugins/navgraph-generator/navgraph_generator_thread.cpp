/***************************************************************************
 *  navgraph_generator_thread.cpp - Plugin to generate navgraphs
 *
 *  Created: Mon Feb 09 17:37:30 2015
 *  Copyright  2015  Tim Niemueller [www.niemueller.de]
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

#include "navgraph_generator_thread.h"

#include <core/threading/mutex_locker.h>
#include <navgraph/generators/voronoi.h>

using namespace fawkes;

/** @class NavGraphGeneratorThread "navgraph_generator_thread.h"
 * Thread to perform graph-based path planning.
 * @author Tim Niemueller
 */

/** Constructor. */
NavGraphGeneratorThread::NavGraphGeneratorThread()
  : Thread("NavGraphGeneratorThread", Thread::OPMODE_WAITFORWAKEUP),
    BlackBoardInterfaceListener("NavGraphGeneratorThread")
{
}


/** Destructor. */
NavGraphGeneratorThread::~NavGraphGeneratorThread()
{
}

void
NavGraphGeneratorThread::init()
{
  bbox_set_ = false;

  navgen_if_ =
    blackboard->open_for_writing<NavGraphGeneratorInterface>("/navgraph-generator");
  bbil_add_message_interface(navgen_if_);
  blackboard->register_listener(this, BlackBoard::BBIL_FLAG_MESSAGES);
}

void
NavGraphGeneratorThread::finalize()
{
  blackboard->unregister_listener(this);
  bbil_remove_message_interface(navgen_if_);
  blackboard->close(navgen_if_);
}


void
NavGraphGeneratorThread::loop()
{
  NavGraphGeneratorVoronoi nggv;

  logger->log_debug(name(), "Calculating new graph");

  if (bbox_set_) {
    logger->log_debug(name(), "  Setting bound box (%f,%f) to (%f,%f)",
		      bbox_p1_.x, bbox_p1_.y, bbox_p2_.x, bbox_p2_.y);
    nggv.set_bounding_box(bbox_p1_.x, bbox_p1_.y, bbox_p2_.x, bbox_p2_.y);
  }

  for (auto o : obstacles_) {
    logger->log_debug(name(), "  Adding obstacle %s at (%f,%f)",
		      o.first.c_str(), o.second.x, o.second.y);
    nggv.add_obstacle(o.second.x, o.second.y);
  }

  // Acquire lock on navgraph, no more searches/modifications until we are done
  MutexLocker lock(navgraph.objmutex_ptr());

  // remember default properties
  std::map<std::string, std::string> default_props = navgraph->default_properties();

  navgraph->clear();
  logger->log_debug(name(), "  Computing Voronoi");
  nggv.compute(navgraph);

  // restore default properties
  navgraph->set_default_properties(default_props);

  // add POIs
  for (auto p : pois_) {
    // add poi
    NavGraphNode node(p.first,
		      p.second.position.x, p.second.position.y,
		      p.second.properties);
    switch (p.second.conn_mode) {
    case NavGraphGeneratorInterface::CLOSEST_NODE:
      logger->log_debug(name(), "  Connecting POI %s at (%f,%f) to closest node",
			p.first.c_str(), p.second.position.x, p.second.position.y);
      navgraph->add_node_and_connect(node, NavGraph::CLOSEST_NODE);
      break;
    case NavGraphGeneratorInterface::CLOSEST_EDGE:
      try {
	logger->log_debug(name(), "  Connecting POI %s at (%f,%f) to closest edge",
			  p.first.c_str(), p.second.position.x, p.second.position.y);
	navgraph->add_node_and_connect(node, NavGraph::CLOSEST_EDGE);
      } catch (Exception &e) {
	logger->log_error(name(), "  Failed to add POI %s, exception follows",
			  p.first.c_str());
	logger->log_error(name(), e);
      }
      break;
    case NavGraphGeneratorInterface::CLOSEST_EDGE_OR_NODE:
      logger->log_debug(name(), "  Connecting POI %s at (%f,%f) to closest edge or node",
			p.first.c_str(), p.second.position.x, p.second.position.y);
      navgraph->add_node_and_connect(node, NavGraph::CLOSEST_EDGE_OR_NODE);
      break;
    }
  }

  // Finalize graph setup
  try {
    logger->log_debug(name(), "  Calculate reachability relations");
    navgraph->calc_reachability();
  } catch (Exception &e) {
    logger->log_error(name(), "Failed to finalize graph setup, exception follows");
    logger->log_error(name(), e);
  }

  logger->log_debug(name(), "  Graph computed, notifying listeners");
  navgraph->notify_of_change();
}


bool
NavGraphGeneratorThread::bb_interface_message_received(Interface *interface,
						       Message *message) throw()
{
  // in continuous mode wait for signal if disabled
  MutexLocker lock(loop_mutex);

  if (message->is_of_type<NavGraphGeneratorInterface::ClearMessage>()) {
    obstacles_.clear();
    pois_.clear();
    bbox_set_ = false;
  } else if (message->is_of_type<NavGraphGeneratorInterface::SetBoundingBoxMessage>()) {
    NavGraphGeneratorInterface::SetBoundingBoxMessage *msg =
      message->as_type<NavGraphGeneratorInterface::SetBoundingBoxMessage>();
    bbox_set_  = true;
    bbox_p1_.x = msg->p1_x();
    bbox_p1_.y = msg->p1_y();
    bbox_p2_.x = msg->p2_x();
    bbox_p2_.y = msg->p2_y();

  } else if (message->is_of_type<NavGraphGeneratorInterface::AddObstacleMessage>()) {
    NavGraphGeneratorInterface::AddObstacleMessage *msg =
      message->as_type<NavGraphGeneratorInterface::AddObstacleMessage>();
    obstacles_[msg->id()] = cart_coord_2d_t(msg->x(), msg->y());

  } else if (message->is_of_type<NavGraphGeneratorInterface::RemoveObstacleMessage>()) {
    NavGraphGeneratorInterface::RemoveObstacleMessage *msg =
      message->as_type<NavGraphGeneratorInterface::RemoveObstacleMessage>();
    ObstacleMap::iterator f;
    if ((f = obstacles_.find(msg->id())) != obstacles_.end()) {
      obstacles_.erase(f);
    }

  } else if (message->is_of_type<NavGraphGeneratorInterface::AddPointOfInterestMessage>()) {
    NavGraphGeneratorInterface::AddPointOfInterestMessage *msg =
      message->as_type<NavGraphGeneratorInterface::AddPointOfInterestMessage>();
    PointOfInterest poi;
    poi.position  = cart_coord_2d_t(msg->x(), msg->y());
    poi.conn_mode = msg->mode();
    pois_[msg->id()] = poi;

  } else if (message->is_of_type<NavGraphGeneratorInterface::RemovePointOfInterestMessage>()) {
    NavGraphGeneratorInterface::RemovePointOfInterestMessage *msg =
      message->as_type<NavGraphGeneratorInterface::RemovePointOfInterestMessage>();
    PoiMap::iterator f;
    if ((f = pois_.find(msg->id())) != pois_.end()) {
      pois_.erase(f);
    }

  } else if (message->is_of_type<NavGraphGeneratorInterface::SetPointOfInterestPropertyMessage>()) {
    NavGraphGeneratorInterface::SetPointOfInterestPropertyMessage *msg =
      message->as_type<NavGraphGeneratorInterface::SetPointOfInterestPropertyMessage>();
    PoiMap::iterator f;
    if ((f = pois_.find(msg->id())) != pois_.end()) {
      f->second.properties[msg->property_name()] = msg->property_value();
    } else {
      logger->log_warn(name(), "POI %s unknown, cannot set propety %s=%s",
		       msg->id(), msg->property_name(), msg->property_value());
    }
  } else if (message->is_of_type<NavGraphGeneratorInterface::ComputeMessage>()) {
    wakeup();

  } else {
    logger->log_error(name(), "Received unknown message of type %s, ignoring",
		      message->type());
  }

  return false;
}

