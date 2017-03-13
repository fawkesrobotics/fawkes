/***************************************************************************
 *  navgraph_interactive_thread.cpp - ROSPub navgraph editing
 *
 *  Created: Thu Jan 15 16:26:40 2015
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

#include "rospub_thread.h"

#include <core/threading/mutex_locker.h>

#include <ros/ros.h>
#include <fawkes_msgs/NavGraph.h>
#include <fawkes_msgs/NavGraphNode.h>
#include <fawkes_msgs/NavGraphEdge.h>

using namespace fawkes;

/** @class NavGraphROSPubThread "rospub_thread.h"
 * Publish navgaraph to ROS.
 * @author Tim Niemueller
 */

/** Constructor. */
NavGraphROSPubThread::NavGraphROSPubThread()
  : Thread("NavGraphROSPubThread", Thread::OPMODE_WAITFORWAKEUP)
{
}

/** Destructor. */
NavGraphROSPubThread::~NavGraphROSPubThread()
{
}


void
NavGraphROSPubThread::init()
{
	pub_ = rosnode->advertise<fawkes_msgs::NavGraph>("navgraph", 10, /* latching */ true);

	publish_graph();
	
	navgraph->add_change_listener(this);
}

void
NavGraphROSPubThread::finalize()
{
	navgraph->remove_change_listener(this);
	pub_.shutdown();
}

void
NavGraphROSPubThread::loop()
{
}


void
NavGraphROSPubThread::graph_changed() throw()
{
	try {
		publish_graph();
	} catch (fawkes::Exception &e) {
		logger->log_warn(name(), "Failed to publish graph, exception follows");
		logger->log_warn(name(), e);
	} catch (std::runtime_error &e) {
		logger->log_warn(name(), "Failed to publish graph: %s", e.what());
	}
}

void
NavGraphROSPubThread::publish_graph()
{
	MutexLocker lock(navgraph.objmutex_ptr());

	fawkes_msgs::NavGraph ngm;

	const std::vector<NavGraphNode> &nodes = navgraph->nodes();
	for (const NavGraphNode &node : nodes) {
		fawkes_msgs::NavGraphNode ngn;
		ngn.name = node.name();
		ngn.has_orientation = node.has_property(navgraph::PROP_ORIENTATION);
		ngn.pose.x = node.x();
		ngn.pose.y = node.y();
		if (ngn.has_orientation) {
			ngn.pose.theta = node.property_as_float(navgraph::PROP_ORIENTATION);
		}
		ngn.unconnected = node.unconnected();
		const std::map<std::string, std::string> &props = node.properties();
		for (const auto p : props) {
			fawkes_msgs::NavGraphProperty ngp;
			ngp.key = p.first;
			ngp.value = p.second;
			ngn.properties.push_back(ngp);
		}
		ngm.nodes.push_back(ngn);
	}
	const std::vector<NavGraphEdge> &edges = navgraph->edges();
	for (const NavGraphEdge &edge : edges) {
		fawkes_msgs::NavGraphEdge nge;
		nge.from_node = edge.from();
		nge.to_node = edge.to();
		nge.directed = edge.is_directed();
		const std::map<std::string, std::string> &props = edge.properties();
		for (const auto p : props) {
			fawkes_msgs::NavGraphProperty ngp;
			ngp.key = p.first;
			ngp.value = p.second;
			nge.properties.push_back(ngp);
		}
		ngm.edges.push_back(nge);
	}
	
	pub_.publish(ngm);
}
