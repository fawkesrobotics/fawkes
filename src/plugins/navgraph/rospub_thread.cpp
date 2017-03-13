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

#include <cmath>

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
  cfg_base_frame_      = config->get_string("/frames/base");
  cfg_global_frame_    = config->get_string("/frames/fixed");

  pub_ = rosnode->advertise<fawkes_msgs::NavGraph>("navgraph", 10, /* latching */ true);
	svs_search_path_ = rosnode->advertiseService("navgraph/search_path",
	                                             &NavGraphROSPubThread::svs_search_path_cb, this);
	svs_get_pwcosts_ = rosnode->advertiseService("navgraph/get_pairwise_costs",
	                                             &NavGraphROSPubThread::svs_get_pwcosts_cb, this);

	publish_graph();
	
	navgraph->add_change_listener(this);
}

void
NavGraphROSPubThread::finalize()
{
	navgraph->remove_change_listener(this);
	pub_.shutdown();
	svs_search_path_.shutdown();
	svs_get_pwcosts_.shutdown();
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
NavGraphROSPubThread::convert_nodes(const std::vector<fawkes::NavGraphNode> &nodes,
                                    std::vector<fawkes_msgs::NavGraphNode> &out)
{
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
		out.push_back(ngn);
	}
}

void
NavGraphROSPubThread::publish_graph()
{
	MutexLocker lock(navgraph.objmutex_ptr());

	fawkes_msgs::NavGraph ngm;

	const std::vector<NavGraphNode> &nodes = navgraph->nodes();
	convert_nodes(nodes, ngm.nodes);

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


bool
NavGraphROSPubThread::svs_search_path_cb(fawkes_msgs::NavGraphSearchPath::Request  &req,
                                         fawkes_msgs::NavGraphSearchPath::Response &res)
{
	NavGraphNode from, to;


	if (req.from_node.empty()) {
		fawkes::tf::Stamped<fawkes::tf::Pose> pose;
		if (! tf_listener->transform_origin(cfg_base_frame_, cfg_global_frame_, pose)) {
			logger->log_warn(name(),
			                 "Failed to compute pose, cannot generate plan");

			res.ok = false;
			res.errmsg = "Failed to compute pose, cannot generate plan";
			return true;
		}

		from =
			navgraph->closest_node(pose.getOrigin().x(), pose.getOrigin().y());
		if (! from.is_valid()) {
			res.ok = false;
			res.errmsg = "Failed to get closest node to pose";
			return true;
		}

		fawkes_msgs::NavGraphNode free_start;
		free_start.name = "free-start";
		free_start.pose.x = pose.getOrigin().x();
		free_start.pose.y = pose.getOrigin().y();
		free_start.has_orientation = true;
		free_start.pose.theta = tf::get_yaw(pose.getRotation());
		res.path.push_back(free_start);
	} else {
		from = navgraph->node(req.from_node);
		if (! from.is_valid()) {
			res.ok = false;
			res.errmsg = "Failed to find start node " + req.from_node;
			return true;
		}
	}

	NavGraphPath path;

	if (! req.to_node.empty()) {
		path = navgraph->search_path(from.name(), req.to_node);
	} else {
		NavGraphNode close_to_goal = navgraph->closest_node(req.to_pose.x, req.to_pose.y);
		path = navgraph->search_path(from, close_to_goal);
		if (! path.empty()) {
			NavGraphNode free_target("free-target", req.to_pose.x, req.to_pose.y);
			if (std::isfinite(req.to_pose.theta)) {
				free_target.set_property("orientation", (float)req.to_pose.theta);
			}
			path.add_node(free_target, navgraph->cost(path.nodes().back(), free_target));
		}
	}

	// translate path into result
	convert_nodes(path.nodes(), res.path);
	res.cost = path.cost();
	
	res.ok = true;
	return true;
}

bool
NavGraphROSPubThread::svs_get_pwcosts_cb(fawkes_msgs::NavGraphGetPairwiseCosts::Request  &req,
                                         fawkes_msgs::NavGraphGetPairwiseCosts::Response &res)
{
	for (unsigned int i = 0; i < req.nodes.size(); ++i) {
		for (unsigned int j = 0; j < req.nodes.size(); ++j) {
			if (i == j) continue;

			fawkes::NavGraphNode from_node, to_node;
			try {
				from_node = navgraph->node(req.nodes[i]);
				to_node = navgraph->node(req.nodes[j]);
			} catch (fawkes::Exception &e) {
				res.ok = false;
				res.errmsg = "Failed to get path from '" + req.nodes[i] + "' to '" +
					req.nodes[j] + "': " + e.what_no_backtrace();
				res.path_costs.clear();
				return true;					
			}

			fawkes::NavGraphNode start_node, goal_node;
				
			if (from_node.unconnected()) {
				start_node = navgraph->closest_node_to(from_node.name());
				//logger->log_warn(name(), "[F-NavGraph] From node %s is UNCONNECTED, starting instead from %s",
				//                 from_node.name().c_str(), start_node.name().c_str());
			} else {
				start_node = from_node;
			}
			if (to_node.unconnected()) {
				goal_node = navgraph->closest_node_to(to_node.name());
				//logger->log_warn(name(), "[F-NavGraph] To node %s is UNCONNECTED, ending instead at %s",
				//                 to_node.name().c_str(), goal_node.name().c_str());
			} else {
				goal_node = to_node;
			}
			fawkes::NavGraphPath p = navgraph->search_path(start_node, goal_node);
			if (p.empty()) {
				res.ok = false;
				res.errmsg = "Failed to get path from '" + start_node.name() + "' to '" + goal_node.name() + "'";
				res.path_costs.clear();
				return true;
			}
			fawkes_msgs::NavGraphPathCost pc;
			pc.from_node = req.nodes[i];
			pc.to_node = req.nodes[j];
			pc.cost = p.cost();
			if (from_node.unconnected()) {
				pc.cost += navgraph->cost(from_node, start_node);
			}
			if (to_node.unconnected()) {
				pc.cost += navgraph->cost(goal_node, to_node);
			}
			res.path_costs.push_back(pc);
		}
	}

	res.ok = true;
	return true;	
}
