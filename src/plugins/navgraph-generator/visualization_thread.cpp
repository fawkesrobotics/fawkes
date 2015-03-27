
/***************************************************************************
 *  visualization_thread.cpp - Visualization for navgraph-generator via rviz
 *
 *  Created: Fri Mar 27 12:12:17 2015
 *  Copyright  2011-2012  Tim Niemueller [www.niemueller.de]
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

#include "visualization_thread.h"

#include <ros/ros.h>

using namespace fawkes;

/** @class NavGraphGeneratorVisualizationThread "visualization_thread.h"
 * Send Marker messages to rviz to show navgraph-generator info.
 * @author Tim Niemueller
 */

/** Constructor. */
NavGraphGeneratorVisualizationThread::NavGraphGeneratorVisualizationThread()
  : fawkes::Thread("NavGraphGeneratorVisualizationThread", Thread::OPMODE_WAITFORWAKEUP)
{
  set_coalesce_wakeups(true);
}


void
NavGraphGeneratorVisualizationThread::init()
{
  cfg_global_frame_  = config->get_string("/frames/fixed");

  vispub_ =
    rosnode->advertise<visualization_msgs::MarkerArray>
    ("visualization_marker_array", 100, /* latching */ true);

  last_id_num_ = 0;
}

void
NavGraphGeneratorVisualizationThread::finalize()
{
  visualization_msgs::MarkerArray m;

  for (size_t i = 0; i < last_id_num_; ++i) {
    visualization_msgs::Marker delop;
    delop.header.frame_id = "/map";
    delop.header.stamp = ros::Time::now();
    delop.ns = "navgraph_generator";
    delop.id = i;
    delop.action = visualization_msgs::Marker::DELETE;
    m.markers.push_back(delop);
  }
  vispub_.publish(m);
  usleep(500000); // needs some time to actually send
  vispub_.shutdown();
}


void
NavGraphGeneratorVisualizationThread::loop()
{
  visualization_msgs::MarkerArray m;
  unsigned int idnum = 0;

  for (auto &o : obstacles_) {
    visualization_msgs::Marker text;
    text.header.frame_id = cfg_global_frame_;
    text.header.stamp = ros::Time::now();
    text.ns = "navgraph_generator";
    text.id = idnum++;
    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::Marker::ADD;
    text.pose.position.x = o.second.x;
    text.pose.position.y = o.second.y;
    text.pose.position.z = .15;
    text.pose.orientation.w = 1.;
    text.scale.z = 0.15;
    text.color.r = text.color.g = text.color.b = 1.0f;
    text.color.a = 1.0;
    text.lifetime = ros::Duration(0, 0);
    text.text = o.first;
    m.markers.push_back(text);

    visualization_msgs::Marker sphere;
    sphere.header.frame_id = cfg_global_frame_;
    sphere.header.stamp = ros::Time::now();
    sphere.ns = "navgraph_generator";
    sphere.id = idnum++;
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.action = visualization_msgs::Marker::ADD;
    sphere.pose.position.x = o.second.x;
    sphere.pose.position.y = o.second.y;
    sphere.pose.position.z = 0.05;
    sphere.pose.orientation.w = 1.;
    sphere.scale.x = 0.05;
    sphere.scale.y = 0.05;
    sphere.scale.z = 0.05;
    sphere.color.r = 1.0;
    sphere.color.g = sphere.color.b = 0.;
    sphere.color.a = 1.0;
    sphere.lifetime = ros::Duration(0, 0);
    m.markers.push_back(sphere);
  }      

  for (auto &o : map_obstacles_) {
    visualization_msgs::Marker text;
    text.header.frame_id = cfg_global_frame_;
    text.header.stamp = ros::Time::now();
    text.ns = "navgraph_generator";
    text.id = idnum++;
    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::Marker::ADD;
    text.pose.position.x = o.second.x;
    text.pose.position.y = o.second.y;
    text.pose.position.z = .15;
    text.pose.orientation.w = 1.;
    text.scale.z = 0.15;
    text.color.r = text.color.g = text.color.b = 1.0f;
    text.color.a = 1.0;
    text.lifetime = ros::Duration(0, 0);
    text.text = o.first;
    m.markers.push_back(text);

    visualization_msgs::Marker sphere;
    sphere.header.frame_id = cfg_global_frame_;
    sphere.header.stamp = ros::Time::now();
    sphere.ns = "navgraph_generator";
    sphere.id = idnum++;
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.action = visualization_msgs::Marker::ADD;
    sphere.pose.position.x = o.second.x;
    sphere.pose.position.y = o.second.y;
    sphere.pose.position.z = 0.05;
    sphere.pose.orientation.w = 1.;
    sphere.scale.x = 0.05;
    sphere.scale.y = 0.05;
    sphere.scale.z = 0.05;
    sphere.color.r = sphere.color.g = 1.0;
    sphere.color.b = 0.;
    sphere.color.a = 1.0;
    sphere.lifetime = ros::Duration(0, 0);
    m.markers.push_back(sphere);
  }      

  for (size_t i = idnum; i < last_id_num_; ++i) {
    visualization_msgs::Marker delop;
    delop.header.frame_id = cfg_global_frame_;
    delop.header.stamp = ros::Time::now();
    delop.ns = "navgraph_generator";
    delop.id = i;
    delop.action = visualization_msgs::Marker::DELETE;
    m.markers.push_back(delop);
  }
  last_id_num_ = idnum;

  vispub_.publish(m);
}


/** Trigger publishing of visualization.
 * @param obstacles obstacles used for graph generation
 * @param map_obstacles obstacles generated from map
 * @param pois points of interest
 */
void
NavGraphGeneratorVisualizationThread::publish(const NavGraphGeneratorThread::ObstacleMap &obstacles,
					      const NavGraphGeneratorThread::ObstacleMap &map_obstacles,
					      const NavGraphGeneratorThread::PoiMap      &pois)
{
  obstacles_     = obstacles;
  map_obstacles_ = map_obstacles;
  pois_          = pois;

  wakeup();
}
