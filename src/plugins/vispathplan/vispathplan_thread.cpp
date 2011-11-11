
/***************************************************************************
 *  vispathplan_thread.cpp - Visualization pathplan via rviz
 *
 *  Created: Fri Nov 11 21:25:46 2011
 *  Copyright  2011  Tim Niemueller [www.niemueller.de]
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

#include "vispathplan_thread.h"

#include <utils/graph/rcsoft_map_graph.h>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

using namespace fawkes;

/** @class PathplanVisualizationThread "vispathplan_thread.h"
 * Send Marker messages to rviz to show pathplan info.
 * @author Tim Niemueller
 */

typedef std::multimap<std::string, std::string> ConnMap;

/** Constructor. */
PathplanVisualizationThread::PathplanVisualizationThread()
  : fawkes::Thread("PathplanVisualizationThread", Thread::OPMODE_WAITFORWAKEUP)
{
  set_coalesce_wakeups(true);
}


void
PathplanVisualizationThread::init()
{
  std::string filename = config->get_string("/plugins/vispathplan/pathplan_file");

  RCSoftMapGraph graph(filename);
  std::vector<fawkes::RCSoftMapNode> nodes = graph.nodes();

  vispub_ = rosnode->advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 100, /* latching */ true);
  last_id_num_ = 0;

  std::map<std::string, fawkes::RCSoftMapNode> nodemap;
  for (unsigned int i = 0; i < nodes.size(); ++i) {
    nodemap[nodes[i].name()] = nodes[i];
  }
  ConnMap conns;

  visualization_msgs::MarkerArray m;
  visualization_msgs::Marker lines;
  lines.header.frame_id = "/map";
  lines.header.stamp = ros::Time::now();
  lines.ns = "pathplan";
  lines.id = last_id_num_++;
  lines.type = visualization_msgs::Marker::LINE_LIST;
  lines.action = visualization_msgs::Marker::ADD;
  lines.color.r = 0.5;
  lines.color.g = lines.color.b = 0.f;
  lines.color.a = 1.0;
  lines.scale.x = 0.02;
  lines.lifetime = ros::Duration(0, 0);

  for (size_t i = 0; i < nodes.size(); ++i) {
    // Copy to get memory freed on exception
    logger->log_info(name(), "Publishing node %s", nodes[i].name().c_str());

    visualization_msgs::Marker sphere;
    sphere.header.frame_id = "/map";
    sphere.header.stamp = ros::Time::now();
    sphere.ns = "pathplan";
    sphere.id = last_id_num_++;
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.action = visualization_msgs::Marker::ADD;
    sphere.pose.position.x =  nodes[i].x();
    sphere.pose.position.y = -nodes[i].y();
    sphere.pose.position.z = 0.;
    sphere.pose.orientation.w = 1.;
    sphere.scale.x = 0.05;
    sphere.scale.y = 0.05;
    sphere.scale.z = 0.05;
    sphere.color.r = 1.f;
    sphere.color.g = sphere.color.b = 0.f;
    sphere.color.a = 1.0;
    sphere.lifetime = ros::Duration(0, 0);
    m.markers.push_back(sphere);

    visualization_msgs::Marker text;
    text.header.frame_id = "/map";
    text.header.stamp = ros::Time::now();
    text.ns = "pathplan";
    text.id = last_id_num_++;
    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::Marker::ADD;
    text.pose.position.x =  nodes[i].x();
    text.pose.position.y = -nodes[i].y();
    text.pose.position.z = 0.08;
    text.pose.orientation.w = 1.;
    text.scale.z = 0.1; // 5cm high
    text.color.r = text.color.g = text.color.b = 1.0f;
    text.color.a = 1.0;
    text.lifetime = ros::Duration(0, 0);
    text.text = nodes[i].name();
    m.markers.push_back(text);

    std::vector<std::string> children = nodes[i].children();
    for (unsigned int j = 0; j < children.size(); ++j) {
      std::pair<ConnMap::iterator, ConnMap::iterator>
        ret = conns.equal_range(children[j]);

      ConnMap::value_type v = std::make_pair(children[j], nodes[i].name());
      ConnMap::iterator f = std::find(ret.first, ret.second, v);

      if (f == ret.second) {
        if (nodemap.find(children[j]) != nodemap.end()) {
          geometry_msgs::Point p1;
          p1.x =  nodes[i].x();
          p1.y = -nodes[i].y();
          p1.z = 0.;

          geometry_msgs::Point p2;
          p2.x =  nodemap[children[j]].x();
          p2.y = -nodemap[children[j]].y();
          p2.z = 0.;

          lines.points.push_back(p1);
          lines.points.push_back(p2);

          conns.insert(std::make_pair(nodes[i].name(), children[j]));
        }
      }
    }
  }

  m.markers.push_back(lines);

  vispub_.publish(m);
}

void
PathplanVisualizationThread::finalize()
{
  visualization_msgs::MarkerArray m;

  for (size_t i = 0; i < last_id_num_; ++i) {
    visualization_msgs::Marker delop;
    delop.header.frame_id = "/map";
    delop.header.stamp = ros::Time::now();
    delop.ns = "pathplan";
    delop.id = i;
    delop.action = visualization_msgs::Marker::DELETE;
    m.markers.push_back(delop);
  }
  vispub_.publish(m);
  usleep(500000); // needs some time to actually send
  vispub_.shutdown();
}


void
PathplanVisualizationThread::loop()
{
}

