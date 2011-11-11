
/***************************************************************************
 *  visualization_thread.cpp - Visualization via rviz
 *
 *  Created: Fri Nov 11 00:20:45 2011
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

#include "visualization_thread.h"

#include <core/threading/mutex_locker.h>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#ifdef USE_POSEPUB
#  include <geometry_msgs/PointStamped.h>
#endif

using namespace fawkes;

/** @class TabletopVisualizationThread "visualization_thread.h"
 * Send Marker messages to rviz.
 * This class takes input from the table top object detection thread and
 * publishes according marker messages for visualization in rviz.
 * @author Tim Niemueller
 */

/** Constructor. */
TabletopVisualizationThread::TabletopVisualizationThread()
  : fawkes::Thread("TabletopVisualizationThread", Thread::OPMODE_WAITFORWAKEUP)
{
  set_coalesce_wakeups(true);
}


void
TabletopVisualizationThread::init()
{
  vispub_ = new ros::Publisher();
  *vispub_ = rosnode->advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 100);
#ifdef USE_POSEPUB
  posepub_ = new ros::Publisher();
  *posepub_ = rosnode->advertise<geometry_msgs::PointStamped>("table_point", 10);
#endif
  last_id_num_ = 0;
}

void
TabletopVisualizationThread::finalize()
{
  visualization_msgs::MarkerArray m;

  for (size_t i = 0; i < last_id_num_; ++i) {
    visualization_msgs::Marker delop;
    delop.header.frame_id = frame_id_;
    delop.header.stamp = ros::Time::now();
    delop.ns = "tabletop";
    delop.id = i;
    delop.action = visualization_msgs::Marker::DELETE;
    m.markers.push_back(delop);
  }
  vispub_->publish(m);


  vispub_->shutdown();
  delete vispub_;
#ifdef USE_POSEPUB
  posepub_->shutdown();
  delete posepub_;
#endif
}


void
TabletopVisualizationThread::loop()
{
  MutexLocker lock(&mutex_);
  visualization_msgs::MarkerArray m;

  unsigned int idnum = 0;

  for (size_t i = 0; i < centroids_.size(); ++i) {
    char *tmp;
    if (asprintf(&tmp, "TObj %zu", i) != -1) {
      // Copy to get memory freed on exception
      std::string id = tmp;
      free(tmp);

      visualization_msgs::Marker text;
      text.header.frame_id = frame_id_;
      text.header.stamp = ros::Time::now();
      text.ns = "tabletop";
      text.id = idnum++;
      text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::Marker::ADD;
      text.pose.position.x = centroids_[i][0];
      text.pose.position.y = centroids_[i][1];
      text.pose.position.z = centroids_[i][2];
      text.pose.orientation.w = 1.;
      text.scale.z = 0.05; // 5cm high
      text.color.r = text.color.g = text.color.b = 1.0f;
      text.color.a = 1.0;
      text.lifetime = ros::Duration(10, 0);
      text.text = id;
      m.markers.push_back(text);
    }
  }

  Eigen::Vector4f normal_end = (table_centroid_ + (normal_ * -0.15));

  visualization_msgs::Marker normal;
  normal.header.frame_id = frame_id_;
  normal.header.stamp = ros::Time::now();
  normal.ns = "tabletop";
  normal.id = idnum++;
  normal.type = visualization_msgs::Marker::ARROW;
  normal.action = visualization_msgs::Marker::ADD;
  normal.points.resize(2);
  normal.points[0].x = table_centroid_[0];
  normal.points[0].y = table_centroid_[1];
  normal.points[0].z = table_centroid_[2];
  normal.points[1].x = normal_end[0];
  normal.points[1].y = normal_end[1];
  normal.points[1].z = normal_end[2];
  normal.scale.x = 0.02;
  normal.scale.y = 0.04;
  normal.color.r = 1.0;
  normal.color.g = normal.color.b = 0.f;
  normal.color.a = 1.0;
  normal.lifetime = ros::Duration(10, 0);
  m.markers.push_back(normal);

  visualization_msgs::Marker hull;
  hull.header.frame_id = frame_id_;
  hull.header.stamp = ros::Time::now();
  hull.ns = "tabletop";
  hull.id = idnum++;
  hull.type = visualization_msgs::Marker::LINE_STRIP;
  hull.action = visualization_msgs::Marker::ADD;
  hull.points.resize(table_hull_vertices_.size() + 1);
  for (size_t i = 0; i < table_hull_vertices_.size(); ++i) {
    hull.points[i].x = table_hull_vertices_[i][0];
    hull.points[i].y = table_hull_vertices_[i][1];
    hull.points[i].z = table_hull_vertices_[i][2];
  }
  hull.points[table_hull_vertices_.size()].x = table_hull_vertices_[0][0];
  hull.points[table_hull_vertices_.size()].y = table_hull_vertices_[0][1];
  hull.points[table_hull_vertices_.size()].z = table_hull_vertices_[0][2];
  hull.scale.x = 0.01; // 5cm high
  hull.color.r = 1.0;
  hull.color.g = hull.color.b = 0.f;
  hull.color.a = 1.0;
  hull.lifetime = ros::Duration(10, 0);
  m.markers.push_back(hull);

  for (size_t i = idnum; i < last_id_num_; ++i) {
    visualization_msgs::Marker delop;
    delop.header.frame_id = frame_id_;
    delop.header.stamp = ros::Time::now();
    delop.ns = "tabletop";
    delop.id = i;
    delop.action = visualization_msgs::Marker::DELETE;
    m.markers.push_back(delop);
  }
  last_id_num_ = idnum;

  vispub_->publish(m);

#ifdef USE_POSEPUB
  geometry_msgs::PointStamped p;
  p.header.frame_id = frame_id_;
  p.header.stamp = ros::Time::now();
  p.point.x = table_centroid_[0];
  p.point.y = table_centroid_[1];
  p.point.z = table_centroid_[2];
  posepub_->publish(p);
#endif
}


void
TabletopVisualizationThread::visualize(const std::string &frame_id,
                                       Eigen::Vector4f &table_centroid,
                                       Eigen::Vector4f &normal,
                                       std::vector<Eigen::Vector4f> &table_hull_vertices,
                                       std::vector<Eigen::Vector4f> &centroids) throw()
{
  MutexLocker lock(&mutex_);
  frame_id_ = frame_id;
  table_centroid_ = table_centroid;
  normal_ = normal;
  table_hull_vertices_ = table_hull_vertices;
  centroids_ = centroids;
  wakeup();
}
