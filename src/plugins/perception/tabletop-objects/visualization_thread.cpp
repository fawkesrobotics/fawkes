
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
#include "cluster_colors.h"

#include <core/threading/mutex_locker.h>
#include <utils/math/angle.h>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#ifdef USE_POSEPUB
#  include <geometry_msgs/PointStamped.h>
#endif
#include <Eigen/Geometry>

extern "C"
{
#ifdef HAVE_QHULL_2011
#  include "libqhull/libqhull.h"
#  include "libqhull/mem.h"
#  include "libqhull/qset.h"
#  include "libqhull/geom.h"
#  include "libqhull/merge.h"
#  include "libqhull/poly.h"
#  include "libqhull/io.h"
#  include "libqhull/stat.h"
#else
#  include "qhull/qhull.h"
#  include "qhull/mem.h"
#  include "qhull/qset.h"
#  include "qhull/geom.h"
#  include "qhull/merge.h"
#  include "qhull/poly.h"
#  include "qhull/io.h"
#  include "qhull/stat.h"
#endif
}

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
    try {
      tf::Stamped<tf::Point>
        centroid(tf::Point(centroids_[i][0], centroids_[i][1], centroids_[i][2]),
                 fawkes::Time(0, 0), frame_id_);
      tf::Stamped<tf::Point> baserel_centroid;
      tf_listener->transform_point("/base_link", centroid, baserel_centroid);

      char *tmp;
      if (asprintf(&tmp, "TObj %zu", i) != -1) {
        // Copy to get memory freed on exception
        std::string id = tmp;
        free(tmp);

        visualization_msgs::Marker text;
        text.header.frame_id = "/base_link";
        text.header.stamp = ros::Time::now();
        text.ns = "tabletop";
        text.id = idnum++;
        text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::Marker::ADD;
        text.pose.position.x = baserel_centroid[0];
        text.pose.position.y = baserel_centroid[1];
        text.pose.position.z = baserel_centroid[2] + 0.13;
        text.pose.orientation.w = 1.;
        text.scale.z = 0.05; // 5cm high
        text.color.r = text.color.g = text.color.b = 1.0f;
        text.color.a = 1.0;
        text.lifetime = ros::Duration(10, 0);
        text.text = id;
        m.markers.push_back(text);
      }

      visualization_msgs::Marker sphere;
      sphere.header.frame_id = "/base_link";
      sphere.header.stamp = ros::Time::now();
      sphere.ns = "tabletop";
      sphere.id = idnum++;
      sphere.type = visualization_msgs::Marker::CYLINDER;
      sphere.action = visualization_msgs::Marker::ADD;
      sphere.pose.position.x = baserel_centroid[0];
      sphere.pose.position.y = baserel_centroid[1];
      sphere.pose.position.z = baserel_centroid[2];
      sphere.pose.orientation.w = 1.;
      sphere.scale.x = sphere.scale.y = 0.08;
      sphere.scale.z = 0.09;
      sphere.color.r = (float)cluster_colors[i][0] / 255.f;
      sphere.color.g = (float)cluster_colors[i][1] / 255.f;
      sphere.color.b = (float)cluster_colors[i][2] / 255.f;
      sphere.color.a = 1.0;
      sphere.lifetime = ros::Duration(10, 0);
      m.markers.push_back(sphere);
    } catch (tf::TransformException &e) {} // ignored
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
  normal.color.r = 0.4;
  normal.color.g = normal.color.b = 0.f;
  normal.color.a = 1.0;
  normal.lifetime = ros::Duration(10, 0);
  m.markers.push_back(normal);

  // Table surrounding polygon
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
  hull.color.r = 0.4;
  hull.color.g = hull.color.b = 0.f;
  hull.color.a = 1.0;
  hull.lifetime = ros::Duration(10, 0);
  m.markers.push_back(hull);

  triangulate_hull();

  if (! table_triangle_vertices_.empty()) {
    visualization_msgs::Marker plane;
    plane.header.frame_id = frame_id_;
    plane.header.stamp = ros::Time::now();
    plane.ns = "tabletop";
    plane.id = idnum++;
    plane.type = visualization_msgs::Marker::TRIANGLE_LIST;
    plane.action = visualization_msgs::Marker::ADD;
    plane.points.resize(table_triangle_vertices_.size());

    for (unsigned int i = 0; i < table_triangle_vertices_.size(); ++i) {
      plane.points[i].x = table_triangle_vertices_[i][0];
      plane.points[i].y = table_triangle_vertices_[i][1];
      plane.points[i].z = table_triangle_vertices_[i][2];
    }
    plane.pose.orientation.w = 1.;
    plane.scale.x = 1.0;
    plane.scale.y = 1.0;
    plane.scale.z = 1.0;
    plane.color.r = ((float)table_color[0] / 255.f) * 0.8;
    plane.color.g = ((float)table_color[1] / 255.f) * 0.8;
    plane.color.b = ((float)table_color[2] / 255.f) * 0.8;
    plane.color.a = 1.0;
    plane.lifetime = ros::Duration(10, 0);
    m.markers.push_back(plane);
  }

  // Frustrum
  visualization_msgs::Marker frustrum;
  frustrum.header.frame_id = frame_id_;
  frustrum.header.stamp = ros::Time::now();
  frustrum.ns = "tabletop";
  frustrum.id = idnum++;
  frustrum.type = visualization_msgs::Marker::LINE_LIST;
  frustrum.action = visualization_msgs::Marker::ADD;
  frustrum.points.resize(8);
  frustrum.points[0].x = frustrum.points[2].x = frustrum.points[4].x = frustrum.points[6].x = 0.;
  frustrum.points[0].y = frustrum.points[2].y = frustrum.points[4].y = frustrum.points[6].y = 0.;
  frustrum.points[0].z = frustrum.points[2].z = frustrum.points[4].z = frustrum.points[6].z = 0.;

  Eigen::Matrix3f upper_right_m;
  upper_right_m =
    Eigen::AngleAxisf(deg2rad(-28.5), Eigen::Vector3f::UnitZ())
    * Eigen::AngleAxisf(deg2rad(-21.5), Eigen::Vector3f::UnitY());
  Eigen::Vector3f upper_right = upper_right_m * Eigen::Vector3f(4,0,0);

  Eigen::Matrix3f upper_left_m;
  upper_left_m =
    Eigen::AngleAxisf(deg2rad(28.5), Eigen::Vector3f::UnitZ())
    * Eigen::AngleAxisf(deg2rad(-21.5), Eigen::Vector3f::UnitY());
  Eigen::Vector3f upper_left = upper_left_m * Eigen::Vector3f(4,0,0);

  Eigen::Matrix3f lower_right_m;
  lower_right_m =
    Eigen::AngleAxisf(deg2rad(-28.5), Eigen::Vector3f::UnitZ())
    * Eigen::AngleAxisf(deg2rad(21.5), Eigen::Vector3f::UnitY());
  Eigen::Vector3f lower_right = lower_right_m * Eigen::Vector3f(2,0,0);

  Eigen::Matrix3f lower_left_m;
  lower_left_m =
    Eigen::AngleAxisf(deg2rad(28.5), Eigen::Vector3f::UnitZ())
    * Eigen::AngleAxisf(deg2rad(21.5), Eigen::Vector3f::UnitY());
  Eigen::Vector3f lower_left = lower_left_m * Eigen::Vector3f(2,0,0);

  frustrum.points[1].x = upper_right[0];
  frustrum.points[1].y = upper_right[1];
  frustrum.points[1].z = upper_right[2];

  frustrum.points[3].x = lower_right[0];
  frustrum.points[3].y = lower_right[1];
  frustrum.points[3].z = lower_right[2];

  frustrum.points[5].x = lower_left[0];
  frustrum.points[5].y = lower_left[1];
  frustrum.points[5].z = lower_left[2];

  frustrum.points[7].x = upper_left[0];
  frustrum.points[7].y = upper_left[1];
  frustrum.points[7].z = upper_left[2];

  frustrum.scale.x = 0.02;
  frustrum.color.r = 1.0;
  frustrum.color.g = frustrum.color.b = 0.f;
  frustrum.color.a = 1.0;
  frustrum.lifetime = ros::Duration(10, 0);
  m.markers.push_back(frustrum);


  visualization_msgs::Marker frustrum_triangles;
  frustrum_triangles.header.frame_id = frame_id_;
  frustrum_triangles.header.stamp = ros::Time::now();
  frustrum_triangles.ns = "tabletop";
  frustrum_triangles.id = idnum++;
  frustrum_triangles.type = visualization_msgs::Marker::TRIANGLE_LIST;
  frustrum_triangles.action = visualization_msgs::Marker::ADD;
  frustrum_triangles.points.resize(6);
  frustrum_triangles.points[0].x = frustrum_triangles.points[3].x = 0.;
  frustrum_triangles.points[0].y = frustrum_triangles.points[3].y = 0.;
  frustrum_triangles.points[0].z = frustrum_triangles.points[3].z = 0.;

  frustrum_triangles.points[1].x = upper_right[0];
  frustrum_triangles.points[1].y = upper_right[1];
  frustrum_triangles.points[1].z = upper_right[2];

  frustrum_triangles.points[2].x = lower_right[0];
  frustrum_triangles.points[2].y = lower_right[1];
  frustrum_triangles.points[2].z = lower_right[2];

  frustrum_triangles.points[4].x = lower_left[0];
  frustrum_triangles.points[4].y = lower_left[1];
  frustrum_triangles.points[4].z = lower_left[2];

  frustrum_triangles.points[5].x = upper_left[0];
  frustrum_triangles.points[5].y = upper_left[1];
  frustrum_triangles.points[5].z = upper_left[2];

  frustrum_triangles.scale.x = 1;
  frustrum_triangles.scale.y = 1;
  frustrum_triangles.scale.z = 1;
  frustrum_triangles.color.r = 1.0;
  frustrum_triangles.color.g = frustrum_triangles.color.b = 0.f;
  frustrum_triangles.color.a = 0.23;
  frustrum_triangles.lifetime = ros::Duration(10, 0);
  m.markers.push_back(frustrum_triangles);

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
                                       V_Vector4f &table_hull_vertices,
                                       V_Vector4f &centroids) throw()
{
  MutexLocker lock(&mutex_);
  frame_id_ = frame_id;
  table_centroid_ = table_centroid;
  normal_ = normal;
  table_hull_vertices_ = table_hull_vertices;
  centroids_ = centroids;
  wakeup();
}


void
TabletopVisualizationThread::triangulate_hull()
{
  // Don't need to, resizing and overwriting them all later
  //table_triangle_vertices_.clear();

  // True if qhull should free points in qh_freeqhull() or reallocation
  boolT ismalloc = True;
  qh DELAUNAY = True;

  int dim = 3;
  char *flags = const_cast<char *>("qhull Qt Pp");;
  FILE *outfile = NULL;
  FILE *errfile = stderr;

  // Array of coordinates for each point
  coordT *points = (coordT *)calloc(table_hull_vertices_.size() * dim, sizeof(coordT));

  for (size_t i = 0; i < table_hull_vertices_.size(); ++i)
  {
    points[i * dim + 0] = (coordT)table_hull_vertices_[i][0];
    points[i * dim + 1] = (coordT)table_hull_vertices_[i][1];
    points[i * dim + 2] = (coordT)table_hull_vertices_[i][2];
  }

  // Compute convex hull
  int exitcode = qh_new_qhull(dim, table_hull_vertices_.size(), points,
                              ismalloc, flags, outfile, errfile);

  if (exitcode != 0) {
    // error, return empty vector
    // Deallocates memory (also the points)
    qh_freeqhull(!qh_ALL);
    int curlong, totlong;
    qh_memfreeshort (&curlong, &totlong);
    return;
  }

  qh_triangulate();

  int num_facets = qh num_facets;

  table_triangle_vertices_.resize(num_facets * dim);
  facetT *facet;
  size_t i = 0;
  FORALLfacets
  {
    vertexT *vertex;
    int vertex_n, vertex_i;
    FOREACHvertex_i_(facet->vertices)
    {
      table_triangle_vertices_[i + vertex_i][0] = vertex->point[0];
      table_triangle_vertices_[i + vertex_i][1] = vertex->point[1];
      table_triangle_vertices_[i + vertex_i][2] = vertex->point[2];
    }

    i += dim;
  }

  // Deallocates memory (also the points)
  qh_freeqhull(!qh_ALL);
  int curlong, totlong;
  qh_memfreeshort(&curlong, &totlong);
}
