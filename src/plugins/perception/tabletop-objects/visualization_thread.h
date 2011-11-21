
/***************************************************************************
 *  visualization_thread.h - Visualization via rviz
 *
 *  Created: Fri Nov 11 00:11:23 2011
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

#ifndef __PLUGINS_PERCEPTION_TABLETOP_OBJECTS_VISUALIZATION_THREAD_H_
#define __PLUGINS_PERCEPTION_TABLETOP_OBJECTS_VISUALIZATION_THREAD_H_

#ifndef HAVE_VISUAL_DEBUGGING
#  error TabletopVisualizationThread was disabled by build flags
#endif

#include "visualization_thread_base.h"

#include <core/threading/thread.h>
#include <core/threading/mutex.h>
#include <aspect/tf.h>
#include <plugins/ros/aspect/ros.h>

namespace ros {
  class Publisher;
}

class TabletopVisualizationThread
: public TabletopVisualizationThreadBase,
  public fawkes::Thread,
  public fawkes::TransformAspect,
  public fawkes::ROSAspect
{
 public:
  TabletopVisualizationThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

  virtual void visualize(const std::string &frame_id,
                         Eigen::Vector4f &table_centroid,
                         Eigen::Vector4f &normal,
                         V_Vector4f &table_hull_vertices,
                         V_Vector4f &centroids) throw();

 private:
  void triangulate_hull();

 private:
  fawkes::Mutex mutex_;
  std::string frame_id_;
  Eigen::Vector4f table_centroid_;
  Eigen::Vector4f normal_;
  V_Vector4f table_hull_vertices_;
  V_Vector4f table_triangle_vertices_;
  V_Vector4f centroids_;
  ros::Publisher *vispub_;
#ifdef USE_POSEPUB
  ros::Publisher *posepub_;
#endif
  size_t last_id_num_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


#endif
