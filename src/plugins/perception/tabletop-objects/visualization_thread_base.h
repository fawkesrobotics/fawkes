
/***************************************************************************
 *  visualization_thread_base.h - Visualization base class
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

#ifndef __PLUGINS_PERCEPTION_TABLETOP_OBJECTS_VISUALIZATION_BASE_H_
#define __PLUGINS_PERCEPTION_TABLETOP_OBJECTS_VISUALIZATION_BASE_H_

#ifndef HAVE_VISUAL_DEBUGGING
#  error TabletopVisualizationThread was disabled by build flags
#endif

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <utils/time/time.h>

class TabletopVisualizationThreadBase
{
 public:
  /** Aligned vector of vectors/points. */
  typedef std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > V_Vector4f;
  // Eigen::aligned_allocator<Eigen::Vector4f> 

  virtual ~TabletopVisualizationThreadBase();

  virtual void visualize(const std::string &frame_id,
                         Eigen::Vector4f &table_centroid,
                         Eigen::Vector4f &normal,
                         V_Vector4f &table_hull_vertices,
                         V_Vector4f &table_model_vertices,
                         V_Vector4f &good_table_hull_edges,
                         V_Vector4f &centroids,
                         V_Vector4f &cylinder_params,
                         std::vector<double> &obj_confidence,
                         std::vector<signed int>& best_obj_guess) throw() = 0;
};

#endif
