
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
#include <utils/time/time.h>

class TabletopVisualizationThreadBase
{
 public:
  virtual ~TabletopVisualizationThreadBase();

  virtual void visualize(const std::string &frame_id,
                         Eigen::Vector4f &table_centroid,
                         Eigen::Vector4f &normal,
                         std::vector<Eigen::Vector4f> &table_hull_vertices,
                         std::vector<Eigen::Vector4f> &centroids) throw() = 0;
};

#endif
