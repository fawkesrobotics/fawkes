
/***************************************************************************
 *  visualization_thread.h - Visualization
 *
 *  Created: Fri Nov 11 00:53:07 2011
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

#include "visualization_thread_base.h"

/** @class TabletopVisualizationThreadBase "visualization_thread_base.h"
 * Base class for virtualization thread.
 * This is only required to create a level of indirection to cope
 * with the re-defined msgs problem in PCL.
 *
 * @fn void TabletopVisualizationThreadBase::visualize(std::string &frame_id, Eigen::Vector4f &table_centroid, Eigen::Vector4f &normal, std::vector<Eigen::Vector4f> &table_hull_vertices, V_Vector4f &table_model_vertices, V_Vector4f &good_table_hull_edges, std::vector<Eigen::Vector4f> &centroids) throw()
 * Visualize the given data.
 * @param frame_id reference frame ID
 * @param table_centroid centroid of table
 * @param normal normal vector of table
 * @param table_hull_vertices points of the table hull
 * @param table_model_vertices points of the fitted table model
 * @param good_table_hull_edges "good" egdes in table hull, i.e. edges that have
 * been considered for determining the table orientation
 * @param centroids object cluster centroids
 * @param cylinder_params The result of the cylinder fitting of the objects
 * @param obj_confidence The fitting confidences
 * @param best_obj_guess The best guesses of the objects
 */

/** Virtual empty destructor. */
TabletopVisualizationThreadBase::~TabletopVisualizationThreadBase()
{
}

