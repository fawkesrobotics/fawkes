
/***************************************************************************
 *  rcsoft_map_graph.h - Access the annotated map.
 *
 *  Created: Mon Mar 21 17:19:41 2011
 *  Copyright  2011  Daniel Beck
 *
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

#ifndef __ECLIPSE_EXTERNALS_RCSOFT_MAP_GRAPH_H
#define __ECLIPSE_EXTERNALS_RCSOFT_MAP_GRAPH_H

extern "C" int p_map_graph_load();
extern "C" int p_is_map_graph_loaded();
extern "C" int p_map_graph_get_node_coords3();
extern "C" int p_map_graph_get_node_coords4();
extern "C" int p_map_graph_get_nodes();
extern "C" int p_map_graph_get_closest_node();
extern "C" int p_map_graph_search_nodes();
extern "C" int p_map_graph_get_children();

#endif /* __ECLIPSE_EXTERNALS_BLACKBOARD_H_ */

