
/***************************************************************************
 *  rcsoft_map_graph.ecl - Access map files
 *
 *  Created: Mon Mar 21 18:34:47 2011
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

%% module definition
:- module(rcsoft_map_graph).
:- export map_graph_load/1.
:- export is_map_graph_loaded/0.
:- export map_graph_get_node_coords/3.
:- export map_graph_get_node_coords/4.
:- export map_graph_get_nodes/1.
:- export map_graph_search_nodes/2.
:- export map_graph_get_closest_node/3.
:- export map_graph_get_children/2.

%% definition of external predicates
:- external(map_graph_load/1, p_map_graph_load).
:- external(is_map_graph_loaded/0, p_is_map_graph_loaded).
:- external(map_graph_get_node_coords/3, p_map_graph_get_node_coords3).
:- external(map_graph_get_node_coords/4, p_map_graph_get_node_coords4).
:- external(map_graph_get_nodes/1, p_map_graph_get_nodes).
:- external(map_graph_search_nodes/2, p_map_graph_search_nodes).
:- external(map_graph_get_closest_node/3, p_map_graph_get_closest_node).
:- external(map_graph_get_children/2, p_map_graph_get_children).
