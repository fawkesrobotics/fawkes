
/***************************************************************************
 *  rcsoft_map_graph.cpp - Access the annotated map.
 *
 *  Created: Mon Mar 21 17:23:57 2011
 *  Copyright  2011  Daniel Beck
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

#include "rcsoft_map_graph.h"

#include <utils/graph/topological_map_graph.h>
#include <core/exception.h>
#include <eclipseclass.h>
#include <cstdio>
#include <cstring>
#include <cstdlib>

/** @class fawkes::EclExternalRCSoftMapGraph
 * Wrapper class for using the RCSoftMapGraph in the implementation of
 * the external predicates.
 * @author Daniel Beck
 */
namespace fawkes
{
class EclExternalRCSoftMapGraph
{
public:
  /** Cosntructor. */
  EclExternalRCSoftMapGraph() : m_map_graph(0) {}
  /** Destructor. */
  ~EclExternalRCSoftMapGraph() { delete m_map_graph; }

  /** Load map file.
   * @param file the map file
   */
  void load( const char* file )
  {
    m_map_graph = new TopologicalMapGraph( std::string(file) );
  }

  /** Query status.
   * @return true if a map file is loaded; false otherwise
   */
  bool loaded()
  {
    return m_map_graph ? true : false;
  }

  /** Access the TopologicalMapGraph instance.
   * @return the TopologicalMapGraph instance
   */
  TopologicalMapGraph* map_graph()
  {
    return m_map_graph;
  }

private:
  TopologicalMapGraph* m_map_graph;

};

}

using namespace std;
using namespace fawkes;

EclExternalRCSoftMapGraph g_map_graph;

int
p_map_graph_load()
{
  if ( g_map_graph.loaded() )
  {
    printf( "p_map_load(): map already loaded\n" );
    return EC_fail;
  }

  char* mapfile;
  if ( EC_succeed != EC_arg( 1 ).is_string( &mapfile) )
  {
    printf( "p_map_load(): first argument is not a string\n" );
    return EC_fail;
  }

  try
  {
    g_map_graph.load( mapfile );
  }
  catch ( Exception& e )
  {
    e.print_trace();
    return EC_fail;
  }

  return EC_succeed;
}

int
p_map_graph_get_node_coords3()
{
  if ( !g_map_graph.loaded() )
  {
    printf( "p_map_get_node(): map file not loaded\n" );
    return EC_fail;
  }

  char* nodename;
  if ( EC_succeed != EC_arg( 1 ).is_string( &nodename ) )
  {
    printf( "p_map_get_node(): first argument is not a string\n" );
    return EC_fail;
  }

  TopologicalMapNode node = g_map_graph.map_graph()->node( string(nodename) );
  
  // x-coordinate
  if ( EC_succeed != EC_arg( 2 ).unify( EC_word( (double) node.x() ) ) )
  {
    printf( "p_map_get_node(): could not bind return value\n" );
    return EC_fail;
  }

  // y-coordinate
  if ( EC_succeed != EC_arg( 3 ).unify( EC_word( (double) node.y() ) ) )
  {
    printf( "p_map_get_node(): could not bind return value\n" );
    return EC_fail;
  }

  return EC_succeed;
}

int
p_map_graph_get_node_coords4()
{
  if ( EC_succeed != p_map_graph_get_node_coords3() )
  { return EC_fail; }

  char* nodename;
  if ( EC_succeed != EC_arg( 1 ).is_string( &nodename ) )
  {
    printf( "p_map_get_node(): first argument is not a string\n" );
    return EC_fail;
  }

  TopologicalMapNode node = g_map_graph.map_graph()->node( string(nodename) );

  // check for orientation property
  int result = EC_succeed;
  if (node.has_property("orientation")) {
    double ori = node.property_as_float("orientation");
    result = EC_arg( 4 ).unify( EC_word( ori ) );
  } else {
    result = EC_arg( 4 ).unify( EC_atom( (char*) "false" ) );
  }

  if ( EC_succeed != result)
  { return EC_fail; }

  return EC_succeed;
}


int
p_map_graph_get_nodes()
{
  if ( !g_map_graph.loaded() )
  {
    printf( "p_map_get_nodes(): map file not loaded\n" );
    return EC_fail;
  }

  vector< TopologicalMapNode > nodes = g_map_graph.map_graph()->nodes();
  EC_word tail = nil();

  for ( vector< TopologicalMapNode >::iterator nit = nodes.begin();
	nit != nodes.end();
	++nit )
  {
    EC_word n = list( nit->name().c_str(),
		      list( (double) nit->x(),
			    list( (double) nit->y(), nil() ) ) );
    tail = list( n, tail );
  }

  if ( EC_succeed != EC_arg( 1 ).unify( tail ) )
  {
    printf( "p_map_get_nodes(): could not bind return value\n" );
    return EC_fail;
  }

  return EC_succeed;
}

int
p_map_graph_get_closest_node()
{
  if ( !g_map_graph.loaded() )
  {
    printf( "p_map_search_nodes(): map file not loaded\n" );
    return EC_fail;
  }

  double x;
  double y;
  if ( EC_succeed != EC_arg( 1 ).is_double( &x ) )
  {
    printf( "p_map_graph_get_closest_node(): no x-coordinate given\n" );
    return EC_fail;
  }

  if ( EC_succeed != EC_arg( 2 ).is_double( &y ) )
  {
    printf( "p_map_graph_get_closest_node(): no y-coordinate given\n" );
    return EC_fail;
  }

  TopologicalMapNode node = g_map_graph.map_graph()->closest_node( (float) x,
							      (float) y,
							      "" );

  if ( EC_succeed != EC_arg( 3 ).unify( EC_word( node.name().c_str() ) ) )
  {
    printf( "p_map_graph_get_closest_node(): could not bind return value\n" );
    return EC_fail;
  }

  return EC_succeed;
}

int
p_map_graph_search_nodes()
{
  if ( !g_map_graph.loaded() )
  {
    printf( "p_map_search_nodes(): map file not loaded\n" );
    return EC_fail;
  }

  char* property;
  if ( EC_succeed != EC_arg( 1 ).is_string( &property ) )
  {
    printf( "p_map_search_nodes(): no property given\n" );
    return EC_fail;
  }

  vector< TopologicalMapNode > nodes = g_map_graph.map_graph()->search_nodes( string(property) );
  EC_word tail = nil();

  for ( vector< TopologicalMapNode >::iterator nit = nodes.begin();
	nit != nodes.end();
	++nit )
  {
    EC_word n = list( nit->name().c_str(),
		      list( (double) nit->x(),
			    list( (double) nit->y(), nil() ) ) );
    tail = list( n, tail );
  }

  if ( EC_succeed != EC_arg( 1 ).unify( tail ) )
  {
    printf( "p_map_search_nodes(): could not bind return value\n" );
    return EC_fail;
  }

  return EC_succeed;
}

int
p_map_graph_get_children()
{
  if ( !g_map_graph.loaded() )
  {
    printf( "p_map_graph_get_children(): no map file loaded\n" );
    return EC_fail;
  }

  char* nodename;
  if ( EC_succeed != EC_arg( 1 ).is_string( &nodename ) )
  {
    printf( "p_map_graph_get_children(): no node name given\n" );
    return EC_fail;
  }

  TopologicalMapNode node = g_map_graph.map_graph()->node( nodename );
  vector< string > children = node.reachable_nodes();
  EC_word tail = nil();
  for ( vector< string >::iterator nit = children.begin();
	nit != children.end();
	++nit )
  {
    tail = list( EC_word( (*nit).c_str() ), tail );
  }

  if ( EC_succeed != EC_arg( 2 ).unify( tail ) )
  {
    printf( "p_map_graph_get_children(): cannot bind return value\n" );
    return EC_fail;
  }

  return EC_succeed;
}
