
/***************************************************************************
 *  rcsoft_map_graph.cpp - Access the annotated map.
 *
 *  Created: Mon Mar 21 17:23:57 2011
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

#include "rcsoft_map_graph.h"

#include <utils/graph/rcsoft_map_graph.h>
#include <core/exception.h>
#include <eclipseclass.h>
#include <cstdio>

using namespace std;
using namespace fawkes;

class RCSoftMapGraphWrapper
{
public:
  RCSoftMapGraphWrapper() : m_map_graph(0) {};
  ~RCSoftMapGraphWrapper()
  {
    delete m_map_graph;
  }

  void load( const char* file )
  {
    m_map_graph = new RCSoftMapGraph( string(file) );
  }

  bool loaded()
  {
    return m_map_graph ? true : false;
  }

  RCSoftMapGraph* map_graph()
  {
    return m_map_graph;
  }

private:
  RCSoftMapGraph* m_map_graph;
};

RCSoftMapGraphWrapper g_map_graph;

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
p_map_graph_get_node_coords()
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

  RCSoftMapNode node = g_map_graph.map_graph()->node( string(nodename) );
  
  if ( EC_succeed != EC_arg( 2 ).unify( EC_word( (double) node.x() ) ) )
  {
    printf( "p_map_get_node(): could not bind return value\n" );
    return EC_fail;
  }

  if ( EC_succeed != EC_arg( 3 ).unify( EC_word( (double) node.y() ) ) )
  {
    printf( "p_map_get_node(): could not bind return value\n" );
    return EC_fail;
  }

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

  vector< RCSoftMapNode > nodes = g_map_graph.map_graph()->nodes();
  EC_word tail = nil();

  for ( vector< RCSoftMapNode >::iterator nit = nodes.begin();
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

  vector< RCSoftMapNode > nodes = g_map_graph.map_graph()->search_nodes( string(property) );
  EC_word tail = nil();

  for ( vector< RCSoftMapNode >::iterator nit = nodes.begin();
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
