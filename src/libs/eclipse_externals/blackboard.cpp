
/***************************************************************************
 *  blackboard.h - External predicates to remotely access the Fawkes
 *                 blackboard
 *
 *  Created: Wed Mar 09 17:10:54 2011
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

#include "blackboard.h"

#include <blackboard/remote.h>

#include <eclipseclass.h>

#include <vector>

#include <cstdio>
#include <cstring>
#include <cstdlib>

using namespace std;
using namespace fawkes;

BlackBoard* g_blackboard = 0;
vector< Interface* > g_interfaces;

int
p_connect_to_blackboard()
{
  if ( g_blackboard )
 {
   printf( "connect_to_blackboard/2: already connected\n" );
   return EC_fail;
 }
    
  // get hostname
  char* hostname;

  if ( EC_succeed != EC_arg( 1 ).is_string( &hostname ) )
  {
    printf( "bb_connect/2: first argument is not a string\n" );
    return EC_fail;
  }

try
 {
   printf( "bb_connect/2: connecting to host %s\n", hostname );
   g_blackboard = new RemoteBlackBoard( hostname, 1910 );
 }
 catch ( Exception& e )
 {
   e.print_trace();
   return EC_fail;
 }

 return EC_succeed;
}


int
p_disconnect_from_blackboard()
{
  if ( !g_blackboard )
  {
    printf( "bb_disconnect/2: not connected\n" );
    return EC_fail;
  }

  delete g_blackboard;
  g_blackboard = 0;

  return EC_succeed;
}


int
p_is_alive()
{
  if ( !g_blackboard )
  {
    printf( "bb_alive/0: not connected\n" );
    return EC_fail;
  }

  if ( g_blackboard->is_alive() )
  { return EC_succeed; }
  else
  { return EC_fail; }
}


int
p_open_interface()
{
  if ( !g_blackboard )
  {
    printf("p_open_interface(): not connected\n" );
    return EC_fail;
  }

  EC_atom mode;
  char* interface_type;
  char* interface_id;

  if ( EC_succeed != EC_arg( 1 ).is_atom( &mode) )
  {
    printf( "p_open_for_reading(): no mode given\n" );
    return EC_fail;
  }

  if ( EC_succeed != EC_arg( 2 ).is_string( &interface_type ) )
  {
    printf( "p_open_for_reading(): no type given\n" );
    return EC_fail;
  }

  if ( EC_succeed != EC_arg ( 3 ).is_string( &interface_id ) )
  {
    printf( "bb_open_for_reading/2: no id given\n" );
    return EC_fail;
  }

  try
  {
    Interface* iface;

    if ( 0 == strcmp( "w", mode.name() ) )
    { iface = g_blackboard->open_for_writing( interface_type, interface_id ); }
    else
    { iface = g_blackboard->open_for_reading( interface_type, interface_id ); }

    g_interfaces.push_back( iface );
  }
  catch (Exception& e)
  {
    e.print_trace();
    return EC_fail;
  }

  return EC_succeed;
}


int
p_close_interface()
{
  if ( !g_blackboard )
  {
    printf("bb_close_interface/2: not connected\n" );
    return EC_fail;
  }

  // char* interface_type;
  char* interface_id;

  // if ( EC_succeed != EC_arg( 1 ).is_string( &interface_type ) )
  // {
  //   printf( "bb_close_interface/2: no type given\n" );
  //   return EC_fail;
  // }

  if ( EC_succeed != EC_arg( 1 ).is_string( &interface_id ) )
  {
    printf( "bb_close_interface/2: no id given\n" );
    return EC_fail;
  }

  bool iface_found = false;

  for ( vector< Interface* >::iterator it = g_interfaces.begin();
	it != g_interfaces.end();
	++it )
  {
    // if ( (*it)->oftype( interface_type ) &&
    // 	 0 == strcmp( (*it)->id(), interface_id ) )
    if ( 0 == strcmp( (*it)->id(), interface_id ) )

    {
      iface_found = true;
      g_blackboard->close( *it );
      break;
    }
  }

  if ( iface_found )
  { return EC_succeed; }
  else
  { return EC_fail; }
}

int
p_read_interfaces()
{
  for ( vector< Interface* >::iterator it = g_interfaces.begin();
	it != g_interfaces.end();
	++it )
  {
    (*it)->read();
  }

  return EC_succeed;
}

int
p_write_interfaces()
{
  for ( vector< Interface* >::iterator it = g_interfaces.begin();
	it != g_interfaces.end();
	++it )
  {
    if ( (*it)->is_writer() )
    { (*it)->write(); }
  }

  return EC_succeed;
}

int
p_read_from_interface()
{
  char* interface_id;
  char* field;

  if ( EC_succeed != EC_arg( 1 ).is_string( &interface_id ) )
  {
    printf( "bb_read_interface/3: no interface id given\n" );
    return EC_fail;
  }

  if ( EC_succeed != EC_arg( 2 ).is_string( &field ) )
  {
    printf( "bb_read_interface/3: no field given\n" );
    return EC_fail;
  }

  vector< Interface* >::iterator it;
  for ( it = g_interfaces.begin();
	it != g_interfaces.end();
	++it )
  {
    if ( 0 == strcmp( interface_id, (*it)->id() ) )
    {
      InterfaceFieldIterator fit;
      for ( fit = (*it)->fields();
	    fit != (*it)->fields_end();
	    ++fit )
      {
	if ( 0 == strcmp( field, fit.get_name() ) )
	{
	  switch ( fit.get_type() ) {
	  case IFT_BOOL:
	    if ( fit.get_bool() )
	    {
	      if ( EC_succeed != EC_arg( 3 ).unify( EC_atom( (char*) "true" ) ) )
	      {
		printf( "p_read_from_interface(): could not bind return value\n" );
		return EC_fail;
	      }
	    }
	    else
	    {
	      if ( EC_succeed != EC_arg( 3 ).unify( EC_atom( (char*) "false" ) ) )
	      {
		printf( "p_read_from_interface(): could not bind return value\n" );
		return EC_fail;
	      }
	    }
	     
	    break;
	    
	  case IFT_INT8:
	    if ( EC_succeed != EC_arg( 3 ).unify( EC_word( (long) fit.get_int8() ) ) )
	    {
	      printf( "p_read_from_interface(): could not bind return value\n" );
	      return EC_fail;
	    }
	    
	    break;
	    
	  case IFT_UINT8:
	    if ( EC_succeed != EC_arg( 3 ).unify( EC_word( (long) fit.get_uint8() ) ) )
	    {
	      printf( "p_read_from_interface(): could not bind return value\n" );
	      return EC_fail;
	    }
	    
	    break;
	    
	  case IFT_INT16:
	    if ( EC_succeed != EC_arg( 3 ).unify( EC_word( (long) fit.get_int16() ) ) )
	    {
	      printf( "p_read_from_interface(): could not bind return value\n" );
	      return EC_fail;
	    }
	    
	    break;
	    
	  case IFT_UINT16:
	    if ( EC_succeed != EC_arg( 3 ).unify( EC_word( (long) fit.get_uint16() ) ) )
	    {
	      printf( "p_read_from_interface(): could not bind return value\n" );
	      return EC_fail;
	    }
	    
	    break;
	    
	  case IFT_INT32:
	    if ( EC_succeed != EC_arg( 3 ).unify( EC_word( (long) fit.get_int32() ) ) )
	    {
	      printf( "bb_read_interface/3: could not bind value\n" );
	      return EC_fail;
	    }
	    
	    break;
	    
	  case IFT_UINT32:
	    if ( EC_succeed != EC_arg( 3 ).unify( EC_word( (long) fit.get_uint32() ) ) )
	    {
	      printf( "p_read_from_interface(): could not bind return value\n" );
	      return EC_fail;
	    }
	    
	    break;
	    
	  case IFT_INT64:
	    if ( EC_succeed != EC_arg( 3 ).unify( EC_word( (long) fit.get_int64() ) ) )
	    {
	      printf( "p_read_from_interface(): could not bind return value\n" );
	      return EC_fail;
	    }
	    
	    break;
	    
	  case IFT_UINT64:
	    if ( EC_succeed != EC_arg( 3 ).unify( EC_word( (long) fit.get_uint64() ) ) )
	    {
	      printf( "p_read_from_interface(): could not bind return value\n" );
	      return EC_fail;
	    }
	    
	    break;
	    
	  case IFT_FLOAT:
	    if ( EC_succeed != EC_arg( 3 ).unify( EC_word( (double) fit.get_float() ) ) )
	    {
	      printf( "p_read_from_interface(): could not bind return value\n" );
	      return EC_fail;
	    }
	    
	    break;

	  case IFT_STRING:
	    if ( EC_succeed != EC_arg( 3 ).unify( EC_word( fit.get_string() ) ) )
	    {
	      printf( "p_read_from_interface(): could not bind return value\n" );
	      return EC_fail;
	    }
	    
	    break;
	    
	  case IFT_BYTE:
	  case IFT_ENUM:
	    printf( "p_read_from_interface(): NOT YET IMPLEMENTED\n" );
	    break;

	  default:
	    break;
	  }

	  break;
	}

      }

      if ( fit == (*it)->fields_end() )
      {
	printf( "bb_read_interface/3: interface %s has no field %s\n",
		interface_id, field );
	
	return EC_fail;
      }

      break;
    }
  }

  if ( it == g_interfaces.end() )
  {
    printf( "bb_read_interface/3: no interface with id %s found\n",
	    interface_id );
    
    return EC_fail;
  }

  return EC_succeed;
}

int
p_write_to_interface()
{
  char* interface_id;
  char* field;

  if ( EC_succeed != EC_arg( 1 ).is_string( &interface_id ) )
  {
    printf( "bb_write_interface/3: no interface id given\n" );
    return EC_fail;
  }

  if ( EC_succeed != EC_arg( 2 ).is_string( &field ) )
  {
    printf( "bb_write_interface/3: no field given\n" );
    return EC_fail;
  }

  vector< Interface* >::iterator it;
  for ( it = g_interfaces.begin();
	it != g_interfaces.end();
	++it )
  {
    if ( 0 == strcmp( interface_id, (*it)->id() ) )
    {
      if ( !(*it)->is_writer() )
      {
	printf( "bb_write_interface/3: not a writer\n" );
	return EC_fail;
      }

      InterfaceFieldIterator fit;
      for ( fit = (*it)->fields();
	    fit != (*it)->fields_end();
	    ++fit )
      {
	if ( 0 == strcmp( field, fit.get_name() ) )
	{
	  switch ( fit.get_type() ) {
	  case IFT_BOOL:
	    {
	      EC_atom val;
	      if ( EC_succeed != EC_arg( 3 ).is_atom( &val ) )
	      {
		printf( "p_write_to_interface(): no value_given\n" );
		return EC_fail;
	      }

	      if ( 0 == strcmp( "true", val.name() ) )
	      { fit.set_bool( true ); }
	      else if ( 0 == strcmp( "false", val.name() ) )
	      { fit.set_bool( false ); }
	      else
	      {
		printf( "p_write_to_interface(): boolean value neither true nor false\n" );
		return EC_fail;
	      }
	    }

	    break;
	    
	  case IFT_INT8:
	    {
	      long val;
	      if ( EC_succeed != EC_arg( 3 ).is_long( &val ) )
	      {
		printf( "p_write_to_interface(): no value given\n" );
		return EC_fail;
	      }

	      fit.set_int8( (int8_t) val );
	    }

	    break;
	    
	  case IFT_UINT8:
	    {
	      long val;
	      if ( EC_succeed != EC_arg( 3 ).is_long( &val ) )
	      {
		printf( "p_write_to_interface(): no value given\n" );
		return EC_fail;
	      }

	      fit.set_uint8( (uint8_t) val );
	    }

	    break;
	    
	  case IFT_INT16:
	    {
	      long val;
	      if ( EC_succeed != EC_arg( 3 ).is_long( &val ) )
	      {
		printf( "p_write_to_interface(): no value given\n" );
		return EC_fail;
	      }

	      fit.set_int16( (int16_t) val );
	    }

	    break;
	    
	  case IFT_UINT16:
	    {
	      long val;
	      if ( EC_succeed != EC_arg( 3 ).is_long( &val ) )
	      {
		printf( "p_write_to_interface(): no value given\n" );
		return EC_fail;
	      }

	      fit.set_uint16( (uint16_t) val );
	    }

	    break;
	    
	  case IFT_INT32:
	    {
	      long val;
	      if ( EC_succeed != EC_arg( 3 ).is_long( &val ) )
	      {
		printf( "p_write_to_interface(): no value given\n" );
		return EC_fail;
	      }

	      fit.set_int32( (int32_t) val );
	    }
	    
	    break;
	    
	  case IFT_UINT32:
	    {
	      long val;
	      if ( EC_succeed != EC_arg( 3 ).is_long( &val ) )
	      {
		printf( "p_write_to_interface(): no value given\n" );
		return EC_fail;
	      }

	      fit.set_uint32( (uint32_t) val );
	    }
	    
	    break;
	    
	  case IFT_INT64:
	    {
	      long val;
	      if ( EC_succeed != EC_arg( 3 ).is_long( &val ) )
	      {
		printf( "p_write_to_interface(): no value given\n" );
		return EC_fail;
	      }

	      fit.set_int64( (int64_t) val );
	    }
	    
	    break;
	    
	  case IFT_UINT64:
	    {
	      long val;
	      if ( EC_succeed != EC_arg( 3 ).is_long( &val ) )
	      {
		printf( "p_write_to_interface(): no value given\n" );
		return EC_fail;
	      }

	      fit.set_uint64( (uint64_t) val );
	    }
	    
	    break;
	    
	  case IFT_FLOAT:
	    {
	      double val;
	      if ( EC_succeed != EC_arg( 3 ).is_double( &val ) )
	      {
		printf( "p_write_to_interface(): no value given\n" );
		return EC_fail;
	      }

	      fit.set_float( (float) val );
	    }
	    
	    break;
	    
	  case IFT_STRING:
	    {
	      char* val;
	      if ( EC_succeed != EC_arg( 3 ).is_string( &val ) )
	      {
		printf( "p_write_to_interface(): no value given\n" );
		return EC_fail;
	      }

	      fit.set_string( val );
	    }
	    
	    break;
	    
	  case IFT_BYTE:
	  case IFT_ENUM:
	    printf( "p_write_to_interface(): NOT YET IMPLEMENTET\n" );
	    break;

	  default:
	    break;
	  }

	  break;
	}

      }

      if ( fit == (*it)->fields_end() )
      {
	printf( "bb_read_interface/3: interface %s has no field %s\n",
		interface_id, field );
	
	return EC_fail;
      }

      break;
    }
  }

  if ( it == g_interfaces.end() )
  {
    printf( "bb_read_interface/3: no interface with id %s found\n",
	    interface_id );
    
    return EC_fail;
  }

  return EC_succeed;
  
}
