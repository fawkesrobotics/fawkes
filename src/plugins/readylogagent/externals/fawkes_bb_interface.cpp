
/***************************************************************************
 *  fawkes_bb_interface.h - External predicates to access Fawkes interfaces
 *
 *  Created: Wed Jul 15 16:20:04 2009
 *  Copyright  2009  Daniel Beck
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

#include "fawkes_bb_interface.h"
#include <plugins/readylogagent/eclipse_thread.h>

#include <core/exceptions/software.h>
#include <interface/interface.h>
#include <interfaces/ObjectPositionInterface.h>

#include <eclipseclass.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <list>

using namespace std;
using namespace fawkes;

EC_word  construct_iface_struct( Interface* iface );
EC_word  construct_msg_struct( const char* iface_type, Message* msg );
EC_word* construct_struct_args( const InterfaceFieldIterator& begin,
				const InterfaceFieldIterator& end,
				unsigned int num_fields );
void parse_struct_args( EC_word iface_struct,
			const InterfaceFieldIterator& begin,
			const InterfaceFieldIterator& end,
			unsigned int num_fields );

int
p_read_interface()
{
  // read_interface(+IdStr, -DataStruct)
  // DataStruct is bound to a struct with named elements. The name of
  // that struct is data_InterfaceType, the fields are named in the
  // same way as the fields of the interface.

  // check if interface with given id is available
  Interface* interface;
  char* id;

  // get interface id
  if ( EC_succeed != EC_arg( 1 ).is_string( &id ) )
  { 
    printf( "First argument of read_interface/2 is not a string\n" );
    return EC_fail;
  }

  // get interface
  interface = EclipseAgentThread::instance()->get_registered_interface( id );
  if ( !interface )
  {
    printf( "Interface with id %s is not available to the agent\n", id );
    return EC_fail;
  }

  // construct data structure
  EC_word iface_struct;
  try
  {
    iface_struct = construct_iface_struct( interface );
  }
  catch ( Exception& e )
  {
    e.print_trace();
    return EC_fail;
  }

  // bind 2nd argument
  return unify( EC_arg(2), iface_struct );
}

int
p_write_interface()
{
  // write_interface(+IdStr, +DataStruct)
  // The structure passed as the second argument has to be of type
  // data_InterfaceType such that the interface type matches the type
  // of the interface with the given id.

  char* id;
  Interface* interface;

  // get interface id
  if ( EC_succeed != EC_arg( 1 ).is_string( &id ) )
  {
    printf( "Firste argument of write_interface/2 is not a string\n" );
    return EC_fail;
  }

  // get interface
  interface = EclipseAgentThread::instance()->get_registered_interface( id );
  if ( !interface )
  {
    printf( "Interface with id %s is not available to the agent\n", id );
    return EC_fail;
  }

  // get functor of 2nd argument
  EC_functor fctor;
  if ( EC_succeed != EC_arg( 2 ).functor( &fctor ) )
  {
    printf( "Second argument of writer_interface/2 is not a compound term\n" );
    return EC_fail;
  }

  // check interface type
  char* iface_type = 0;
  if ( 1 != sscanf( fctor.name(), "data_%s", iface_type ) ||
       0 != strcmp( interface->type(), iface_type ) )
  {
    printf( "Second argument of write_interface/2 is not of type data_%s but of type %s\n",
	    interface->type(), fctor.name() );
    free( iface_type );
    return EC_fail;
  }
  free( iface_type );

  // check arity
  unsigned int arity = (unsigned int) EC_arg( 2 ).arity();
  if ( interface->num_fields() != arity )
  {
    printf( "Second argument of write_interface/2 has wrong arity\n" );
    return EC_fail;
  }

  // copy data
  try
  {
    parse_struct_args( EC_word( 2 ),
		       interface->fields(),
		       interface->fields_end(),
		       interface->num_fields() );
  }
  catch ( Exception& e )
  {
    e.print_trace();
    return EC_fail;
  }

  return EC_succeed;
}

int
p_send_message()
{
  // send_message(id, data_InterfaceType_MessageType{key1:param1, ...})

  // check if interface with given id is available
  char* iface_id;
  Interface* interface;

  // get interface id
  if ( EC_succeed != EC_arg( 1 ).is_string( &iface_id ) )
  { 
    printf( "First argument of send_message/2 is not an atom\n" );
    return EC_fail;
  }

  // get interface
  interface = EclipseAgentThread::instance()->get_registered_interface( iface_id );
  if ( !interface )
  {
    printf( "Interface with id %s is not available to the agent\n", iface_id );
    return EC_fail;
  }

  // check functor of 2nd argument
  EC_functor fctor;
  if ( EC_succeed != EC_arg( 2 ).functor( &fctor ) )
  {
    printf( "Second argument of send_message/2 has no functor\n" );
    return EC_fail;
  }

  char* fctor_name = strdup( fctor.name() );
  strtok( fctor_name, "_" );
  char* iface_type = strtok( NULL, "_" );
  char* msg_type = strtok( NULL, "_" );
  
  if ( !iface_type ||
       !msg_type   ||
       0 != strcmp( interface->type(), iface_type )  )
  {
    printf( "Malformed functor: %s\n", fctor.name() );
    return EC_fail;
  }

  // create message of given type
  Message* msg;
  try
  {
    msg = interface->create_message( msg_type );
  }
  catch ( UnknownTypeException& e )
  {
    printf( "Message type %s is not available for interfaces of type %s\n",
	    msg_type, interface->type() );
    return EC_fail;
  }

  // cleanup
  free( fctor_name );

  // parse parameters
  try
  {
    parse_struct_args( EC_arg( 2 ), msg->fields(), msg->fields_end(), msg->num_fields() );
  }
  catch ( Exception& e )
  {
    e.print_trace();
    return EC_fail;
  }

  // enqueue message
  interface->msgq_enqueue( msg );

  return EC_succeed;
}

int
p_recv_messages()
{
  // recv_messages( +IdStr, -MsgList )
  // MsgList is a list of message type structurs. The last message in
  // the list is the most recent one.

  // check if interface with given id is available
  char* iface_id;
  Interface* interface;

  if ( EC_succeed != EC_arg( 1 ).is_string( &iface_id ) )
  { 
    printf( "First argument of send_message/2 is not an atom\n" );
    return EC_fail;
  }

  interface = EclipseAgentThread::instance()->get_registered_interface( iface_id );
  if ( !interface )
  {
    printf( "Interface with id %s is not available to the agent\n", iface_id );
    return EC_fail;
  }

  std::list< EC_word > messages;

  while ( !interface-> msgq_empty() )
  {
    Message* msg = interface->msgq_first();
    EC_word msg_struct;

    try
    {
      msg_struct = construct_msg_struct( interface->type(), msg );
    }
    catch( Exception& e )
    {
      e.print_trace();
      return EC_fail;
    }

    messages.push_back( msg_struct );
    
    interface->msgq_pop();
  }

  EC_word l = nil();
  while( !messages.empty() )
  {
    l = ::list( messages.front(), l );
    messages.pop_front();
  }

  return unify( EC_arg( 2 ), l );
}

EC_word
construct_iface_struct( Interface* iface )
{
  EC_word* args = construct_struct_args( iface->fields(),
					 iface->fields_end(),
					 iface->num_fields() );

  char* fctor;
  asprintf( &fctor, "data_%s", iface->type() );

  EC_word ret = term( EC_functor( fctor, iface->num_fields() ), args );

  delete[] args;
  free( fctor );

  return ret;
}

EC_word
construct_msg_struct( const char* iface_type, Message* msg )
{
  EC_word* args = construct_struct_args( msg->fields(), msg->fields_end(), msg->num_fields() );

  char* fctor;
  asprintf( &fctor, "data_%s_%s", iface_type, msg->type() );

  EC_word ret = term( EC_functor( fctor, msg->num_fields() ), args );

  delete[] args;
  free( fctor );

  return ret;
}

EC_word*
construct_struct_args( const InterfaceFieldIterator& begin,
		       const InterfaceFieldIterator& end,
		       unsigned int num_fields )
{
  EC_word* args = new EC_word[ num_fields ];

  InterfaceFieldIterator field_iter;
  unsigned int           field_idx;
  for ( field_iter = begin, field_idx = 0;
	field_iter != end;
	++field_iter, ++field_idx )
  {
    interface_fieldtype_t type = field_iter.get_type();

    switch ( type )
    {
    case IFT_BOOL: {
      char* t = strdup( "true" );
      char* f = strdup( "fail" );

      args[ field_idx ] = field_iter.get_bool() ? EC_atom( t ) : EC_atom( f );
      
      free( t );
      free( f );
      
      break;
    }

    case IFT_INT:
      args[ field_idx ] = EC_word( field_iter.get_int() );
      break;

    case IFT_UINT:
      args[ field_idx ] = EC_word( (long) field_iter.get_uint() );
      break;

    case IFT_LONGINT:
      args[ field_idx ] = EC_word( field_iter.get_longint() );
      break;

    case IFT_LONGUINT:
      args[ field_idx ] = EC_word( (long) field_iter.get_longuint() );
      break;

    case IFT_FLOAT:
      args[ field_idx ] = EC_word( field_iter.get_float() );
      break;

    case IFT_STRING:
      args[ field_idx ] = EC_word( field_iter.get_string() );
      break;

    case IFT_BYTE:
      args[ field_idx ] = EC_word( field_iter.get_byte() );
      break;

    default:
      throw UnknownTypeException( "Field of unknown type %s", field_iter.get_typename() );
    }
  }

  return args;
}

void
parse_struct_args( EC_word data_struct,
		   const InterfaceFieldIterator& begin,
		   const InterfaceFieldIterator& end,
		   unsigned int num_fields )
{
  unsigned int           field_idx;
  InterfaceFieldIterator field_iter;

  for ( field_iter = begin, field_idx = 1;
	field_iter != end;
	++field_iter, ++field_idx )
  {
    interface_fieldtype_t type = field_iter.get_type();

    EC_word arg;
    if ( EC_succeed != data_struct.arg( field_idx, arg ) )
    { throw Exception( "Failed to parse interface data. Couldn't read %d-th parameter.\n", field_idx ); }

    switch( type )
    {
    case IFT_BOOL: {
      char* t = strdup( "true" );
      char* f = strdup( "fail" );
      if ( EC_succeed == arg.unify( EC_atom( t ) ) )
      { field_iter.set_bool( true ); }
      else if ( EC_succeed == arg.unify( EC_atom( f ) ) )
      { field_iter.set_bool( false ); }
      else
      {
	free( t );
	free( f );
	throw TypeMismatchException( "Wrong data type for %d-th argument\n", field_idx );
      }

      free( t );
      free( f );

      break;
    }

    case IFT_INT: {
      long val;
      if ( EC_succeed == arg.is_long( &val ) )
      { field_iter.set_int( (int) val ); }
      else
      { throw TypeMismatchException( "Wrong data type for %d-the argument\n", field_idx ); }

      break;
    }

    case IFT_UINT: {
      long val;
      if ( EC_succeed == arg.is_long( &val ) )
      { field_iter.set_uint( (unsigned int) val ); }
      else
      { throw TypeMismatchException( "Wrong data type for %d-the argument\n", field_idx ); }

      break;
    }

    case IFT_LONGINT: {
      long val;
      if ( EC_succeed == arg.is_long( &val ) )
      { field_iter.set_longint( val ); }
      else
      { throw TypeMismatchException( "Wrong data type for %d-the argument\n", field_idx ); }

      break;
    }

    case IFT_LONGUINT: {
      long val;
      if ( EC_succeed == arg.is_long( &val ) )
      { field_iter.set_longuint( (long unsigned int) val ); }
      else
      { throw TypeMismatchException( "Wrong data type for %d-the argument\n", field_idx ); }

      break;
    }

    case IFT_FLOAT: {
      double val;
      if ( EC_succeed == arg.is_double( &val ) )
      { field_iter.set_float( (float) val ); }
      else
      { throw TypeMismatchException( "Wrong data type for %d-the argument\n", field_idx ); }

      break;
    }
      
    case IFT_STRING: {
      char* val;
      if ( EC_succeed == arg.is_string( &val ) )
      { field_iter.set_string( val ); }
      else
      { throw TypeMismatchException( "Wrong data type for %d-the argument\n", field_idx ); }

      break;
    }
      
    default:
      break;
    }
  }
}
