
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


#include <eclipseclass.h>


#include <cstring>
#include <cstdlib>

namespace fawkes{
/** @class fawkes::EclExternalBlackBoard
 * Wrapper class for using the blackboard in the implementation of the external
 * predicates.
 * @author Daniel Beck
 */

BlackBoard* EclExternalBlackBoard::m_blackboard = NULL;
EclExternalBlackBoard*  EclExternalBlackBoard::m_instance = NULL;

  /** Constructor. */
  EclExternalBlackBoard::EclExternalBlackBoard(){
    if (m_instance == NULL){
      m_instance = this;
    }else{
      throw Exception( "There is already an instance of type EclExternalBlackBoard instantiated" );
    }
  }

  /** Constructor. */
  EclExternalBlackBoard::EclExternalBlackBoard(BlackBoard* blackboard) {
    if (m_instance == NULL){;
      m_instance = this;
      m_blackboard = blackboard;
    }else{
      throw Exception( "There is already an instance of type EclExternalBlackBoard instantiated" );
    }
  }

  /** Destructor. */
  EclExternalBlackBoard::~EclExternalBlackBoard() {
    for ( std::vector< Interface* >::iterator iit = m_interfaces.begin();
	  iit != m_interfaces.end();
	  ++iit )
    { m_blackboard->close( *iit ); }
    delete m_instance;
    //delete m_blackboard;
  }

  /** Creates the initial EclExternalBlackBoard object
   * @param bb pointer to the BlackBoard to be used
   */
  void EclExternalBlackBoard::create_initial_object(BlackBoard *bb) {
      m_instance = new EclExternalBlackBoard(bb);
  }


  /** Get the EclExternalBlackBoard instance.
  * @return the instance
  */
  EclExternalBlackBoard* EclExternalBlackBoard::instance()
  {
    if ( !m_instance )
    { throw Exception( "No instance of type EclExternalBlackBoard instantiated" ); }

    return m_instance;
  }


  /** Open remote blackboard connection.
   * @param host the host running Fawkes
   */
  void EclExternalBlackBoard::connect( const char* host )
  {
    m_blackboard = new RemoteBlackBoard( host, 1910 );
  }


  /** Query connection status.
   * @return true if connected; false otherwise
   */
  bool EclExternalBlackBoard::connected()
  {
    return m_blackboard ? true : false;
  }

  /** Disconnect remote blackboard connection. */
  void EclExternalBlackBoard::disconnect()
  {
    for ( std::vector< Interface* >::iterator iit = m_interfaces.begin();
	  iit != m_interfaces.end();
	  ++iit )
    { m_blackboard->close( *iit ); }
    //delete m_blackboard;
    //m_blackboard = 0;
  }

  /** Access the BlackBoard instance.
   * @return the blackboard instance
   */
  BlackBoard* EclExternalBlackBoard::blackboard_instance()
  {
    if ( !m_blackboard )
    { throw Exception( "No instance of type BlackBoard instantiated" ); }

    return m_blackboard;
  }

  /** Obtain the list of opened interfaces.
   * @return list of opened interfaces
   */
  std::vector< Interface* >& EclExternalBlackBoard::interfaces()
  {
    return m_interfaces;
  }

}

using namespace std;
using namespace fawkes;


bool process_message_args(Message* msg, EC_word arg_list);

int
p_connect_to_remote_blackboard()
{
  if ( EclExternalBlackBoard::instance()->connected() )
  {
    printf( "p_connect_to_remote_blackboard(): already connected\n" );
    return EC_fail;
  }

  // get hostname
  char* hostname;

  if ( EC_succeed != EC_arg( 2 ).is_string( &hostname ) )
  {
    printf( "p_connect_to_remote_blackboard(): first argument is not a string\n" );
    return EC_fail;
  }

  try
  {
    EclExternalBlackBoard::instance()->connect( hostname );
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
  if ( !EclExternalBlackBoard::instance()->connected() )
  {
    printf( "p_disconnect_from_blackboard(): not connected\n" );
    return EC_fail;
  }

  EclExternalBlackBoard::instance()->disconnect();

  return EC_succeed;
}


int
p_is_alive()
{
  if ( !EclExternalBlackBoard::instance()->connected() )
  {
    printf( "p_is_alive(): not connected\n" );
    return EC_fail;
  }

  if ( EclExternalBlackBoard::instance()->blackboard_instance()->is_alive() )
  { return EC_succeed; }
  else
  { return EC_fail; }
}

int
p_is_connected()
{
  if ( EclExternalBlackBoard::instance()->connected() ){
    return EC_succeed;
  }else{
    return EC_fail;
  }
}

int
p_open_interface()
{
  if ( !EclExternalBlackBoard::instance()->connected() )
  {
    printf("p_open_interface(): not connected\n" );
    return EC_fail;
  }

  EC_atom mode;
  char* interface_type;
  char* interface_id;

  if ( EC_succeed != EC_arg( 1 ).is_atom( &mode) )
  {
    printf( "p_open_interface(): no mode given\n" );
    return EC_fail;
  }

  if ( EC_succeed != EC_arg( 2 ).is_string( &interface_type ) )
  {
    printf( "p_open_interface(): no type given\n" );
    return EC_fail;
  }

  if ( EC_succeed != EC_arg ( 3 ).is_string( &interface_id ) )
  {
    printf( "p_open_interface(): no id given\n" );
    return EC_fail;
  }

  try
  {
    Interface* iface;

    if ( 0 == strcmp( "w", mode.name() ) )
    {  iface = EclExternalBlackBoard::instance()->blackboard_instance()->open_for_writing( interface_type, interface_id ); }
    else
    { iface = EclExternalBlackBoard::instance()->blackboard_instance()->open_for_reading( interface_type, interface_id ); }

    EclExternalBlackBoard::instance()->interfaces().push_back( iface );
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
  if ( !EclExternalBlackBoard::instance()->connected() )
  {
    printf("p_close_interface(): not connected\n" );
    return EC_fail;
  }

  // char* interface_type;
  char* interface_id;

  if ( EC_succeed != EC_arg( 1 ).is_string( &interface_id ) )
  {
    printf( "p_close_interface(): no id given\n" );
    return EC_fail;
  }

  bool iface_found = false;

  for ( vector< Interface* >::iterator it = EclExternalBlackBoard::instance()->interfaces().begin();
	it != EclExternalBlackBoard::instance()->interfaces().end();
	++it )
  {
    if ( 0 == strcmp( (*it)->id(), interface_id ) )

    {
      iface_found = true;
      EclExternalBlackBoard::instance()->blackboard_instance()->close( *it );
      EclExternalBlackBoard::instance()->interfaces().erase( it );
      break;
    }
  }

  if ( iface_found )
  { return EC_succeed; }
  else
  { return EC_fail; }
}


int
p_has_writer()
{
  char* interface_id;

  if ( EC_succeed != EC_arg( 1 ).is_string( &interface_id ) )
  {
    printf( "p_has_writer(): no id given\n" );
    return EC_fail;
  }

  for ( vector< Interface* >::iterator it = EclExternalBlackBoard::instance()->interfaces().begin();
	it != EclExternalBlackBoard::instance()->interfaces().end();
	++it )
  {
    if ( 0 == strcmp( (*it)->id(), interface_id ) )
    {
      if ( (*it)->has_writer() )
      { return EC_succeed; }
      else
      { return EC_fail; }

      break;
    }
  }

  return EC_fail;
}


int
p_instance_serial()
{
  if ( !EclExternalBlackBoard::instance()->connected() )
  {
    printf( "p_instance_serial(): not connected to blackboard\n" );
    return EC_fail;
  }

  char* interface_id;
  if ( EC_succeed != EC_arg( 1 ).is_string( &interface_id ) )
  {
    printf( "p_instance_serial(): no interface id given\n" );
    return EC_fail;
  }

  for ( vector< Interface* >::iterator iit = EclExternalBlackBoard::instance()->interfaces().begin();
	iit != EclExternalBlackBoard::instance()->interfaces().end();
	++iit )
  {
    if ( 0 == strcmp( (*iit)->id(), interface_id ) )
    {
      if ( EC_succeed != EC_arg( 2 ).unify( (*iit)->serial() ) )
      {
	printf( "p_instance_serial(): could not bind return value\n" );
	return EC_fail;
      }

      return EC_succeed;
    }
  }

  return EC_fail;
}


int
p_read_interfaces()
{
  for ( vector< Interface* >::iterator it = EclExternalBlackBoard::instance()->interfaces().begin();
	it != EclExternalBlackBoard::instance()->interfaces().end();
	++it )
  {
    (*it)->read();
  }

  return EC_succeed;
}

int
p_write_interfaces()
{
  for ( vector< Interface* >::iterator it = EclExternalBlackBoard::instance()->interfaces().begin();
	it != EclExternalBlackBoard::instance()->interfaces().end();
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
    printf( "p_read_from_interface(): no interface id given\n" );
    return EC_fail;
  }

  if ( EC_succeed != EC_arg( 2 ).is_string( &field ) )
  {
    printf( "p_read_from_interface(): no field given\n" );
    return EC_fail;
  }

  vector< Interface* >::iterator it;
  for ( it = EclExternalBlackBoard::instance()->interfaces().begin();
	it != EclExternalBlackBoard::instance()->interfaces().end();
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
	    printf( "p_read_from_interface(): NOT YET IMPLEMENTED\n" );
	    break;

	  case IFT_ENUM:
	    if ( EC_succeed != EC_arg( 3 ).unify( fit.get_value_string() ) )
	    {
	      printf( "p_read_from_interface(): could not bind return value\n" );
	      return EC_fail;
	    }

	  default:
	    break;
	  }

	  break;
	}

      }

      if ( fit == (*it)->fields_end() )
      {
	printf( "p_read_from_interface(): interface %s has no field %s\n",
		interface_id, field );
	
	return EC_fail;
      }

      break;
    }
  }

  if ( it == EclExternalBlackBoard::instance()->interfaces().end() )
  {
    printf( "p_read_from_interface(): no interface with id %s found\n",
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
    printf( "p_write_to_interface(): no interface id given\n" );
    return EC_fail;
  }

  if ( EC_succeed != EC_arg( 2 ).is_string( &field ) )
  {
    printf( "p_write_to_interface(): no field given\n" );
    return EC_fail;
  }

  vector< Interface* >::iterator it;
  for ( it = EclExternalBlackBoard::instance()->interfaces().begin();
	it != EclExternalBlackBoard::instance()->interfaces().end();
	++it )
  {
    if ( 0 == strcmp( interface_id, (*it)->id() ) )
    {
      if ( !(*it)->is_writer() )
      {
	printf( "p_write_to_interface(): not a writer\n" );
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
	printf( "p_write_to_interface(): interface %s has no field %s\n",
		interface_id, field );
	
	return EC_fail;
      }

      break;
    }
  }

  if ( it == EclExternalBlackBoard::instance()->interfaces().end() )
  {
    printf( "p_write_to_interface(): no interface with id %s found\n",
	    interface_id );
    
    return EC_fail;
  }

  return EC_succeed;
  
}


int
p_send_message()
{
  char* interface_id;
  char* message_type;

  if ( EC_succeed != EC_arg( 1 ).is_string( &interface_id ) )
  {
    printf( "p_send_message(): no interface id given\n" );
    return EC_fail;
  }

  if ( EC_succeed != EC_arg( 2 ).is_string( &message_type ) )
  {
    printf( "p_send_message(): no message type given\n" );
    return EC_fail;
  }

  vector< Interface* >::iterator it;
  for ( it = EclExternalBlackBoard::instance()->interfaces().begin();
	it != EclExternalBlackBoard::instance()->interfaces().end();
	++it )
  {
    if ( 0 == strcmp( interface_id, (*it)->id() ) )
    {
      if ( (*it)->is_writer() )
      {
	printf( "p_send_message(): interface with id %s is a writer\n", interface_id );
	return EC_fail;
      }

      try
      {
	Message* msg = (*it)->create_message( message_type );

	EC_word head;
	EC_word tail;
	if ( EC_succeed == EC_arg( 3 ).is_list( head, tail ) )
	{
	  if ( !process_message_args(msg, ::list(head, tail)) )
	  {
	    return EC_fail;
	  };
	}

	(*it)->msgq_enqueue( msg );
      }
      catch (Exception& e)
      {
	e.print_trace();
	return EC_fail;
      }

      break;
    }
  }

  if ( it == EclExternalBlackBoard::instance()->interfaces().end() )
  {
    printf( "p_send_message(): no interface with name %s\n", interface_id );
    return EC_fail;
  }

  return EC_succeed;
}


int
p_recv_messages()
{
  char* interface_id;

  if ( EC_succeed != EC_arg( 1 ).is_string( &interface_id ) )
  {
    printf( "p_recv_messages(): no interface id given\n" );
    return EC_fail;
  }

  vector< Interface* >::iterator it;

  for ( it  = EclExternalBlackBoard::instance()->interfaces().begin();
	it != EclExternalBlackBoard::instance()->interfaces().end();
	++it )
  {
    if ( 0 == strcmp( interface_id, (*it)->id() ) )
    {
      EC_word msg_list = nil();

      while ( !(*it)->msgq_empty() )
      {
	Message* msg = (*it)->msgq_first();

	// construct list of key-value pairs: [[field1, val1], [field2, val2], ...]
	EC_word args = nil();
	for ( InterfaceFieldIterator fit = msg->fields();
	      fit != msg->fields_end();
	      ++fit )
	{
	  EC_word value;

	  switch ( fit.get_type() )
	  {
	  case IFT_BOOL:
	    if ( fit.get_bool() )
	    { value = EC_atom( (char*) "true" ); }
	    else
	    { value = EC_atom( (char*) "false" ); }

	    break;

	  case IFT_INT8:
	    value = EC_word( (long) fit.get_int8() );
	    break;

	  case IFT_UINT8:
	    value = EC_word( (long) fit.get_uint8() );
	    break;

	  case IFT_INT16:
	    value = EC_word( (long) fit.get_int16() );
	    break;

	  case IFT_UINT16:
	    value = EC_word( (long) fit.get_uint16() );
	    break;

	  case IFT_INT32:
	    value = EC_word( (long) fit.get_int32() );
	    break;

	  case IFT_UINT32:
	    value = EC_word( (long) fit.get_uint32() );
	    break;

	  case IFT_INT64:
	    value = EC_word( (long) fit.get_int64() );
	    break;

	  case IFT_UINT64:
	    value = EC_word( (long) fit.get_uint64() );
	    break;

	  case IFT_FLOAT:
	    value = EC_word( (double) fit.get_float() );
	    break;

	  case IFT_STRING:
	    value = EC_word( fit.get_string() );
	    break;

	  case IFT_BYTE:
	  case IFT_ENUM:
	    printf( "p_recv_messages(): NOT YET IMPLEMENTED\n" );
	    break;

	  default:
	    printf( "p_recv_messages(): unknown field type\n" );
	  }

	  EC_word field = ::list( EC_word( fit.get_name() ),
				  ::list( value, nil() ) );
	  args = ::list(field, args);
	}

	// construct list of messages: [[MsgType, [[Key1, Val1], ...]], ... ]
	msg_list = ::list( ::list( EC_word( msg->type() ),
				   ::list(args, nil() ) ),
			   msg_list );

	(*it)->msgq_pop();
      }

      if ( EC_succeed != EC_arg( 2 ).unify( msg_list ) )
      {
	printf( "p_recv_messages(): could not bind return value\n" );
	return EC_fail;
      }

      break;
    }
  }

  if ( it == EclExternalBlackBoard::instance()->interfaces().end() )
  {
    printf( "p_recv_messages(): no interface with id %s found\n", interface_id );
    return EC_fail;
  }

  return EC_succeed;
}


bool
process_message_args(Message* msg, EC_word arg_list)
{
  EC_word head;
  EC_word tail;

  for ( ; EC_succeed == arg_list.is_list(head, tail) ; arg_list = tail )
  {
    // [field, value]
    EC_word field;
    EC_word value;
    EC_word t1;
    EC_word t2;

    if ( EC_succeed != head.is_list( field, t1 ) ||
	 EC_succeed != t1.is_list( value, t2 )        )
    {
      printf( "p_send_messge(): could not parse argument list\n" );
      return false;
    }

    char* field_name;
    if ( EC_succeed != field.is_string( &field_name ) )
    {
      printf( "p_send_message(): malformed argument list\n" );
      return false;
    }

    InterfaceFieldIterator fit;
    for ( fit = msg->fields();
	  fit != msg->fields_end();
	  ++fit )
    {
      if ( 0 == strcmp( fit.get_name(), field_name ) )
      {
	switch( fit.get_type() )
	{
	case IFT_BOOL:
	  {
	    EC_atom val;
	    if ( EC_succeed != value.is_atom( &val ) )
	    {
	      printf( "p_send_message(): no value_given (bool)\n" );
	      return false;
	    }

	    if ( 0 == strcmp( "true", val.name() ) )
	    { fit.set_bool( true ); }
	    else if ( 0 == strcmp( "false", val.name() ) )
	    { fit.set_bool( false ); }
	    else
	    {
	      printf( "p_send_message(): boolean value neither true nor false\n" );
	      return false;
	    }
	  }

	  break;

	case IFT_INT8:
	  {
	    long val;
	    if ( EC_succeed != value.is_long( &val ) )
	    {
	      printf( "p_send_message(): no value given (int8)\n" );
	      return false;
	    }

	    fit.set_int8( (int8_t) val );
	  }

	  break;

	case IFT_UINT8:
	  {
	    long val;
	    if ( EC_succeed != value.is_long( &val ) )
	    {
	      printf( "p_send_message(): no value given (uint8)\n" );
	      return false;
	    }

	    fit.set_uint8( (uint8_t) val );
	  }

	  break;

	case IFT_INT16:
	  {
	    long val;
	    if ( EC_succeed != value.is_long( &val ) )
	    {
	      printf( "p_send_message(): no value given (int16)\n" );
	      return false;
	    }

	    fit.set_int16( (int16_t) val );
	  }

	  break;

	case IFT_UINT16:
	  {
	    long val;
	    if ( EC_succeed != value.is_long( &val ) )
	    {
	      printf( "p_send_message(): no value given (uint16)\n" );
	      return false;
	    }

	    fit.set_uint16( (uint16_t) val );
	  }

	  break;

	case IFT_INT32:
	  {
	    long val;
	    if ( EC_succeed != value.is_long( &val ) )
	    {
	      printf( "p_send_message(): no value given (int32)\n" );
	      return false;
	    }

	    fit.set_int32( (int32_t) val );
	  }

	  break;

	case IFT_UINT32:
	  {
	    long val;
	    if ( EC_succeed != value.is_long( &val ) )
	    {
	      printf( "p_send_message(): no value given (uint32)\n" );
	      return false;
	    }

	    fit.set_uint32( (uint32_t) val );
	  }

	  break;

	case IFT_INT64:
	  {
	    long val;
	    if ( EC_succeed != value.is_long( &val ) )
	    {
	      printf( "p_send_message(): no value given (int64)\n" );
	      return false;
	    }

	    fit.set_int64( (int64_t) val );
	  }

	  break;

	case IFT_UINT64:
	  {
	    long val;
	    if ( EC_succeed != value.is_long( &val ) )
	    {
	      printf( "p_send_message(): no value given (uint64)\n" );
	      return false;
	    }

	    fit.set_uint64( (uint64_t) val );
	  }

	  break;

	case IFT_FLOAT:
	  {
	    double val;
	    if ( EC_succeed != value.is_double( &val ) )
	    {
	      printf( "p_send_message(): no value given (float)\n" );
	      return false;
	    }

	    fit.set_float( (float) val );
	  }

	  break;

	case IFT_STRING:
	  {
	    char* val;
	    if ( EC_succeed != value.is_string( &val ) )
	    {
	      printf( "p_send_message(): no value given (string)\n" );
	      return false;
	    }

	    fit.set_string( val );
	  }

	  break;

	case IFT_BYTE:
	case IFT_ENUM:
	  printf( "p_send_message(): NOT YET IMPLEMENTET\n" );
	  break;

	default:
	  break;
	}

	break;
      }
    }

    if ( fit == msg->fields_end() )
    {
      printf( "p_send_message(): message has no field with name %s\n",
	      field_name );
      return false;
    }
  }

  return true;
}
