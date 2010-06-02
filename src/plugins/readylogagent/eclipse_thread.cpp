
/***************************************************************************
 *  eclipse_thread.cpp - Fawkes Readylog ECLiPSe Thread
 *
 *  Created: Wed Jul 16 10:42:49 2009
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

#include "eclipse_thread.h"
#include "externals/fawkes_bb_interface.h"
#include "externals/fawkes_logger.h"

#include <interfaces/TestInterface.h>
#include <core/exception.h>

#include <eclipseclass.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

using namespace std;
using namespace fawkes;

/** @class EclipseAgentThread "eclipse_thread.h"
 * This thread creates an ECLiPSe context in which the Readylog
 * interpreter and the program are loaded.
 * @author Daniel Beck
 */

extern "C" int ec_external( dident, int (*) (), dident );

EclipseAgentThread* EclipseAgentThread::m_instance = NULL;

/** Constructor. */
EclipseAgentThread::EclipseAgentThread()
  : Thread( "ECLiPSe thread", fawkes::Thread::OPMODE_CONTINUOUS ),
    m_initialized( false )
{
  m_instance = this;
}

/** Destructor. */
EclipseAgentThread::~EclipseAgentThread()
{
}

void
EclipseAgentThread::init()
{
  // set ECLiPSe installation directory
  char* eclipse_dir = NULL;
  try
  {
    eclipse_dir = strdup( config->get_string( "/readylogagent/eclipse_dir" ).c_str() );
    logger->log_info( name(), "Setting ECLIPSEDIR to %s", eclipse_dir );
    ec_set_option_ptr( EC_OPTION_ECLIPSEDIR, (void*) eclipse_dir );
  }
  catch (...)
  {
    // ignore
  }

  // initialize ECLiPSe context
  if ( 0 != ec_init() )
  { throw fawkes::Exception( "Failed to initialize ECLiPSe context" ); }

  free( eclipse_dir );

  // register external predicates
  if ( EC_succeed != ec_external( ec_did( "read_interface",  2 ), p_read_interface,  ec_did( "eclipse", 0 ) ) )
  { throw Exception( "Registering external predicate read_interface/2 failed" ); }
  if ( EC_succeed != ec_external( ec_did( "write_interface", 2 ), p_write_interface, ec_did( "eclipse", 0 ) ) )
  { throw Exception( "Registering external predicate write_interface/2 failed" ); }
  if ( EC_succeed != ec_external( ec_did( "send_message",    2 ), p_send_message,    ec_did( "eclipse", 0 ) ) )
  { throw Exception( "Registering external predicate send_message/2 failed" ); }
  if ( EC_succeed != ec_external( ec_did( "recv_messages",   2 ), p_recv_messages,   ec_did( "eclipse", 0 ) ) )
  { throw Exception( "Registering external predicate recv_messages/2 failed" ); }
  if ( EC_succeed != ec_external( ec_did( "log",             2 ), p_log,             ec_did( "eclipse", 0 ) ) )
  { throw Exception( "Registering external predicate log/2 failed" ); }

  m_initialized = true;

  // open & register interfaces
  try
  {
    // open for interfaces reading
    Configuration::ValueIterator* vit = config->search( "/readylogagent/interfaces/reading" );
    while ( vit->next() )
    {
      if ( vit->is_string() )
      {
	string s = vit->get_string();
	if ( s.find("::") == string::npos )
	{ throw Exception( "Not a valid interface id: %s", s.c_str() ); }

	string iftype = s.substr( 0, s.find( "::" ) );
	string ifname = s.substr( s.find( "::" ) + 2 );

	logger->log_debug( name(), "Opening interface %s of type %s for reading",
			   ifname.c_str(), iftype.c_str() );

	Interface* iface = blackboard->open_for_reading( iftype.c_str(), ifname.c_str() );
	m_reading_ifaces.push_back( iface );
	register_interface( iface );
      }
    }

    // open interfaces for writing
    vit = config->search( "/readylogagent/interfaces/writing" );
    while ( vit->next() )
    {
      if ( vit->is_string() )
      {
	string s = vit->get_string();
	if ( s.find("::") == string::npos )
	{ throw Exception( "Not a valid interface id: %s", s.c_str() ); }

	string iftype = s.substr( 0, s.find( "::" ) );
	string ifname = s.substr( s.find( "::" ) + 2 );

	logger->log_debug( name(), "Opening interface %s of type %s for writing",
			   ifname.c_str(), iftype.c_str() );

	Interface* iface = blackboard->open_for_writing( iftype.c_str(), ifname.c_str() );
	m_writing_ifaces.push_back( iface );
	register_interface( iface );
      }
    }
  }
  catch ( Exception& e )
  {
    e.append( "Failed to open interfaces" );
    throw e;
  }

  // load utility predicates
  load_file( ECLIPSE_CODE_DIR"/utils/logging.ecl" );

  // load interpreter and agent
  load_file( ECLIPSE_CODE_DIR"/interpreter/dummy.ecl" );
}

void
EclipseAgentThread::finalize()
{
  ec_cleanup();
}

void
EclipseAgentThread::once()
{
  post_goal( "run" );
  if ( EC_succeed != EC_resume() )
  { throw Exception( "Error running agent program" ); }
}

/** Post an event to the ECLiPSe context.
 * @param event the name of the event
 */
void
EclipseAgentThread::post_event( const char* event )
{
  if ( !m_initialized ) { return; }

  // send event to the interpreter
  char* atom = strdup( event );
  ::post_event( EC_atom( atom ) );
  free( atom );
}

/** Read all registered interfaces. */
void
EclipseAgentThread::read_interfaces()
{
  for ( vector< Interface* >::iterator i = m_reading_ifaces.begin();
	i != m_reading_ifaces.end();
	++i )
  { (*i)->read(); }

  for ( vector< Interface* >::iterator i = m_writing_ifaces.begin();
	i != m_writing_ifaces.end();
	++i )
  { (*i)->read(); }
}

/** Write the registered interface that have been opened for writing. */
void
EclipseAgentThread::write_interfaces()
{
  for ( vector< Interface* >::iterator i = m_writing_ifaces.begin();
	i != m_writing_ifaces.end();
	++i )
  { (*i)->write(); }
}

/** Load a file into the ECLiPSe context.
 * @param filename the name of the file
 * @return false if the ECLiPSe context hasn't been intialized yet
 */
bool
EclipseAgentThread::load_file( const char* filename )
{
  if ( !m_initialized )  { return false; }

  char* ensure_loaded = strdup( "ensure_loaded" );
  post_goal( term( EC_functor( ensure_loaded, 1 ), filename ) );
  free( ensure_loaded );

  if ( EC_succeed != ec_resume() )
  { throw Exception( "File %s could not be loaded", filename ); }

  return true;
}

/** Register an interface for access from within the ECLiPSe context.
 * @param interface the interface to register
 * @return false if the ECLiPSe context hasn't been intialized yet
 */
bool
EclipseAgentThread::register_interface( fawkes::Interface* interface )
{
  if ( !m_initialized ) { return false; }

  m_registered_interfaces[ string( interface->id() ) ] = interface;


  // define structs for interface data ----------------------------------
  // data_IntefaceType(field1, field2, ...) -----------------------------
  
  // check whether struct is already defined
  char* struct_name;
  asprintf( &struct_name, "data_%s", interface->type() );

  post_goal( term( EC_functor( (char *) "current_struct", 2 ),
		   EC_atom( struct_name ),
		   newvar() ) );

  if ( EC_succeed != ec_resume() )
  {
    // define named structure
    // data_InterfaceType( field1, field2, ... )

    vector< string > fields;
    for ( InterfaceFieldIterator i = interface->fields();
	  i != interface->fields_end();
	  ++i )
    { fields.push_back( i.get_name() ); }

    EC_word args[ fields.size() ];

    for ( size_t i = 0 ; i < fields.size(); ++i )
    {
      char* c = strdup( fields.at( i ).c_str() );
      args[ i ] = EC_atom( c );
      free( c );
    }

    EC_word new_struct = term( EC_functor( struct_name, (int) fields.size() ), args );
    
    char* local = strdup( "local" );
    char* strct = strdup( "struct" );
    EC_word struct_def = term( EC_functor( strct, 1 ), new_struct );
    EC_word struct_def_local = term( EC_functor( local, 1), struct_def );

    char* call = strdup( "call" );
    // call( struct( data_InterfaceType(field1, field2, ...) ) )
    post_goal( term( EC_functor( call, 1 ), struct_def_local ) );

    // cleanup
    free( local );
    free( strct );
    free( call );
    
    if ( EC_succeed != ec_resume() )
    { throw Exception( "Failed to define structure %s", struct_name ); }
  }

  free( struct_name );

  
  // define structs for message data ------------------------------------
  // data_IntefaceType_MessageType(field1, field2, ...) -----------------
  
  std::list<const char *> message_types = interface->get_message_types();
  for ( std::list<const char *>::iterator type_iter = message_types.begin();
	type_iter != message_types.end();
	++type_iter )
  {
    // check whether struct is already defined
    char* struct_name;
    asprintf( &struct_name, "data_%s_%s", interface->type(), *type_iter );
    
    post_goal( term( EC_functor( (char *) "current_struct", 2 ),
		     EC_atom( struct_name ),
		     newvar() ) );

    if ( EC_succeed != ec_resume() )
    {
      // define name structure
      // data_InterfaceType_MessageType( field1, field2, ... )

      Message* msg = interface->create_message( *type_iter );

      vector< string > fields;
      for ( InterfaceFieldIterator field_iter = msg->fields();
	    field_iter != msg->fields_end();
	    ++field_iter )
      { 
	string name = field_iter.get_name();
	fields.push_back( name );
      }

      delete msg;

      EC_word args[ fields.size() ];

      for ( size_t i = 0; i < fields.size(); ++i )
      {
	char* c = strdup( fields.at( i ).c_str() );
	args[ i ] = EC_atom( c );
	free( c );
      }

      if ( 0 != fields.size() )
      {
	EC_word new_struct = term( EC_functor( struct_name, (int) fields.size() ), args );
	char* local = strdup( "local" );
	char* strct = strdup( "struct" );
	EC_word struct_def = term( EC_functor( strct, 1 ), new_struct );
	EC_word struct_def_local = term( EC_functor( local, 1), struct_def );

	char* call = strdup( "call" );
	// call( struct( data_InterfaceType_MessageType(field1, field2, ...) ) )
	post_goal( term( EC_functor( call, 1 ), struct_def_local ) );

	// cleanup
	free( local );
	free( strct );
	free( call );
      
	if ( EC_succeed != ec_resume() )
	{ throw Exception( "Failed to define structure %s", struct_name ); }
      }
    }
    
    free( struct_name );
  }

  return true;
}

/** Get the registered interface with the given id.
 * @param id the interface id
 * @return the interface or NULL if no interface with the given id is registerd
 */
fawkes::Interface*
EclipseAgentThread::get_registered_interface( const char* id )
{
  map< string, fawkes::Interface* >::iterator i = m_registered_interfaces.find( string( id ) );

  if ( i == m_registered_interfaces.end() ) { return NULL; }

  return i->second;
}

/** Get the logger.
 * @return the logger
 */
fawkes::Logger*
EclipseAgentThread::get_logger()
{
  return logger;
}

/** Get the EclipseAgentThread instance.
 * @return the instance
 */
EclipseAgentThread*
EclipseAgentThread::instance()
{
  if ( !m_instance )
  { throw Exception( "No instance of type EclipseThread instantiated" ); }

  return m_instance;
}
