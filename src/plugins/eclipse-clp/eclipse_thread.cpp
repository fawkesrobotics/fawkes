
/***************************************************************************
 *  eclipse_thread.cpp - Fawkes ECLiPSe Thread
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
#include "externals/blackboard.h"
#include "externals/fawkes_logger.h"

#include <interfaces/TestInterface.h>
#include <core/threading/mutex_locker.h>
#include <core/exception.h>
#include <eclipseclass.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

namespace fawkes{
  class EclExternalBlackBoard;
};

using namespace std;
using namespace fawkes;

/** @class EclipseAgentThread "eclipse_thread.h"
 * This thread creates an ECLiPSe context in which the ECLiPSe
 * interpreter and the program are loaded.
 * @author Daniel Beck
 */

extern "C" int ec_external( dident, int (*) (), dident );

EclipseAgentThread* EclipseAgentThread::m_instance = NULL;

/** Constructor. */
EclipseAgentThread::EclipseAgentThread()
  : Thread( "ECLiPSe thread", fawkes::Thread::OPMODE_WAITFORWAKEUP ),
    m_initialized( false )
{
  m_instance = this;
  mutex = new fawkes::Mutex();
}

/** Destructor. */
EclipseAgentThread::~EclipseAgentThread()
{
  delete mutex;
}

void
EclipseAgentThread::init()
{
  _running = false;
  fawkes::EclExternalBlackBoard::create_initial_object(blackboard);
  // set ECLiPSe installation directory
  char* eclipse_dir = NULL;
  try
  {
    eclipse_dir = strdup( config->get_string( "/eclipse-clp/eclipse_dir" ).c_str() );
    logger->log_info( name(), "Setting ECLIPSEDIR to %s", eclipse_dir );
    ec_set_option_ptr( EC_OPTION_ECLIPSEDIR, (void*) eclipse_dir );
  }
  catch (...)
  {
    // ignore
  }
  

  agent = strdup( config->get_string( "/eclipse-clp/agent" ).c_str() );

  try{
  //set default module in which goals called from the top-level will be executed
  ec_set_option_ptr(EC_OPTION_DEFAULT_MODULE, (void*) agent.c_str());
    
  }
  catch (...){
    throw fawkes::Exception( "Failed to set default ECLiPSe module");
  }
  // initialize ECLiPSe context
  if ( 0 != ec_init() )
  { throw fawkes::Exception( "Failed to initialize ECLiPSe context" ); }

  free( eclipse_dir );


  m_initialized = true;



  // load utility predicates
  //load_file( ECLIPSE_CODE_DIR"/utils/logging.ecl" );
  std::string agent_path = ECLIPSE_CODE_DIR"/interpreter/"+ agent +".ecl";
  // load interpreter and agent
  load_file( agent_path.c_str() );

   // register external predicates
	if ( EC_succeed != ec_external( ec_did( "log",             2 ), p_log,             ec_did( agent.c_str(), 0 ) ) )
  { throw Exception( "Registering external predicate log/2 failed" ); }

}

void
EclipseAgentThread::finalize()
{
  ec_cleanup();
}

/*
void
EclipseAgentThread::once()
{
  post_goal( "run" );
  if ( EC_succeed != EC_resume() )
  { throw Exception( "Error running agent program" ); }
}
*/

bool EclipseAgentThread::running()
{
  MutexLocker lock(mutex);
  return _running;
}

void
EclipseAgentThread::loop()
{
  if (!running()){
    mutex->lock();
    _running = true;
    mutex->unlock();

    post_goal( "cycle" );
    if ( EC_succeed != EC_resume() )
    { throw Exception( "Error running agent program" ); }

    mutex->lock();
    _running = false;
    mutex->unlock();
  }
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
