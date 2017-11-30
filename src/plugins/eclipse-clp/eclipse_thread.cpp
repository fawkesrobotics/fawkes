
/***************************************************************************
 *  eclipse_thread.cpp - Fawkes ECLiPSe Thread
 *
 *  Created: Wed Jul 16 10:42:49 2009
 *  Copyright  2009      Daniel Beck
 *             2013-2014 Gesche Gierse
 *             2014      Tim Niemueller
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
#include "externals/eclipse_path.h"
#include "externals/eclipseclp_config.h"
#include "blackboard_listener_thread.h"

#include <interfaces/TestInterface.h>
#include <core/threading/mutex_locker.h>
#include <core/exception.h>

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

extern "C" int ec_external( dident, int (*) (...), dident );

EclipseAgentThread* EclipseAgentThread::m_instance = NULL;

/** Constructor. */
EclipseAgentThread::EclipseAgentThread()
  : Thread( "ECLiPSe thread", fawkes::Thread::OPMODE_CONTINUOUS ),
    m_initialized( false )
{
  set_prepfin_conc_loop(true);
  m_instance = this;
  mutex = new fawkes::Mutex();
}

/** Destructor. */
EclipseAgentThread::~EclipseAgentThread()
{
  if (EclExternalBlackBoard::instance()) {
    logger->log_info(name(), "Cleaning up");
    EclExternalBlackBoard::cleanup_instance();
  }
  delete mutex;
}

void
EclipseAgentThread::init()
{
  fawkes::EclExternalBlackBoard::create_initial_object(blackboard, logger);
  fawkes::EclExternalConfig::create_initial_object(config);
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

  agent = config->get_string( "/eclipse-clp/agent" );

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

  std::vector<std::string> paths = config->get_strings("/eclipse-clp/file_path");

  // initialise pathfinding utility
  EclipsePath::create_initial_object();
  EclipsePath::instance()->add_regex(boost::regex("@AGENT@"), agent);
  for (size_t i = 0; i < paths.size(); ++i) {
    EclipsePath::instance()->add_path(paths[i]);
  }

  EclipsePath::instance()->apply_regexes();

  // debug
  EclipsePath::instance()->print_all_paths();

  // make locate_file/2 available
  std::string filepath_path = EclipsePath::instance()->locate_file("filepath.ecl");
  if (filepath_path.empty()) {
    throw Exception("Failed to determine path to filepath module");
  }
  load_file(filepath_path.c_str());
  char *filepath = ::strdup("filepath");
  post_goal(term(EC_functor(":", 2), EC_atom(filepath),
                 term(EC_functor("add_library_path", 1),
                      ::list(EC_word(SRCDIR "/externals"),
                             ::list(EC_word(SRCDIR "/utils"),
                                    ::list(EC_word(SRCDIR "/consoletool"),
                                           ::list(EC_word(SRCDIR "/interpreter"), nil()) ) ) ) )
  ) );
  if (EC_succeed != EC_resume())
    throw Exception("Failed to add " SRCDIR "/externals to library path");

  // check if navgraph is used and pass config value
  if (config->get_bool( ("/eclipse-clp/"+agent+"/use_graph").c_str() )){
    graph_path = CONFDIR + config->get_string( ("/eclipse-clp/"+agent+"/rel_graph_path").c_str());

    logger->log_info( name(), "Setting graph_path to %s", graph_path.c_str() );
    post_goal( term(EC_functor("load_graph",1), graph_path.c_str()) );
    if ( EC_succeed != EC_resume() )
    { throw Exception( "Error loading graph config to agent" ); }
  }

  // load interpreter and agent
  std::string agent_path = EclipsePath::instance()->locate_file(agent + ".ecl");
  if (agent_path.empty()) {
    throw Exception("Failed to determine path to agent module");
  }
  load_file( agent_path.c_str() );

   // register external predicates
       if ( EC_succeed != ec_external( ec_did( "log",             2 ), p_log,             ec_did( agent.c_str(), 0 ) ) )
  { throw Exception( "Registering external predicate log/2 failed" ); }

}

void
EclipseAgentThread::finalize()
{
  ec_cleanup();
  if (EclExternalBlackBoard::instance())
    fawkes::EclExternalBlackBoard::cleanup_instance();
}


void
EclipseAgentThread::once()
{
  post_goal("run");
  ec_result = EC_resume("init", ec_yield_reason);
}


void
EclipseAgentThread::loop()
{
  if (ec_result == EC_status::EC_yield) {
    EC_word bb_updates(::nil());
    if (EC_word(ec_yield_reason) == EC_atom("exogenous_update")) {
      while (BlackboardListenerThread::instance()->event_pending())
        bb_updates = ::list(bb_updates, *BlackboardListenerThread::instance()->event_pop());
    }
    else if (BlackboardListenerThread::instance()->event_pending())
      post_event("event_exogUpdate");

    ec_result = EC_resume(bb_updates, ec_yield_reason);
  }
  else {
    if (ec_result == EC_status::EC_succeed)
      logger->log_warn(name(), "Agent program terminated successfully.");
    else
      logger->log_error(name(), "Agent program failed.");

    logger->log_warn(name(), "Stopping Agent thread.");
    exit();
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
