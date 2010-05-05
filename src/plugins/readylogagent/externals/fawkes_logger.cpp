
/***************************************************************************
 *  fawkes_logger.cpp - External predicates that allow the usage of the Logger
 *
 *  Created: Wed Jul 22 11:25:21 2009
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

#include "fawkes_logger.h"
#include <plugins/readylogagent/eclipse_thread.h>

#include <utils/logging/logger.h>
#include <core/exception.h>

#include <eclipseclass.h>

#include <cstring>
#include <cstdio>

int
p_log()
{
  // log(+LogLevel, +LogString)

  fawkes::Logger* logger;
  try
  {
    logger = EclipseAgentThread::instance()->get_logger();
  }
  catch ( fawkes::Exception& e )
  {
    e.print_trace();
    return EC_fail;
  }

  EC_atom log_level;
  if ( EC_succeed != EC_arg( 1 ).is_atom( &log_level ) )
  {
    printf( "Could not obtain log level\n" );
    return EC_fail;
  }

  fawkes::Logger::LogLevel ll;
  if ( 0 == strcmp( "ll_debug", log_level.name() ) )
  {
    ll = fawkes::Logger::LL_DEBUG;
  }
  else if ( 0 == strcmp( "ll_info", log_level.name() ) )
  {
    ll = fawkes::Logger::LL_INFO;
  }
  else if ( 0 == strcmp( "ll_warn", log_level.name() ) )
  {
    ll = fawkes::Logger::LL_WARN;
  }
  else if ( 0 == strcmp( "ll_error", log_level.name() ) )
  {
    ll = fawkes::Logger::LL_ERROR;
  }
  else
  {
    printf( "Unknown log level %s\n", log_level.name() );
    return EC_fail;
  }

  char* log_string;
  if ( EC_succeed != EC_arg( 2 ).is_string( &log_string ) )
  {
    printf( "Could not get 2nd argument of log/2\n" );
    return EC_fail;
  }

  logger->log( ll, "ReadylogAgent", log_string );

  return EC_succeed;
}
