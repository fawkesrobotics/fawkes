
/***************************************************************************
 *  factory.cpp - Logger factory
 *
 *  Created: Mon Jun 04 10:57:21 2007
 *  Copyright  2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#include <utils/logging/factory.h>
#include <utils/logging/console.h>
#include <utils/logging/file.h>
#include <utils/logging/multi.h>

#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class UnknownLoggerTypeException factory.h <utils/logging/factory.h>
 * Unknown logger type exception.
 * Thrown if the requested logger has not been recognized
 */

/** Constructor.
 * @param msg optional explanation
 */
UnknownLoggerTypeException::UnknownLoggerTypeException(const char *msg)
  : Exception("Unknown logger type")
{
  append(msg);
}


/** @class LoggerFactory factory.h <utils/logging/factory.h>
 * Logger factory.
 * This logging factory provides access to all loggers in a unified way. You just
 * supply a logger argument string and depending on the logger type an instance of
 * the desired logger is returned or otherwise
 * an exception is thrown. See instance() for a list of supported logger types.
 *
 * @author Tim Niemueller
 */

/** Get logger instance.
 * Get an instance of a logger of the given type. The argument string is used for
 * logger arguments.
 * Supported logger types:
 * - console, ConsoleLogger
 * - file, FileLogger
 * NOT supported:
 * - NetworkLogger, needs a FawkesNetworkHub which cannot be passed by parameter
 * @param type logger type
 * @param as logger argument string
 * @return logger instance of requested type
 * @exception UnknownLoggerTypeException thrown, if the desired logger could
 * not be instantiated. This could be a misspelled logger type.
 */
Logger *
LoggerFactory::instance(const char *type, const char *as)
{
  Logger *l = NULL;

  if ( strcmp(type, "console") == 0 ) {
    // no supported arguments
    l = new ConsoleLogger();
  } else if ( strcmp(type, "file") == 0 ) {
    char *tmp = strdup(as);
    char *saveptr;
    char *r = strtok_r(tmp, ":", &saveptr);
    const char *file_name;
    r = strtok_r(tmp, ":", &saveptr);
    if ( r == NULL ) {
      file_name = "unnamed.log";
    } else {
      file_name = r;
    }
    l = new FileLogger(file_name);
    free(tmp);
  }

  if ( l == NULL )  throw UnknownLoggerTypeException();
  return l;
}


/** Create MultiLogger instance.
 * This creates a multi logger instance based on the supplied argument string.
 * The argument string is of the form
 * @code
 *  ltype:largs[;ltype2:largs2[;...]]
 * @endcode
 * So it is a list of logger type/argument tuples separated by columns concatenated
 * to one list with exclamation marks. The list is not pre-processed, so if you
 * mention a logger twice this logger is added twice.
 * @param as logger argument string
 * @return multi logger instance with requested loggers
 * @exception UnknownLoggerTypeException thrown if any of the loggers was unknown.
 */
MultiLogger *
LoggerFactory::multilogger_instance(const char *as)
{
  MultiLogger *m = new MultiLogger();

  char *logger_string = strdup(as);
  char *str = logger_string;
  char *saveptr, *r;
  const char *type, *args;
  char *typeargs_saveptr;
  const char *logger_delim = ";";
  const char *logger_typeargs_delim = ":";
  while ((r = strtok_r(str, logger_delim, &saveptr)) != NULL ) {
    type = strtok_r(r, logger_typeargs_delim, &typeargs_saveptr);
    args = strtok_r(NULL, logger_typeargs_delim, &typeargs_saveptr);
    if ( type == NULL ) {
      throw UnknownLoggerTypeException();
    }
    if ( args == NULL ) {
      args = "";
    }

    try {
      Logger *l = instance(type, args);
      m->add_logger(l);
    } catch (Exception &e) {
      e.append("Could not open logger '%s:%s'", type, args);
      free(logger_string);
      delete m;
      throw;
    }
    str = NULL;
  }

  free(logger_string);

  return m;
}


} // end namespace fawkes
