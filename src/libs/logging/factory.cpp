
/***************************************************************************
 *  factory.cpp - Logger factory
 *
 *  Created: Mon Jun 04 10:57:21 2007
 *  Copyright  2007-2011  Tim Niemueller [www.niemueller.de]
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

#include <logging/factory.h>
#include <logging/console.h>
#include <logging/file.h>
#include <logging/syslog.h>
#include <logging/multi.h>

#include <cstring>
#include <cstdlib>
#include <string>

namespace fawkes {

/** @class UnknownLoggerTypeException <logging/factory.h>
 * Unknown logger type exception.
 * Thrown if the requested logger has not been recognized.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param msg optional explanation
 */
UnknownLoggerTypeException::UnknownLoggerTypeException(const char *msg)
  : Exception("Unknown logger type")
{
  append(msg);
}


/** @class LoggerFactory <logging/factory.h>
 * Logger factory.
 * This logging factory provides access to all loggers in a unified
 * way. You just supply a logger argument string and depending on the
 * logger type an instance of the desired logger is returned or
 * otherwise an exception is thrown. See instance() for a list of
 * supported logger types.
 *
 * @author Tim Niemueller
 */

/** Convert a string to a log level.
 * @param log_level log level as string
 * @return log level
 */
Logger::LogLevel
LoggerFactory::string_to_loglevel(const char *log_level)
{
  std::string ll = log_level;

  if (ll == "info" || ll == "INFO") {
    return Logger::LL_INFO;
  } else if (ll == "warn" || ll == "WARN") {
    return Logger::LL_WARN;
  } else if (ll == "error" || ll == "ERROR") {
    return Logger::LL_ERROR;
  } else {
    return Logger::LL_DEBUG;
  }
}

/** Get logger instance.
 * Get an instance of a logger of the given type. The argument string is used for
 * logger arguments.
 * Supported logger types:
 * - console, ConsoleLogger
 * - file, FileLogger
 * - syslog, SyslogLogger
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
  } else if ( strcmp(type, "syslog") == 0 ) {
    l = new SyslogLogger(as);
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
 * @param default_ll default log level for multi logger
 * @return multi logger instance with requested loggers
 * @exception UnknownLoggerTypeException thrown if any of the loggers was unknown.
 */
MultiLogger *
LoggerFactory::multilogger_instance(const char *as, Logger::LogLevel default_ll)
{
  MultiLogger *m = new MultiLogger();
  m->set_loglevel(default_ll);

  char *logger_string = strdup(as);
  char *str = logger_string;
  char *saveptr, *r;
  const char *type, *args, *level;
  char *typeargs_saveptr, *level_saveptr, *type_str;
  const char *logger_delim = ";";
  const char *logger_typeargs_delim = ":";
  const char *logger_level_delim = "/";
  while ((r = strtok_r(str, logger_delim, &saveptr)) != NULL ) {
    type  = strtok_r(r, logger_typeargs_delim, &typeargs_saveptr);
    args  = strtok_r(NULL, logger_typeargs_delim, &typeargs_saveptr);

    type_str = strdup(type);

    type  = strtok_r(type_str, logger_level_delim, &level_saveptr);
    level = strtok_r(NULL, logger_level_delim, &level_saveptr);

    if ( type == NULL ) {
      throw UnknownLoggerTypeException();
    }
    if ( args == NULL ) {
      args = "";
    }

    try {
      Logger *l = instance(type, args);
      m->add_logger(l);
      if (level) {
	Logger::LogLevel ll = string_to_loglevel(level);
	l->set_loglevel(ll);
      }
    } catch (Exception &e) {
      e.append("Could not open logger '%s:%s'", type, args);
      free(type_str);
      free(logger_string);
      delete m;
      throw;
    }
    str = NULL;

    free(type_str);
  }

  free(logger_string);

  return m;
}


} // end namespace fawkes
