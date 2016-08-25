
/***************************************************************************
 *  component.cpp - Component logger
 *
 *  Created: Wed Mar 12 23:46:34 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#include <logging/component.h>
#include <logging/logger.h>

#include <cstring>
#include <cstdlib>
#include <cstdio>

namespace fawkes {

/** @class ComponentLogger <logging/component.h>
 * Component logger.
 * This is a small wrapper around a logger to make it simpler to use in a
 * single component. Once initialized it will only accept messages for a
 * specific component string offers a simplified interface to logging methods.
 *
 * @author Tim Niemueller
 */

/** Constructor.
 * @param logger logger to use
 * @param component component string, copied to internal buffer
 */
ComponentLogger::ComponentLogger(Logger *logger, const char *component)
{
  __logger = logger;
  __component = strdup(component);
}


/** Destructor. */
ComponentLogger::~ComponentLogger()
{
  free(__component);
}


/** Set a new component name.
 * @param format format string for the new command string, cf. sprintf
 * man page for allowed syntax.
 */
void
ComponentLogger::set_component(const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  char *new_component;
  if (vasprintf(&new_component, format, arg) > 0) {
	  char *old_component = __component;
	  __component = new_component;
	  free(old_component);
  }
  va_end(arg);
}
	
/** Log debug message.
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 */
void
ComponentLogger::log_debug(const char *format, ...)
{
  va_list va;
  va_start(va, format);
  __logger->vlog_debug(__component, format, va);
  va_end(va);
}


/** Log info message.
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 */
void
ComponentLogger::log_info(const char *format, ...)
{
  va_list va;
  va_start(va, format);
  __logger->vlog_info(__component, format, va);
  va_end(va);
}


/** Log warning message.
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 */
void
ComponentLogger::log_warn(const char *format, ...)
{
  va_list va;
  va_start(va, format);
  __logger->vlog_warn(__component, format, va);
  va_end(va);
}


/** Log error message.
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 */
void
ComponentLogger::log_error(const char *format, ...)
{
  va_list va;
  va_start(va, format);
  __logger->vlog_error(__component, format, va);
  va_end(va);
}


/** Log debug message.
 * @param message message to log
 */
void
ComponentLogger::log_debug(std::string message)
{
  __logger->log_debug(__component, "%s", message.c_str());
}


/** Log info message.
 * @param message message to log
 */
void
ComponentLogger::log_info(std::string message)
{
  __logger->log_info(__component, "%s", message.c_str());
}


/** Log warning message.
 * @param message message to log
 */
void
ComponentLogger::log_warn(std::string message)
{
  __logger->log_warn(__component, "%s", message.c_str());
}


/** Log error message.
 * @param message message to log
 */
void
ComponentLogger::log_error(std::string message)
{
  __logger->log_error(__component, "%s", message.c_str());
}


/** Log exception at debug log level.
 * @param e exception to log, exception messages will be logged
 */
void
ComponentLogger::log_debug(Exception &e)
{
  __logger->log_debug(__component, e);
}


/** Log exception at info log level.
 * @param e exception to log, exception messages will be logged
 */
void
ComponentLogger::log_info(Exception &e)
{
  __logger->log_info(__component, e);
}


/** Log exception at warn log level.
 * @param e exception to log, exception messages will be logged
 */
void
ComponentLogger::log_warn(Exception &e)
{
  __logger->log_warn(__component, e);
}


/** Log exception at error log level.
 * @param e exception to log, exception messages will be logged
 */
void
ComponentLogger::log_error(Exception &e)
{
  __logger->log_debug(__component, e);
}


} // end namespace fawkes
