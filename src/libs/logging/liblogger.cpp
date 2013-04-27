
/***************************************************************************
 *  liblogger.h - Fawkes lib logger
 *
 *  Created: Mon May 07 15:22:18 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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

#include <logging/liblogger.h>
#include <logging/multi.h>
#include <logging/console.h>

#include <core/exceptions/software.h>
#include <core/threading/mutex.h>

namespace fawkes {

/** @class LibLogger <logging/liblogger.h>
 * Library logger.
 * This logger is meant to be used in libraries that depend on utils anyway
 * and in utils itself. This logger is completely static so it only has to be
 * initialized once per process. If the logger is used before it has been
 * initialized it is automatically initialized with an empty MultiLogger.
 * If you want to see output you have to make sure that you add loggers
 * like the ConsoleLogger.
 *
 * Make sure that you call finalize() at the end of the surrounding process
 * to free all the loggers associcated with the internal multi logger and
 * the multi logger itself.
 *
 * @see MultiLogger
 * @author Tim Niemueller
 */


/** The internal multi logger. */
MultiLogger *  LibLogger::logger = NULL;
/** Internal mutex */
Mutex *        LibLogger::mutex  = NULL;


/** Initialize logger.
 * @param multi_logger Logger to use in this multi logger. If NULL a new
 * logger is created. Note that LibLogger takes over ownership of the
 * multi logger and will destroy it if finalize() is called.
 */
void
LibLogger::init(MultiLogger *multi_logger)
{
  if ( logger != NULL ) {
    throw AccessViolationException("LibLogger already initialized");
  }
  mutex = new Mutex();
  if ( multi_logger == NULL ) {
    logger = new MultiLogger(new ConsoleLogger());
  } else {
    logger = multi_logger;
  }
  
}


/** Delete internal logger.
 * Note that the multi logger took over ownership of the loggers.
 * @see MultiLogger
 */
void
LibLogger::finalize()
{
  delete logger;
  delete mutex;
  logger = NULL;
  mutex  = NULL;
}


/** Add logger.
 * @param l sub-logger to add
 * @see MultiLogger::add_logger()
 */
void
LibLogger::add_logger(Logger *l)
{
  if ( logger == NULL )  init();
  mutex->lock();
  logger->add_logger(l);
  mutex->unlock();
}


/** Remove logger.
 * @param l sub-logger to remove
 * @see MultiLogger::remove_logger()
 */
void
LibLogger::remove_logger(Logger *l)
{
  if ( logger == NULL )  init();
  mutex->lock();
  logger->remove_logger(l);
  mutex->unlock();
}


/** Log debug message.
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 */
void
LibLogger::log_debug(const char *component, const char *format, ...)
{
  if ( logger == NULL )  init();
  mutex->lock();
  va_list va;
  va_start(va, format);
  logger->vlog_debug(component, format, va);
  va_end(va);
  mutex->unlock();
}


/** Log informational message.
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 */
void
LibLogger::log_info(const char *component, const char *format, ...)
{
  if ( logger == NULL )  init();
  mutex->lock();
  va_list va;
  va_start(va, format);
  logger->vlog_info(component, format, va);
  va_end(va);
  mutex->unlock();
}


/** Log warning message.
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 */
void
LibLogger::log_warn(const char *component, const char *format, ...)
{
  if ( logger == NULL )  init();
  mutex->lock();
  va_list va;
  va_start(va, format);
  logger->vlog_warn(component, format, va);
  va_end(va);
  mutex->unlock();
}


/** Log error message.
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 */
void
LibLogger::log_error(const char *component, const char *format, ...)
{
  if ( logger == NULL )  init();
  mutex->lock();
  va_list va;
  va_start(va, format);
  logger->vlog_error(component, format, va);
  va_end(va);
  mutex->unlock();
}


/** Log debug message.
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 * @param va variadic argument list
 */
void
LibLogger::vlog_debug(const char *component, const char *format, va_list va)
{
  if ( logger == NULL )  init();
  mutex->lock();
  logger->vlog_debug(component, format, va);
  mutex->unlock();
}


/** Log informational message.
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 * @param va variadic argument list
 */
void
LibLogger::vlog_info(const char *component, const char *format, va_list va)
{
  if ( logger == NULL )  init();
  mutex->lock();
  logger->vlog_info(component, format, va);
  mutex->unlock();
}


/** Log warning message.
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 * @param va variadic argument list
 */
void
LibLogger::vlog_warn(const char *component, const char *format, va_list va)
{
  if ( logger == NULL )  init();
  mutex->lock();
  logger->vlog_warn(component, format, va);
  mutex->unlock();
}


/** Log error message.
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 * @param va variadic argument list
 */
void
LibLogger::vlog_error(const char *component, const char *format, va_list va)
{
  if ( logger == NULL )  init();
  mutex->lock();
  logger->vlog_error(component, format, va);
  mutex->unlock();
}



/** Log debug message.
 * @param component component, used to distuinguish logged messages
 * @param e exception to log, exception messages will be logged
 */
void
LibLogger::log_debug(const char *component, Exception &e)
{
  if ( logger == NULL )  init();
  mutex->lock();
  logger->log_debug(component, e);
  mutex->unlock();
}

/** Log informational message.
 * @param component component, used to distuinguish logged messages
 * @param e exception to log, exception messages will be logged
 */
void
LibLogger::log_info(const char *component, Exception &e)
{
  if ( logger == NULL )  init();
  mutex->lock();
  logger->log_info(component, e);
  mutex->unlock();
}


/** Log warning message.
 * @param component component, used to distuinguish logged messages
 * @param e exception to log, exception messages will be logged
 */
void
LibLogger::log_warn(const char *component, Exception &e)
{
  if ( logger == NULL )  init();
  mutex->lock();
  logger->log_warn(component, e);
  mutex->unlock();
}


/** Log error message.
 * @param component component, used to distuinguish logged messages
 * @param e exception to log, exception messages will be logged
 */
void
LibLogger::log_error(const char *component, Exception &e)
{
  if ( logger == NULL )  init();
  mutex->lock();
  logger->log_error(component, e);
  mutex->unlock();
}


} // end namespace fawkes
