
/***************************************************************************
 *  logger.cpp - Fawkes logging interface
 *
 *  Created: Tue Jan 16 20:40:15 2007
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

#include <logging/logger.h>

/**
 * @def FAKWES_LOGGING_FORMAT_CHECK
 * Activates semantic format checking for the following function. You have
 * to specifiy which arguments have to be considered for checking. for
 * example for the member function @code Logger::log_info(const char *cat,
 * const char *format, ...) @endcode the arguments are @c format and ...
 * @note For counting the arguments on member functions you have to consider
 *       the implicit @c this as first argument.
 * @note If you want the checks to be deactivated, because you are too lazy
 *       to fix your warnings, define FAWKES_NO_LOGGING_FORMAT_CHECK. Same
 *       if your compiler does not support the syntax.
 * @param[in] string The argument number of the format string. Starting by 1.
 * @param[in] arguments The argument number of the arguments for the format
 *                      string. Starting by 1.
 */

namespace fawkes {

/** @class Logger <logging/logger.h>
 * Interface for logging.
 * This interface facilitates a way to collect all output, be it debugging
 * output, informational output, warning or error messages.
 *
 * There should be no need no more for usage of printf in the code but
 * rather a logger should be used instead.
 *
 * The LoggingAspect should be added to a Thread that has to log something
 * (which is likely to be the case).
 * For library use QuickLog is the recommended way of logging. Do NOT use
 * these in plugins as it hides a dependency of this plugin.
 *
 * A special note to logging hackers: A logger may never ever bounce. This
 * means that not printing a message is ok in case of an internal error in
 * the logger, but it may never indicate that error with an exception!
 * If a logger cannot deliver the messages as it should be (like a network
 * logger that cannot send because the connection is dead) it should at
 * least dump it to stderr!
 *
 * Loggers have to be fast - damn fast. If a lengthy operations is needed
 * (like a network connection that may block) messages shall be enqueued
 * and processed later (for example in a separate thread). This is because
 * everywhere in the software (even in libraries like the utils) a logger
 * may be used to log an error that occured (but which is not that critical
 * that the application should die). In that case a logger which takes to
 * long is absolutely the wrong thing because this would influence the
 * performance of the whole software at unpredicted times - while if the
 * operations are carried out at a specified time or in a separate thread
 * they do not harm the performance.
 *
 * Caution: The line between log_* methods and vlog_* methods is very thin.
 * You can actually call log_info() with a va_list as the only variadic
 * parameter in some cases. The call is syntactically correct, but the
 * result is not what you intended. Thus make sure that you always use the
 * vlog_* method if you pass along a va_list!
 *
 * @fn void Logger::log_debug(const char *component, const char *format, ...) = 0
 * Log debug message.
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 *
 * @fn void Logger::log_info(const char *component, const char *format, ...) = 0
 * Log informational message.
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 *
 * @fn void Logger::log_warn(const char *component, const char *format, ...) = 0
 * Log warning message.
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 *
 * @fn void Logger::log_error(const char *component, const char *format, ...) = 0
 * Log error message.
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 *
 * @fn void Logger::vlog_debug(const char *component, const char *format, va_list va) = 0
 * Log debug message.
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 * @param va variable argument list
 *
 * @fn void Logger::vlog_info(const char *component, const char *format, va_list va) = 0
 * Log informational message.
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 * @param va variable argument list
 *
 * @fn void Logger::vlog_warn(const char *component, const char *format, va_list va) = 0
 * Log warning message.
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 * @param va variable argument list
 *
 * @fn void Logger::vlog_error(const char *component, const char *format, va_list va) = 0
 * Log error message.
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 * @param va variable argument list
 *
 * @fn void Logger::log_debug(const char *component, Exception &e) = 0
 * Log debug exception.
 * @param component component, used to distuinguish logged messages
 * @param e exception to log, exception messages will be logged
 *
 * @fn void Logger::log_info(const char *component, Exception &e) = 0
 * Log informational exception.
 * @param component component, used to distuinguish logged messages
 * @param e exception to log, exception messages will be logged
 *
 * @fn void Logger::log_warn(const char *component, Exception &e) = 0
 * Log warning exception.
 * @param component component, used to distuinguish logged messages
 * @param e exception to log, exception messages will be logged
 *
 * @fn void Logger::log_error(const char *component, Exception &e) = 0
 * Log error exception.
 * @param component component, used to distuinguish logged messages
 * @param e exception to log, exception messages will be logged
 *
 * @fn void Logger::tlog_debug(struct timeval *t, const char *component, const char *format, ...) = 0
 * Log debug message for specific time.
 * @param t time for this message to log
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 *
 * @fn void Logger::tlog_info(struct timeval *t, const char *component, const char *format, ...) = 0
 * Log informational message for specific time.
 * @param t time for this message to log
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 *
 * @fn void Logger::tlog_warn(struct timeval *t, const char *component, const char *format, ...) = 0
 * Log warning message for specific time.
 * @param t time for this message to log
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 *
 * @fn void Logger::tlog_error(struct timeval *t, const char *component, const char *format, ...) = 0
 * Log error message for specific time.
 * @param t time for this message to log
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 *
 * @fn void Logger::tlog_debug(struct timeval *t, const char *component, Exception &e) = 0
 * Log debug exception for specific time.
 * @param t time for this message to log
 * @param component component, used to distuinguish logged messages
 * @param e exception to log, exception messages will be logged
 *
 * @fn void Logger::tlog_info(struct timeval *t, const char *component, Exception &e) = 0
 * Log informational exception for specific time.
 * @param t time for this message to log
 * @param component component, used to distuinguish logged messages
 * @param e exception to log, exception messages will be logged
 *
 * @fn void Logger::tlog_warn(struct timeval *t, const char *component, Exception &e) = 0
 * Log warning exception for specific time.
 * @param t time for this message to log
 * @param component component, used to distuinguish logged messages
 * @param e exception to log, exception messages will be logged
 *
 * @fn void Logger::tlog_error(struct timeval *t, const char *component, Exception &e) = 0
 * Log error exception for specific time.
 * @param t time for this message to log
 * @param component component, used to distuinguish logged messages
 * @param e exception to log, exception messages will be logged
 *
 * @fn void Logger::vtlog_debug(struct timeval *t, const char *component, const char *format, va_list va) = 0
 * Log debug message for specific time.
 * @param t time for this message to log
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 * @param va variable argument list
 *
 * @fn void Logger::vtlog_info(struct timeval *t, const char *component, const char *format, va_list va) = 0
 * Log informational message for specific time.
 * @param t time for this message to log
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 * @param va variable argument list
 *
 * @fn void Logger::vtlog_warn(struct timeval *t, const char *component, const char *format, va_list va) = 0
 * Log warning message for specific time.
 * @param t time for this message to log
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 * @param va variable argument list
 *
 * @fn void Logger::vtlog_error(struct timeval *t, const char *component, const char *format, va_list va) = 0
 * Log error message for specific time.
 * @param t time for this message to log
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 * @param va variable argument list
 *
 */

/** Constructor.
 * @param log_level log level
 */
Logger::Logger(LogLevel log_level)
{
  this->log_level = log_level;
}


/** Virtual empty destructor. */
Logger::~Logger()
{
}


/** Sets the log level.
 * The log level determines the minimum log level. If a message is logged that
 * is below this level the message is ignored.
 * @param level new log level
 */
void
Logger::set_loglevel(LogLevel level)
{
  log_level = level;
}


/** Get log level.
 * @return current log level.
 */
Logger::LogLevel
Logger::loglevel()
{
  return log_level;
}


/** Log message for given log level.
 * @param level log level
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 * @param va variadic argument list
 */
void
Logger::vlog(LogLevel level,
	     const char *component, const char *format, va_list va)
{
  if ( log_level <= level ) {
    switch (level) {
    case LL_DEBUG:  vlog_debug(component, format, va);  break;
    case LL_INFO:   vlog_info(component, format, va);   break;
    case LL_WARN:   vlog_warn(component, format, va);   break;
    case LL_ERROR:  vlog_error(component, format, va);  break;
    default: break;
    }
  }
}



/** Log message for given log level and time.
 * @param level log level
 * @param t time
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 * @param va variadic argument list
 */
void
Logger::vtlog(LogLevel level, struct timeval *t,
	      const char *component, const char *format, va_list va)
{
  if ( log_level <= level ) {
    switch (level) {
    case LL_DEBUG:  vtlog_debug(t, component, format, va);  break;
    case LL_INFO:   vtlog_info(t, component, format, va);   break;
    case LL_WARN:   vtlog_warn(t, component, format, va);   break;
    case LL_ERROR:  vtlog_error(t, component, format, va);  break;
    default: break;
    }
  }
}


/** Log message of given log level.
 * @param level log level
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 */
void
Logger::log(LogLevel level, const char *component, const char *format, ...)
{
  if ( log_level <= level ) {
    va_list va;
    va_start(va, format);
    vlog(level, component, format, va);
    va_end(va);
  }
}


/** Log exception for given log level.
 * @param level log level
 * @param component component, used to distuinguish logged messages
 * @param e exception to log, exception messages will be logged
 */
void
Logger::log(LogLevel level, const char *component, Exception &e)
{
  if ( log_level <= level ) {
    switch (level) {
    case LL_DEBUG:  log_debug(component, e);  break;
    case LL_INFO:   log_info(component, e);   break;
    case LL_WARN:   log_warn(component, e);   break;
    case LL_ERROR:  log_error(component, e);  break;
    default: break;
    }
  }
}



/** Log message of given log level and time.
 * @param t time
 * @param level log level
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 */
void
Logger::tlog(LogLevel level, struct timeval *t,
	     const char *component, const char *format, ...)
{
  if ( log_level <= level ) {
    va_list va;
    va_start(va, format);
    vtlog(level, t, component, format, va);
    va_end(va);
  }
}


/** Log exception for given log level.
 * @param t time
 * @param level log level
 * @param component component, used to distuinguish logged messages
 * @param e exception to log, exception messages will be logged
 */
void
Logger::tlog(LogLevel level, struct timeval *t, const char *component, Exception &e)
{
  if ( log_level <= level ) {
    switch (level) {
    case LL_DEBUG:  tlog_debug(t, component, e);  break;
    case LL_INFO:   tlog_info(t, component, e);   break;
    case LL_WARN:   tlog_warn(t, component, e);   break;
    case LL_ERROR:  tlog_error(t, component, e);  break;
    default: break;
    }
  }
}


} // end namespace fawkes
