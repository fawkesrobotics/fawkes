
/***************************************************************************
 *  logger.h - Fawkes logging interface
 *
 *  Created: Tue Jan 16 20:36:32 2007
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

#ifndef __UTILS_LOGGING_LOGGER_H_
#define __UTILS_LOGGING_LOGGER_H_

#include <core/exception.h>
#include <cstdarg>
#include <sys/time.h>

#if defined(FAWKES_NO_LOGGING_FORMAT_CHECK)
#define FAKWES_LOGGING_FORMAT_CHECK(string, arguments)
#else
#define FAKWES_LOGGING_FORMAT_CHECK(string, arguments) __attribute__((__format__(printf, string, arguments)))
#endif

namespace fawkes {


class Logger
{
 public:

  /** Log level.
   * Defines a level that can be used to determine the amount of output to
   * generate in loggers. The log levels are strictly ordered
   * (debug < info < warn < error < none) so loggers shall implement a
   * facility to set a minimum logging level. Messages below that minimum
   * log level shall be omitted.
   */
  typedef enum {
    LL_DEBUG  = 0,	/**< debug output, relevant only when tracking down problems */
    LL_INFO   = 1,	/**< informational output about normal procedures */
    LL_WARN   = 2,	/**< warning, should be investigated but software still functions,
			 * an example is that something was requested that is not
			 * available and thus it is more likely a user error */
    LL_ERROR  = 4,	/**< error, may be recoverable (software still running) or not
			 * (software has to terminate). This shall be used if the error
			 * is a rare situation that should be investigated. */
    LL_NONE   = 8	/**< use this to disable log output */
  } LogLevel;

  Logger(LogLevel log_level = LL_DEBUG);
  virtual ~Logger();

  virtual void set_loglevel(LogLevel level);
  virtual LogLevel loglevel();

  FAKWES_LOGGING_FORMAT_CHECK(4, 5)
  virtual void log(LogLevel level,
		   const char *component, const char *format, ...);
  FAKWES_LOGGING_FORMAT_CHECK(3, 4)
  virtual void log_debug(const char *component, const char *format, ...)   = 0;
  FAKWES_LOGGING_FORMAT_CHECK(3, 4)
  virtual void log_info(const char *component, const char *format, ...)    = 0;
  FAKWES_LOGGING_FORMAT_CHECK(3, 4)
  virtual void log_warn(const char *component, const char *format, ...)    = 0;
  FAKWES_LOGGING_FORMAT_CHECK(3, 4)
  virtual void log_error(const char *component, const char *format, ...)   = 0;


  virtual void log(LogLevel level, const char *component, Exception &e);
  virtual void log_debug(const char *component, Exception &e)              = 0;
  virtual void log_info(const char *component, Exception &e)               = 0;
  virtual void log_warn(const char *component, Exception &e)               = 0;
  virtual void log_error(const char *component, Exception &e)              = 0;

  virtual void vlog(LogLevel level, const char *component,
		    const char *format, va_list va);
  virtual void vlog_debug(const char *component,
			  const char *format, va_list va)                  = 0;
  virtual void vlog_info(const char *component,
			 const char *format, va_list va)                   = 0;
  virtual void vlog_warn(const char *component,
			 const char *format, va_list va)                   = 0;
  virtual void vlog_error(const char *component,
			  const char *format, va_list va)                  = 0;


  FAKWES_LOGGING_FORMAT_CHECK(5, 6)
  virtual void tlog(LogLevel level, struct timeval *t,
		    const char *component, const char *format, ...);
  FAKWES_LOGGING_FORMAT_CHECK(4, 5)
  virtual void tlog_debug(struct timeval *t, const char *component,
			  const char *format, ...)                         = 0;
  FAKWES_LOGGING_FORMAT_CHECK(4, 5)
  virtual void tlog_info(struct timeval *t, const char *component,
			 const char *format, ...)                          = 0;
  FAKWES_LOGGING_FORMAT_CHECK(4, 5)
  virtual void tlog_warn(struct timeval *t, const char *component,
			 const char *format, ...)                          = 0;
  FAKWES_LOGGING_FORMAT_CHECK(4, 5)
  virtual void tlog_error(struct timeval *t, const char *component,
			  const char *format, ...)                         = 0;

  virtual void tlog(LogLevel level, struct timeval *t, const char *component,
		    Exception &e);
  virtual void tlog_debug(struct timeval *t, const char *component,
			  Exception &e)                                    = 0;
  virtual void tlog_info(struct timeval *t, const char *component,
			 Exception &e)                                     = 0;
  virtual void tlog_warn(struct timeval *t, const char *component,
			 Exception &e)                                     = 0;
  virtual void tlog_error(struct timeval *t, const char *component,
			  Exception &e)                                    = 0;

  virtual void vtlog(LogLevel level, struct timeval *t, const char *component,
		     const char *format, va_list va);
  virtual void vtlog_debug(struct timeval *t, const char *component,
			   const char *format, va_list va)                  = 0;
  virtual void vtlog_info(struct timeval *t, const char *component,
			  const char *format, va_list va)                   = 0;
  virtual void vtlog_warn(struct timeval *t, const char *component,
			  const char *format, va_list va)                   = 0;
  virtual void vtlog_error(struct timeval *t, const char *component,
			   const char *format, va_list va)                  = 0;


 protected:
  /** Minimum log level.
   * A logger shall only log output with a level equal or above the given level,
   * it shall ignore all other messages.
   */
  LogLevel log_level;
};


} // end namespace fawkes

#endif
