
/***************************************************************************
 *  logger.h - Fawkes logging interface
 *
 *  Created: Tue Jan 16 20:36:32 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#ifndef __UTILS_LOGGING_LOGGER_H_
#define __UTILS_LOGGING_LOGGER_H_

#include <core/exception.h>
#include <cstdarg>

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
    DEBUG  = 0,	/**< debug output, relevant only when tracking down problems */
    INFO   = 1,	/**< informational output about normal procedures */
    WARN   = 2,	/**< warning, should be investigated but software still functions,
		 * an example is that something was requested that is not
		 * available and thus it is more likely a user error */
    ERROR  = 4,	/**< error, may be recoverable (software still running) or not
		 * (software has to terminate). This shall be used if the error
		 * is a rare situation that should be investigated. */
    NONE   = 8	/**< use this to disable log output */
  } LogLevel;

  virtual ~Logger();

  virtual void log(LogLevel level,
		   const char *component, const char *format, ...)         = 0;
  virtual void log_debug(const char *component, const char *format, ...)   = 0;
  virtual void log_info(const char *component, const char *format, ...)    = 0;
  virtual void log_warn(const char *component, const char *format, ...)    = 0;
  virtual void log_error(const char *component, const char *format, ...)   = 0;

  virtual void vlog(LogLevel level, const char *component,
		    const char *format, va_list va)                  = 0;
  virtual void vlog_debug(const char *component,
			  const char *format, va_list va)                  = 0;
  virtual void vlog_info(const char *component,
			 const char *format, va_list va)                   = 0;
  virtual void vlog_warn(const char *component,
			 const char *format, va_list va)                   = 0;
  virtual void vlog_error(const char *component,
			  const char *format, va_list va)                  = 0;

  virtual void log(LogLevel level, const char *component, Exception &e)    = 0;
  virtual void log_debug(const char *component, Exception &e)              = 0;
  virtual void log_info(const char *component, Exception &e)               = 0;
  virtual void log_warn(const char *component, Exception &e)               = 0;
  virtual void log_error(const char *component, Exception &e)              = 0;


};

#endif
