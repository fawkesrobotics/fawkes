
/***************************************************************************
 *  console.cpp - Fawkes console logger
 *
 *  Created: Tue Jan 16 21:08:25 2007
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

#include <core/threading/mutex.h>
#include <utils/logging/console.h>

#include <utils/system/console_colors.h>

#include <sys/time.h>
#include <time.h>

using namespace std;

/** @class ConsoleLogger logging/console.h
 * Interface for logging to stderr on console.
 * The ConsoleLogger will pipe all output to stderr on the console. The
 * output will be color coded due to the importance of the output.
 *
 * Debug output will be drawn in grey font, informational output in console
 * default color, warnings will be printed in yellow and errors in red.
 *
 */

/** Constructor.
 * @param log_level minimum level to log
 */
ConsoleLogger::ConsoleLogger(LogLevel log_level)
  : Logger(log_level)
{
  now = (struct timeval *)malloc(sizeof(struct timeval));
  now_s = (struct tm *)malloc(sizeof(struct tm));
  mutex = new Mutex();
}


/** Destructor. */
ConsoleLogger::~ConsoleLogger()
{
  free(now);
  free(now_s);
  delete mutex;
}


/** Log debug message.
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 * @param va variadic argument list
 */
void
ConsoleLogger::vlog_debug(const char *component, const char *format, va_list va)
{
  if (log_level <= DEBUG ) {
    gettimeofday(now, NULL);
    localtime_r(&now->tv_sec, now_s);
    mutex->lock();
    fprintf(stderr, "%s%02d:%02d:%02d.%06ld %s: ", std::c_darkgray, now_s->tm_hour,
	    now_s->tm_min, now_s->tm_sec, now->tv_usec, component);
    vfprintf(stderr, format, va);
    fprintf(stderr, "%s\n", std::c_normal);
    mutex->unlock();
  }
}


/** Log informational message.
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 * @param va variadic argument list
 */
void
ConsoleLogger::vlog_info(const char *component, const char *format, va_list va)
{
  if (log_level <= INFO ) {
    gettimeofday(now, NULL);
    localtime_r(&now->tv_sec, now_s);
    mutex->lock();
    fprintf(stderr, "%02d:%02d:%02d.%06ld %s: ", now_s->tm_hour, now_s->tm_min,
	    now_s->tm_sec, now->tv_usec, component);
    vfprintf(stderr, format, va);
    fprintf(stderr, "\n");
    mutex->unlock();
  }
}


/** Log warning message.
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 * @param va variadic argument list
 */
void
ConsoleLogger::vlog_warn(const char *component, const char *format, va_list va)
{
  if ( log_level <= WARN ) {
    gettimeofday(now, NULL);
    localtime_r(&now->tv_sec, now_s);
    mutex->lock();
    fprintf(stderr, "%s%02d:%02d:%02d.%06ld %s: ", std::c_yellow, now_s->tm_hour,
	    now_s->tm_min, now_s->tm_sec, now->tv_usec, component);
    vfprintf(stderr, format, va);
    fprintf(stderr, "%s\n", std::c_normal);
    mutex->unlock();
  }
}


/** Log error message.
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 * @param va variadic argument list
 */
void
ConsoleLogger::vlog_error(const char *component, const char *format, va_list va)
{
  if ( log_level <= ERROR ) {
    gettimeofday(now, NULL);
    localtime_r(&now->tv_sec, now_s);
    mutex->lock();
    fprintf(stderr, "%s%02d:%02d:%02d.%06ld %s: ", std::c_red, now_s->tm_hour,
	    now_s->tm_min, now_s->tm_sec, now->tv_usec, component);
    vfprintf(stderr, format, va);
    fprintf(stderr, "%s\n", std::c_normal);
    mutex->unlock();
  }
}


/** Log debug message.
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 */
void
ConsoleLogger::log_debug(const char *component, const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  vlog_debug(component, format, arg);
  va_end(arg);
}


/** Log informational message.
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 */
void
ConsoleLogger::log_info(const char *component, const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  vlog_info(component, format, arg);
  va_end(arg);
}


/** Log warning message.
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 */
void
ConsoleLogger::log_warn(const char *component, const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  vlog_warn(component, format, arg);
  va_end(arg);
}


/** Log error message.
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 */
void
ConsoleLogger::log_error(const char *component, const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  vlog_error(component, format, arg);
  va_end(arg);
}


/** Log debug message.
 * @param component component, used to distuinguish logged messages
 * @param e exception to log, exception messages will be logged
 */
void
ConsoleLogger::log_debug(const char *component, Exception &e)
{
  if (log_level <= DEBUG ) {
    gettimeofday(now, NULL);
    localtime_r(&now->tv_sec, now_s);
    mutex->lock();
    for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
      fprintf(stderr, "%s%02d:%02d:%02d.%06ld %s [EXCEPTION]: ", std::c_darkgray, now_s->tm_hour,
	    now_s->tm_min, now_s->tm_sec, now->tv_usec, component);
      fprintf(stderr, *i);
      fprintf(stderr, "%s\n", std::c_normal);
    }
    mutex->unlock();
  }
}

/** Log informational message.
 * @param component component, used to distuinguish logged messages
 * @param e exception to log, exception messages will be logged
 */
void
ConsoleLogger::log_info(const char *component, Exception &e)
{
  if (log_level <= INFO ) {
    gettimeofday(now, NULL);
    localtime_r(&now->tv_sec, now_s);
    mutex->lock();
    for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
      fprintf(stderr, "%02d:%02d:%02d.%06ld %s [EXCEPTION]: ", now_s->tm_hour,
	      now_s->tm_min, now_s->tm_sec, now->tv_usec, component);
      fprintf(stderr, *i);
      fprintf(stderr, "%s\n", std::c_normal);
    }
    mutex->unlock();
  }
}


/** Log warning message.
 * @param component component, used to distuinguish logged messages
 * @param e exception to log, exception messages will be logged
 */
void
ConsoleLogger::log_warn(const char *component, Exception &e)
{
  if (log_level <= WARN ) {
    gettimeofday(now, NULL);
    localtime_r(&now->tv_sec, now_s);
    mutex->lock();
    for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
      fprintf(stderr, "%s%02d:%02d:%02d.%06ld %s [EXCEPTION]: ", std::c_yellow, now_s->tm_hour,
	      now_s->tm_min, now_s->tm_sec, now->tv_usec, component);
      fprintf(stderr, *i);
      fprintf(stderr, "%s\n", std::c_normal);
    }
    mutex->unlock();
  }
}


/** Log error message.
 * @param component component, used to distuinguish logged messages
 * @param e exception to log, exception messages will be logged
 */
void
ConsoleLogger::log_error(const char *component, Exception &e)
{
  if (log_level <= DEBUG ) {
    gettimeofday(now, NULL);
    localtime_r(&now->tv_sec, now_s);
    mutex->lock();
    for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
      fprintf(stderr, "%s%02d:%02d:%02d.%06ld %s [EXCEPTION]: ", std::c_red, now_s->tm_hour,
	      now_s->tm_min, now_s->tm_sec, now->tv_usec, component);
      fprintf(stderr, *i);
      fprintf(stderr, "%s\n", std::c_normal);
    }
    mutex->unlock();
  }
}
