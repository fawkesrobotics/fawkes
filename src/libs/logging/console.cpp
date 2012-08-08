
/***************************************************************************
 *  console.cpp - Fawkes console logger
 *
 *  Created: Tue Jan 16 21:08:25 2007
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#include <core/threading/mutex.h>
#include <logging/console.h>

#include <utils/system/console_colors.h>

#include <cstdlib>
#include <sys/time.h>
#include <ctime>
#include <cstdio>

namespace fawkes {

/** @class ConsoleLogger <logging/console.h>
 * Interface for logging to stderr.
 * The ConsoleLogger will pipe all output to stderr on the console. The
 * output will be color coded due to the importance of the output.
 *
 * Debug output will be drawn in grey font, informational output in console
 * default color, warnings will be printed in brown/orange and errors in red.
 *
 * @author Tim Niemueller
 */

/** Constructor.
 * @param log_level minimum level to log
 */
ConsoleLogger::ConsoleLogger(LogLevel log_level)
  : Logger(log_level)
{
  now_s = (struct ::tm *)malloc(sizeof(struct ::tm));
  mutex = new Mutex();
}


/** Destructor. */
ConsoleLogger::~ConsoleLogger()
{
  free(now_s);
  delete mutex;
}


void
ConsoleLogger::vlog_debug(const char *component, const char *format, va_list va)
{
  if (log_level <= LL_DEBUG ) {
    struct timeval now;
    gettimeofday(&now, NULL);
    mutex->lock();
    localtime_r(&now.tv_sec, now_s);
    fprintf(stderr, "%s%02d:%02d:%02d.%06ld %s: ", c_lightgray, now_s->tm_hour,
	    now_s->tm_min, now_s->tm_sec, (long)now.tv_usec, component);
    vfprintf(stderr, format, va);
    fprintf(stderr, "%s\n", c_normal);
    mutex->unlock();
  }
}


void
ConsoleLogger::vlog_info(const char *component, const char *format, va_list va)
{
  if (log_level <= LL_INFO ) {
    struct timeval now;
    gettimeofday(&now, NULL);
    mutex->lock();
    localtime_r(&now.tv_sec, now_s);
    fprintf(stderr, "%02d:%02d:%02d.%06ld %s: ", now_s->tm_hour, now_s->tm_min,
	    now_s->tm_sec, (long)now.tv_usec, component);
    vfprintf(stderr, format, va);
    fprintf(stderr, "\n");
    mutex->unlock();
  }
}


void
ConsoleLogger::vlog_warn(const char *component, const char *format, va_list va)
{
  if ( log_level <= LL_WARN ) {
    struct timeval now;
    gettimeofday(&now, NULL);
    mutex->lock();
    localtime_r(&now.tv_sec, now_s);
    fprintf(stderr, "%s%02d:%02d:%02d.%06ld %s: ", c_brown, now_s->tm_hour,
	    now_s->tm_min, now_s->tm_sec, (long)now.tv_usec, component);
    vfprintf(stderr, format, va);
    fprintf(stderr, "%s\n", c_normal);
    mutex->unlock();
  }
}


void
ConsoleLogger::vlog_error(const char *component, const char *format, va_list va)
{
  if ( log_level <= LL_ERROR ) {
    struct timeval now;
    gettimeofday(&now, NULL);
    mutex->lock();
    localtime_r(&now.tv_sec, now_s);
    fprintf(stderr, "%s%02d:%02d:%02d.%06ld %s: ", c_red, now_s->tm_hour,
	    now_s->tm_min, now_s->tm_sec, (long)now.tv_usec, component);
    vfprintf(stderr, format, va);
    fprintf(stderr, "%s\n", c_normal);
    mutex->unlock();
  }
}


void
ConsoleLogger::log_debug(const char *component, const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  vlog_debug(component, format, arg);
  va_end(arg);
}


void
ConsoleLogger::log_info(const char *component, const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  vlog_info(component, format, arg);
  va_end(arg);
}


void
ConsoleLogger::log_warn(const char *component, const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  vlog_warn(component, format, arg);
  va_end(arg);
}


void
ConsoleLogger::log_error(const char *component, const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  vlog_error(component, format, arg);
  va_end(arg);
}


void
ConsoleLogger::log_debug(const char *component, Exception &e)
{
  if (log_level <= LL_DEBUG ) {
    struct timeval now;
    gettimeofday(&now, NULL);
    mutex->lock();
    localtime_r(&now.tv_sec, now_s);
    for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
      fprintf(stderr, "%s%02d:%02d:%02d.%06ld %s: [EXCEPTION] ", c_lightgray, now_s->tm_hour,
	    now_s->tm_min, now_s->tm_sec, (long)now.tv_usec, component);
      fprintf(stderr, "%s", *i);
      fprintf(stderr, "%s\n", c_normal);
    }
    mutex->unlock();
  }
}


void
ConsoleLogger::log_info(const char *component, Exception &e)
{
  if (log_level <= LL_INFO ) {
    struct timeval now;
    gettimeofday(&now, NULL);
    mutex->lock();
    localtime_r(&now.tv_sec, now_s);
    for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
      fprintf(stderr, "%02d:%02d:%02d.%06ld %s: [EXCEPTION] ", now_s->tm_hour,
	      now_s->tm_min, now_s->tm_sec, (long)now.tv_usec, component);
      fprintf(stderr, "%s", *i);
      fprintf(stderr, "%s\n", c_normal);
    }
    mutex->unlock();
  }
}


void
ConsoleLogger::log_warn(const char *component, Exception &e)
{
  if (log_level <= LL_WARN ) {
    struct timeval now;
    gettimeofday(&now, NULL);
    mutex->lock();
    localtime_r(&now.tv_sec, now_s);
    for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
      fprintf(stderr, "%s%02d:%02d:%02d.%06ld %s: [EXCEPTION] ", c_brown, now_s->tm_hour,
	      now_s->tm_min, now_s->tm_sec, (long)now.tv_usec, component);
      fprintf(stderr, "%s", *i);
      fprintf(stderr, "%s\n", c_normal);
    }
    mutex->unlock();
  }
}


void
ConsoleLogger::log_error(const char *component, Exception &e)
{
  if (log_level <= LL_ERROR ) {
    struct timeval now;
    gettimeofday(&now, NULL);
    mutex->lock();
    localtime_r(&now.tv_sec, now_s);
    for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
      fprintf(stderr, "%s%02d:%02d:%02d.%06ld %s: [EXCEPTION] ", c_red, now_s->tm_hour,
	      now_s->tm_min, now_s->tm_sec, (long)now.tv_usec, component);
      fprintf(stderr, "%s", *i);
      fprintf(stderr, "%s\n", c_normal);
    }
    mutex->unlock();
  }
}


void
ConsoleLogger::tlog_debug(struct timeval *t, const char *component, const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  vtlog_debug(t, component, format, arg);
  va_end(arg);
}


void
ConsoleLogger::tlog_info(struct timeval *t, const char *component, const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  vtlog_info(t, component, format, arg);
  va_end(arg);
}


void
ConsoleLogger::tlog_warn(struct timeval *t, const char *component, const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  vtlog_warn(t, component, format, arg);
  va_end(arg);
}


void
ConsoleLogger::tlog_error(struct timeval *t, const char *component, const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  vtlog_error(t, component, format, arg);
  va_end(arg);
}


void
ConsoleLogger::tlog_debug(struct timeval *t, const char *component, Exception &e)
{
  if (log_level <= LL_DEBUG ) {
    mutex->lock();
    localtime_r(&t->tv_sec, now_s);
    for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
      fprintf(stderr, "%s%02d:%02d:%02d.%06ld %s: [EXCEPTION] ", c_lightgray, now_s->tm_hour,
	    now_s->tm_min, now_s->tm_sec, (long)t->tv_usec, component);
      fprintf(stderr, "%s", *i);
      fprintf(stderr, "%s\n", c_normal);
    }
    mutex->unlock();
  }
}


void
ConsoleLogger::tlog_info(struct timeval *t, const char *component, Exception &e)
{
  if (log_level <= LL_INFO ) {
    mutex->lock();
    localtime_r(&t->tv_sec, now_s);
    for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
      fprintf(stderr, "%02d:%02d:%02d.%06ld %s: [EXCEPTION] ", now_s->tm_hour,
	      now_s->tm_min, now_s->tm_sec, (long)t->tv_usec, component);
      fprintf(stderr, "%s", *i);
      fprintf(stderr, "%s\n", c_normal);
    }
    mutex->unlock();
  }
}


void
ConsoleLogger::tlog_warn(struct timeval *t, const char *component, Exception &e)
{
  if (log_level <= LL_WARN ) {
    mutex->lock();
    localtime_r(&t->tv_sec, now_s);
    for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
      fprintf(stderr, "%s%02d:%02d:%02d.%06ld %s: [EXCEPTION] ", c_brown, now_s->tm_hour,
	      now_s->tm_min, now_s->tm_sec, (long)t->tv_usec, component);
      fprintf(stderr, "%s", *i);
      fprintf(stderr, "%s\n", c_normal);
    }
    mutex->unlock();
  }
}


void
ConsoleLogger::tlog_error(struct timeval *t, const char *component, Exception &e)
{
  if (log_level <= LL_ERROR ) {
    mutex->lock();
    localtime_r(&t->tv_sec, now_s);
    for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
      fprintf(stderr, "%s%02d:%02d:%02d.%06ld %s: [EXCEPTION] ", c_red, now_s->tm_hour,
	      now_s->tm_min, now_s->tm_sec, (long)t->tv_usec, component);
      fprintf(stderr, "%s", *i);
      fprintf(stderr, "%s\n", c_normal);
    }
    mutex->unlock();
  }
}




void
ConsoleLogger::vtlog_debug(struct timeval *t, const char *component, const char *format, va_list va)
{
  if (log_level <= LL_DEBUG ) {
    mutex->lock();
    localtime_r(&t->tv_sec, now_s);
    fprintf(stderr, "%s%02d:%02d:%02d.%06ld %s: ", c_lightgray, now_s->tm_hour,
	    now_s->tm_min, now_s->tm_sec, (long)t->tv_usec, component);
    vfprintf(stderr, format, va);
    fprintf(stderr, "%s\n", c_normal);
    mutex->unlock();
  }
}


void
ConsoleLogger::vtlog_info(struct timeval *t, const char *component, const char *format, va_list va)
{
  if (log_level <= LL_INFO ) {
    mutex->lock();
    localtime_r(&t->tv_sec, now_s);
    fprintf(stderr, "%02d:%02d:%02d.%06ld %s: ", now_s->tm_hour, now_s->tm_min,
	    now_s->tm_sec, (long)t->tv_usec, component);
    vfprintf(stderr, format, va);
    fprintf(stderr, "\n");
    mutex->unlock();
  }
}


void
ConsoleLogger::vtlog_warn(struct timeval *t, const char *component, const char *format, va_list va)
{
  if ( log_level <= LL_WARN ) {
    mutex->lock();
    localtime_r(&t->tv_sec, now_s);
    fprintf(stderr, "%s%02d:%02d:%02d.%06ld %s: ", c_brown, now_s->tm_hour,
	    now_s->tm_min, now_s->tm_sec, (long)t->tv_usec, component);
    vfprintf(stderr, format, va);
    fprintf(stderr, "%s\n", c_normal);
    mutex->unlock();
  }
}


void
ConsoleLogger::vtlog_error(struct timeval *t, const char *component, const char *format, va_list va)
{
  if ( log_level <= LL_ERROR ) {
    mutex->lock();
    localtime_r(&t->tv_sec, now_s);
    fprintf(stderr, "%s%02d:%02d:%02d.%06ld %s: ", c_red, now_s->tm_hour,
	    now_s->tm_min, now_s->tm_sec, (long)t->tv_usec, component);
    vfprintf(stderr, format, va);
    fprintf(stderr, "%s\n", c_normal);
    mutex->unlock();
  }
}


} // end namespace fawkes
