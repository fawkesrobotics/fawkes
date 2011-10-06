
/***************************************************************************
 *  syslog.cpp - Fawkes syslog logger
 *
 *  Created: Thu Aug 18 17:15:30 2011
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
#include <logging/syslog.h>

#include <syslog.h>
#include <cstdlib>
#include <sys/time.h>
#include <ctime>
#include <cstdio>
#include <cstring>

namespace fawkes {

/** @class SyslogLogger <logging/syslog.h>
 * Interface for logging to syslog.
 * The SyslogLogger will pipe all output to the syslog.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param log_level minimum level to log
 */
SyslogLogger::SyslogLogger(LogLevel log_level)
  : Logger(log_level)
{
  now_s = (struct ::tm *)malloc(sizeof(struct ::tm));
  mutex = new Mutex();
  __ident = NULL;
  openlog("Fawkes", LOG_CONS | LOG_NDELAY, LOG_USER);
}


/** Constructor with ident.
 * @param ident ident string passed to openlog.
 * @param log_level minimum level to log
 */
SyslogLogger::SyslogLogger(const char *ident, LogLevel log_level)
  : Logger(log_level)
{
  now_s = (struct ::tm *)malloc(sizeof(struct ::tm));
  mutex = new Mutex();
  if (ident == NULL) {
    __ident = NULL;
    openlog("Fawkes", LOG_CONS | LOG_NDELAY, LOG_USER);
  } else {
    __ident = strdup(ident);
    openlog(__ident, LOG_CONS | LOG_NDELAY, LOG_USER);
  }
}


/** Destructor. */
SyslogLogger::~SyslogLogger()
{
  free(now_s);
  delete mutex;
  closelog();
  if (__ident)  free(__ident);
  __ident = NULL;
}


void
SyslogLogger::vlog_debug(const char *component, const char *format, va_list va)
{
  if (log_level <= LL_DEBUG ) {
    mutex->lock();
    char *message;
    if (vasprintf(&message, format, va) != -1) {
      syslog(LOG_DEBUG, "%s: %s", component, message);
      free(message);
    } else {
      vsyslog(LOG_DEBUG, format, va);
    }
    mutex->unlock();
  }
}


void
SyslogLogger::vlog_info(const char *component, const char *format, va_list va)
{
  if (log_level <= LL_INFO ) {
    mutex->lock();
    char *message;
    if (vasprintf(&message, format, va) != -1) {
      syslog(LOG_INFO, "%s: %s", component, message);
      free(message);
    } else {
      vsyslog(LOG_INFO, format, va);
    }
    mutex->unlock();
  }
}


void
SyslogLogger::vlog_warn(const char *component, const char *format, va_list va)
{
  if ( log_level <= LL_WARN ) {
    mutex->lock();
    char *message;
    if (vasprintf(&message, format, va) != -1) {
      syslog(LOG_WARNING, "%s: %s", component, message);
      free(message);
    } else {
      vsyslog(LOG_WARNING, format, va);
    }
    mutex->unlock();
  }
}


void
SyslogLogger::vlog_error(const char *component, const char *format, va_list va)
{
  if ( log_level <= LL_ERROR ) {
    mutex->lock();
    char *message;
    if (vasprintf(&message, format, va) != -1) {
      syslog(LOG_ERR, "%s: %s", component, message);
      free(message);
    } else {
      vsyslog(LOG_ERR, format, va);
    }
    mutex->unlock();
  }
}


void
SyslogLogger::log_debug(const char *component, const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  vlog_debug(component, format, arg);
  va_end(arg);
}


void
SyslogLogger::log_info(const char *component, const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  vlog_info(component, format, arg);
  va_end(arg);
}


void
SyslogLogger::log_warn(const char *component, const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  vlog_warn(component, format, arg);
  va_end(arg);
}


void
SyslogLogger::log_error(const char *component, const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  vlog_error(component, format, arg);
  va_end(arg);
}


void
SyslogLogger::log_debug(const char *component, Exception &e)
{
  if (log_level <= LL_DEBUG ) {
    mutex->lock();
    for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
      syslog(LOG_DEBUG, "%s: [EXC] %s", component, *i);
    }
    mutex->unlock();
  }
}


void
SyslogLogger::log_info(const char *component, Exception &e)
{
  if (log_level <= LL_INFO ) {
    mutex->lock();
    for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
      syslog(LOG_INFO, "%s: [EXC] %s", component, *i);
    }
    mutex->unlock();
  }
}


void
SyslogLogger::log_warn(const char *component, Exception &e)
{
  if (log_level <= LL_WARN ) {
    mutex->lock();
    for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
      syslog(LOG_WARNING, "%s: [EXC] %s", component, *i);
    }
    mutex->unlock();
  }
}


void
SyslogLogger::log_error(const char *component, Exception &e)
{
  if (log_level <= LL_DEBUG ) {
    mutex->lock();
    for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
      syslog(LOG_ERR, "%s: [EXC] %s", component, *i);
    }
    mutex->unlock();
  }
}


void
SyslogLogger::tlog_debug(struct timeval *t, const char *component, const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  vtlog_debug(t, component, format, arg);
  va_end(arg);
}


void
SyslogLogger::tlog_info(struct timeval *t, const char *component, const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  vtlog_info(t, component, format, arg);
  va_end(arg);
}


void
SyslogLogger::tlog_warn(struct timeval *t, const char *component, const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  vtlog_warn(t, component, format, arg);
  va_end(arg);
}


void
SyslogLogger::tlog_error(struct timeval *t, const char *component, const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  vtlog_error(t, component, format, arg);
  va_end(arg);
}


void
SyslogLogger::tlog_debug(struct timeval *t, const char *component, Exception &e)
{
  if (log_level <= LL_DEBUG ) {
    mutex->lock();
    localtime_r(&t->tv_sec, now_s);
    for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
      syslog(LOG_DEBUG, "%s @ %02d:%02d:%02d.%06ld: [EXC] %s", component,
             now_s->tm_hour, now_s->tm_min, now_s->tm_sec, (long)t->tv_usec, *i);
    }
    mutex->unlock();
  }
}


void
SyslogLogger::tlog_info(struct timeval *t, const char *component, Exception &e)
{
  if (log_level <= LL_INFO ) {
    mutex->lock();
    localtime_r(&t->tv_sec, now_s);
    for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
      syslog(LOG_INFO, "%s @ %02d:%02d:%02d.%06ld: [EXC] %s", component,
             now_s->tm_hour, now_s->tm_min, now_s->tm_sec, (long)t->tv_usec, *i);
    }
    mutex->unlock();
  }
}


void
SyslogLogger::tlog_warn(struct timeval *t, const char *component, Exception &e)
{
  if (log_level <= LL_WARN ) {
    mutex->lock();
    localtime_r(&t->tv_sec, now_s);
    for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
      syslog(LOG_WARNING, "%s @ %02d:%02d:%02d.%06ld: [EXC] %s", component,
             now_s->tm_hour, now_s->tm_min, now_s->tm_sec, (long)t->tv_usec, *i);
    }
    mutex->unlock();
  }
}


void
SyslogLogger::tlog_error(struct timeval *t, const char *component, Exception &e)
{
  if (log_level <= LL_DEBUG ) {
    mutex->lock();
    localtime_r(&t->tv_sec, now_s);
    for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
      syslog(LOG_ERR, "%s @ %02d:%02d:%02d.%06ld: [EXC] %s", component,
             now_s->tm_hour, now_s->tm_min, now_s->tm_sec, (long)t->tv_usec, *i);
    }
    mutex->unlock();
  }
}




void
SyslogLogger::vtlog_debug(struct timeval *t, const char *component,
                          const char *format, va_list va)
{
  if (log_level <= LL_DEBUG ) {
    mutex->lock();
    localtime_r(&t->tv_sec, now_s);
    char *message;
    if (vasprintf(&message, format, va) != -1) {
      syslog(LOG_DEBUG, "%s @ %02d:%02d:%02d.%06ld: %s", component,
             now_s->tm_hour, now_s->tm_min, now_s->tm_sec,
             (long)t->tv_usec, message);
      free(message);
    } else {
      vsyslog(LOG_DEBUG, format, va);
    }
    mutex->unlock();
  }
}


void
SyslogLogger::vtlog_info(struct timeval *t, const char *component, const char *format, va_list va)
{
  if (log_level <= LL_INFO ) {
    mutex->lock();
    localtime_r(&t->tv_sec, now_s);
    char *message;
    if (vasprintf(&message, format, va) != -1) {
      syslog(LOG_INFO, "%s @ %02d:%02d:%02d.%06ld: %s", component,
             now_s->tm_hour, now_s->tm_min, now_s->tm_sec,
             (long)t->tv_usec, message);
      free(message);
    } else {
      vsyslog(LOG_INFO, format, va);
    }
    mutex->unlock();
  }
}


void
SyslogLogger::vtlog_warn(struct timeval *t, const char *component, const char *format, va_list va)
{
  if ( log_level <= LL_WARN ) {
    mutex->lock();
    localtime_r(&t->tv_sec, now_s);
    char *message;
    if (vasprintf(&message, format, va) != -1) {
      syslog(LOG_WARNING, "%s @ %02d:%02d:%02d.%06ld: %s", component,
             now_s->tm_hour, now_s->tm_min, now_s->tm_sec,
             (long)t->tv_usec, message);
      free(message);
    } else {
      vsyslog(LOG_WARNING, format, va);
    }
    mutex->unlock();
  }
}


void
SyslogLogger::vtlog_error(struct timeval *t, const char *component, const char *format, va_list va)
{
  if ( log_level <= LL_ERROR ) {
    mutex->lock();
    localtime_r(&t->tv_sec, now_s);
    char *message;
    if (vasprintf(&message, format, va) != -1) {
      syslog(LOG_ERR, "%s @ %02d:%02d:%02d.%06ld: %s", component,
             now_s->tm_hour, now_s->tm_min, now_s->tm_sec,
             (long)t->tv_usec, message);
      free(message);
    } else {
      vsyslog(LOG_ERR, format, va);
    }
    mutex->unlock();
  }
}


} // end namespace fawkes
