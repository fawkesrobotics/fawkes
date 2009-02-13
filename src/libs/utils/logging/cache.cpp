
/***************************************************************************
 *  cache.cpp - Fawkes cache logger
 *
 *  Created: Wed Feb 11 23:02:08 2009
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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

#include <core/threading/mutex.h>
#include <utils/logging/cache.h>

#include <cstdlib>
#include <sys/time.h>
#include <ctime>

#define TIMESTR_LENGTH 200
#define MESSAGE_LENGTH 4096

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class CacheLogger <utils/logging/cache.h>
 * Logging Cache.
 * The CacheLogger will cache the log messages. By default these are
 * 20 messages.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param num_entries number of entries in the cache, if the cache is full and a
 * new log message arrives the oldest message is erased.
 * @param log_level minimum level to log
 */
CacheLogger::CacheLogger(unsigned int num_entries, LogLevel log_level)
  : Logger(log_level)
{
  __max_num_entries = num_entries;
  __num_entries = 0;

  now_s = (struct ::tm *)malloc(sizeof(struct ::tm));
  mutex = new Mutex();
}

/** Destructor. */
CacheLogger::~CacheLogger()
{
  free(now_s);
  delete mutex;
}

std::list<CacheLogger::CacheEntry> &
CacheLogger::get_messages()
{
  return __messages;
}

void
CacheLogger::clear()
{
  mutex->lock();
  __num_entries = 0;
  __messages.clear();
  mutex->unlock();
}

void
CacheLogger::push_message(LogLevel ll, const char *component, const char *format, va_list va)
{
  if (log_level <= ll ) {
    struct timeval now;
    gettimeofday(&now, NULL);
    mutex->lock();
    localtime_r(&now.tv_sec, now_s);
    char timestr[TIMESTR_LENGTH];
    snprintf(timestr, TIMESTR_LENGTH, "%02d:%02d:%02d.%06ld", now_s->tm_hour,
	     now_s->tm_min, now_s->tm_sec, now.tv_usec);
    char msg[MESSAGE_LENGTH];
    vsnprintf(msg, MESSAGE_LENGTH, format, va);

    CacheEntry e;
    e.log_level = ll;
    e.component = component;
    e.time      = now;
    e.timestr   = timestr;
    e.message   = msg;
    __messages.push_front(e);

    if (__num_entries == __max_num_entries) {
      __messages.pop_back();
    } else {
      ++__num_entries;
    }
    mutex->unlock();
  }
}

void
CacheLogger::push_message(LogLevel ll, const char *component, Exception &e)
{
  if (log_level <= ll ) {
    struct timeval now;
    gettimeofday(&now, NULL);
    mutex->lock();
    localtime_r(&now.tv_sec, now_s);
    char timestr[TIMESTR_LENGTH];
    snprintf(timestr, TIMESTR_LENGTH, "%02d:%02d:%02d.%06ld",
	     now_s->tm_hour, now_s->tm_min, now_s->tm_sec, now.tv_usec);

    for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
      CacheEntry e;
      e.log_level = ll;
      e.component = component;
      e.time      = now;
      e.timestr   = timestr;
      e.message   = std::string("[EXCEPTION] ") + *i;
      __messages.push_front(e);
    }

    if (__num_entries == __max_num_entries) {
      __messages.pop_back();
    } else {
      ++__num_entries;
    }
    mutex->unlock();
  }
}

void
CacheLogger::vlog_debug(const char *component, const char *format, va_list va)
{
  push_message(LL_DEBUG, component, format, va);
}

void
CacheLogger::vlog_info(const char *component, const char *format, va_list va)
{
  push_message(LL_INFO, component, format, va);
}

void
CacheLogger::vlog_warn(const char *component, const char *format, va_list va)
{
  push_message(LL_WARN, component, format, va);
}

void
CacheLogger::vlog_error(const char *component, const char *format, va_list va)
{
  push_message(LL_ERROR, component, format, va);
}

void
CacheLogger::log_debug(const char *component, const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  push_message(LL_DEBUG, component, format, arg);
  va_end(arg);
}

void
CacheLogger::log_info(const char *component, const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  push_message(LL_INFO, component, format, arg);
  va_end(arg);
}

void
CacheLogger::log_warn(const char *component, const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  push_message(LL_WARN, component, format, arg);
  va_end(arg);
}

void
CacheLogger::log_error(const char *component, const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  push_message(LL_ERROR, component, format, arg);
  va_end(arg);
}

void
CacheLogger::log_debug(const char *component, Exception &e)
{
  push_message(LL_DEBUG, component, e);
}

void
CacheLogger::log_info(const char *component, Exception &e)
{
  push_message(LL_INFO, component, e);
}

void
CacheLogger::log_warn(const char *component, Exception &e)
{
  push_message(LL_WARN, component, e);
}

void
CacheLogger::log_error(const char *component, Exception &e)
{
  push_message(LL_ERROR, component, e);
}

void
CacheLogger::tlog_push_message(LogLevel ll, struct timeval *t, const char *component,
			  const char *format, va_list va)
{
  if (log_level <= ll ) {
    mutex->lock();
    localtime_r(&t->tv_sec, now_s);
    char timestr[TIMESTR_LENGTH];
    snprintf(timestr, TIMESTR_LENGTH, "%02d:%02d:%02d.%06ld", now_s->tm_hour,
	     now_s->tm_min, now_s->tm_sec, t->tv_usec);
    char msg[MESSAGE_LENGTH];
    vsnprintf(msg, MESSAGE_LENGTH, format, va);

    CacheEntry e;
    e.log_level = ll;
    e.component = component;
    e.time      = *t;
    e.timestr   = timestr;
    e.message   = msg;
    __messages.push_front(e);

    if (__num_entries == __max_num_entries) {
      __messages.pop_back();
    } else {
      ++__num_entries;
    }
    mutex->unlock();
  }
}

void
CacheLogger::tlog_push_message(LogLevel ll, struct timeval *t, const char *component, Exception &e)
{
  if (log_level <= ll ) {
    mutex->lock();
    localtime_r(&t->tv_sec, now_s);
    char timestr[TIMESTR_LENGTH];
    snprintf(timestr, TIMESTR_LENGTH,
	     "%02d:%02d:%02d.%06ld", now_s->tm_hour,
	     now_s->tm_min, now_s->tm_sec, t->tv_usec);
    for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
      CacheEntry e;
      e.log_level = ll;
      e.component = component;
      e.time      = *t;
      e.timestr   = timestr;
      e.message   = std::string("[EXCEPTION] ") + *i;
      __messages.push_front(e);
    }

    if (__num_entries == __max_num_entries) {
      __messages.pop_back();
    } else {
      ++__num_entries;
    }
    mutex->unlock();
  }
}

void
CacheLogger::tlog_debug(struct timeval *t, const char *component, const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  tlog_push_message(LL_DEBUG, t, component, format, arg);
  va_end(arg);
}

void
CacheLogger::tlog_info(struct timeval *t, const char *component, const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  tlog_push_message(LL_INFO, t, component, format, arg);
  va_end(arg);
}

void
CacheLogger::tlog_warn(struct timeval *t, const char *component, const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  tlog_push_message(LL_WARN, t, component, format, arg);
  va_end(arg);
}

void
CacheLogger::tlog_error(struct timeval *t, const char *component, const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  tlog_push_message(LL_ERROR, t, component, format, arg);
  va_end(arg);
}

void
CacheLogger::tlog_debug(struct timeval *t, const char *component, Exception &e)
{
  tlog_push_message(LL_DEBUG, t, component, e);
}

void
CacheLogger::tlog_info(struct timeval *t, const char *component, Exception &e)
{
  tlog_push_message(LL_INFO, t, component, e);
}

void
CacheLogger::tlog_warn(struct timeval *t, const char *component, Exception &e)
{
  tlog_push_message(LL_WARN, t, component, e);
}

void
CacheLogger::tlog_error(struct timeval *t, const char *component, Exception &e)
{
  tlog_push_message(LL_ERROR, t, component, e);
}

void
CacheLogger::vtlog_debug(struct timeval *t, const char *component, const char *format, va_list va)
{
  tlog_push_message(LL_DEBUG, t, component, format, va);
}

void
CacheLogger::vtlog_info(struct timeval *t, const char *component, const char *format, va_list va)
{
  tlog_push_message(LL_INFO, t, component, format, va);
}

void
CacheLogger::vtlog_warn(struct timeval *t, const char *component, const char *format, va_list va)
{
  tlog_push_message(LL_WARN, t, component, format, va);
}

void
CacheLogger::vtlog_error(struct timeval *t, const char *component, const char *format, va_list va)
{
  tlog_push_message(LL_ERROR, t, component, format, va);
}

} // end namespace fawkes
