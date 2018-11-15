
/***************************************************************************
 *  cache.cpp - Fawkes cache logger
 *
 *  Created: Wed Feb 11 23:02:08 2009
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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
#include <core/threading/mutex_locker.h>
#include <logging/cache.h>

#include <cstdlib>
#include <sys/time.h>
#include <ctime>
#include <cstdio>
#include <algorithm>

namespace fawkes {

/** @class CacheLogger <logging/cache.h>
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
  max_num_entries_ = num_entries;
  num_entries_ = 0;

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
  return messages_;
}

void
CacheLogger::clear()
{
  mutex->lock();
  num_entries_ = 0;
  messages_.clear();
  mutex->unlock();
}


/** Get maximum number of log entries in cache.
 * @return maximum number of cache entries
 */
unsigned int
CacheLogger::size() const
{
  return max_num_entries_;
}


/** Set maximum number of log entries in cache.
 * @param new_size new size
 */
void
CacheLogger::set_size(unsigned int new_size)
{
  MutexLocker lock(mutex);
  if (new_size < num_entries_) {
    num_entries_ = new_size;
    messages_.resize(num_entries_);
  }
  max_num_entries_ = new_size;
}


/** Lock cache logger, no new messages can be added.
 * Use with care, can cause critical delays in the whole software stack!
 */
void
CacheLogger::lock()
{
  mutex->lock();
}

/** Unlock cache logger. */
void
CacheLogger::unlock()
{
  mutex->unlock();
}

void
CacheLogger::push_message(LogLevel ll, const char *component, const char *format, va_list va)
{
  if (log_level <= ll ) {
    MutexLocker lock(mutex);
    struct timeval now;
    gettimeofday(&now, NULL);
    localtime_r(&now.tv_sec, now_s);
    char *timestr;
    if (asprintf(&timestr, "%02d:%02d:%02d.%06ld", now_s->tm_hour,
		 now_s->tm_min, now_s->tm_sec, now.tv_usec) == -1) {
      // Cannot do anything useful, drop log message
      return;
    }
    char *msg;
    if (vasprintf(&msg, format, va) == -1) {
      // Cannot do anything useful, drop log message
      free(timestr);
      return;
    }

    CacheEntry e;
    e.log_level = ll;
    e.component = component;
    e.time      = now;
    e.timestr   = timestr;
    e.message   = msg;
    messages_.push_front(e);

    free(timestr);
    free(msg);

    if (num_entries_ == max_num_entries_) {
      messages_.pop_back();
    } else {
      ++num_entries_;
    }
  }
}

void
CacheLogger::push_message(LogLevel ll, const char *component, Exception &e)
{
  if (log_level <= ll ) {
    MutexLocker lock(mutex);
    struct timeval now;
    gettimeofday(&now, NULL);
    localtime_r(&now.tv_sec, now_s);
    char *timestr;
    if (asprintf(&timestr, "%02d:%02d:%02d.%06ld", now_s->tm_hour,
		 now_s->tm_min, now_s->tm_sec, now.tv_usec) == -1) {
      return;
    }

    for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
      CacheEntry ce;
      ce.log_level = ll;
      ce.component = component;
      ce.time      = now;
      ce.timestr   = timestr;
      ce.message   = std::string("[EXCEPTION] ") + *i;
      messages_.push_front(ce);
      ++num_entries_;
    }

    free(timestr);

    if (num_entries_ > max_num_entries_) {
      num_entries_ = max_num_entries_;
      messages_.resize(max_num_entries_);
    }
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
    MutexLocker lock(mutex);
    localtime_r(&t->tv_sec, now_s);
    char *timestr;
    if (asprintf(&timestr, "%02d:%02d:%02d.%06ld", now_s->tm_hour,
		 now_s->tm_min, now_s->tm_sec, t->tv_usec) == -1) {
      return;
    }
    char *msg;
    if (vasprintf(&msg, format, va) == -1) {
      free(timestr);
      return;
    }

    CacheEntry e;
    e.log_level = ll;
    e.component = component;
    e.time      = *t;
    e.timestr   = timestr;
    e.message   = msg;
    messages_.push_front(e);

    free(timestr);
    free(msg);

    if (num_entries_ == max_num_entries_) {
      messages_.pop_back();
    } else {
      ++num_entries_;
    }
    mutex->unlock();
  }
}

void
CacheLogger::tlog_push_message(LogLevel ll, struct timeval *t, const char *component, Exception &e)
{
  if (log_level <= ll ) {
    MutexLocker lock(mutex);
    localtime_r(&t->tv_sec, now_s);
    char *timestr;
    if (asprintf(&timestr, "%02d:%02d:%02d.%06ld", now_s->tm_hour,
		 now_s->tm_min, now_s->tm_sec, t->tv_usec) == -1) {
      return;
    }
    for (Exception::iterator i = e.begin(); i != e.end(); ++i) {
      CacheEntry ce;
      ce.log_level = ll;
      ce.component = component;
      ce.time      = *t;
      ce.timestr   = timestr;
      ce.message   = std::string("[EXCEPTION] ") + *i;
      messages_.push_front(ce);
      ++num_entries_;
    }

    free(timestr);

    if (num_entries_ > max_num_entries_) {
      num_entries_ = max_num_entries_;
      messages_.resize(max_num_entries_);
    }
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
