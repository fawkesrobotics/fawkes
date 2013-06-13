
/***************************************************************************
 *  multi.h - Fawkes multi logger
 *
 *  Created: Mon May 07 16:44:15 2007
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

#include <logging/multi.h>
#include <logging/logger.h>

#include <core/utils/lock_list.h>
#include <core/threading/thread.h>

#include <sys/time.h>
#include <time.h>

namespace fawkes {

/// @cond INTERNALS
class MultiLoggerData
{
 public:
  MultiLoggerData()
  {
    mutex = new Mutex();
  }

  ~MultiLoggerData()
  {
    delete mutex;
    mutex = NULL;
  }

  LockList<Logger *>            loggers;
  LockList<Logger *>::iterator  logit;
  Mutex                        *mutex;
  Thread::CancelState           old_state;
};
/// @endcond


/** @class MultiLogger <logging/multi.h>
 * Log through multiple loggers.
 * It can be hand to have the opportunity to log to multiple channels, for
 * example log to a file and via network to a remote console. This can be
 * done with the MultiLogger. You can add an arbitrary number of loggers
 * to log the output to. Use the minimum number of necessary loggers though
 * because this can cause a high burden on log users if you have too many
 * loggers.
 *
 * Note that the multi logger takes over the ownership of the logger. That
 * means that the multi logger destroys all sub-loggers when it is deleted
 * itself. If you want to take over the loggers without destroying them you
 * have to properly remove them before destroying the multi logger.
 *
 * @author Tim Niemueller
 */

/** Constructor.
 * This will create the logger without any sub-loggers. Message that are
 * logged are simply ignored.
 */
MultiLogger::MultiLogger()
{
  data = new MultiLoggerData();
}


/** Constructor.
 * This sets one sub-logger that messages are sent to.
 * @param logger sub-logger
 */
MultiLogger::MultiLogger(Logger *logger)
{
  data = new MultiLoggerData();
  data->loggers.push_back_locked(logger);
}


/** Destructor.
 * This will destroy all sub-data->loggers (they are deleted).
 */
MultiLogger::~MultiLogger()
{
  data->loggers.lock();
  for (data->logit = data->loggers.begin(); data->logit != data->loggers.end(); ++data->logit) {
    delete (*data->logit);
  }
  data->loggers.clear();
  data->loggers.unlock();
  delete data;
}


/** Add a logger.
 * @param logger new sub-logger to add
 */
void
MultiLogger::add_logger(Logger *logger)
{
  data->mutex->lock();
  Thread::set_cancel_state(Thread::CANCEL_DISABLED, &(data->old_state));
  data->loggers.lock();
  data->loggers.push_back(logger);
  logger->set_loglevel(log_level);
  data->loggers.sort();
  data->loggers.unique();
  data->loggers.unlock();
  Thread::set_cancel_state(data->old_state);
  data->mutex->unlock();
}


/** Remove logger.
 * @param logger Sub-logger to remove
 */
void
MultiLogger::remove_logger(Logger *logger)
{
  data->mutex->lock();
  Thread::set_cancel_state(Thread::CANCEL_DISABLED, &(data->old_state));

  data->loggers.remove_locked(logger);
  Thread::set_cancel_state(data->old_state);
  data->mutex->unlock();
}


void
MultiLogger::set_loglevel(LogLevel level)
{
  data->mutex->lock();
  Thread::set_cancel_state(Thread::CANCEL_DISABLED, &(data->old_state));
  log_level = level;

  for (data->logit = data->loggers.begin(); data->logit != data->loggers.end(); ++data->logit) {
    (*data->logit)->set_loglevel(level);
  }
  Thread::set_cancel_state(data->old_state);
  data->mutex->unlock();
}


void
MultiLogger::log(LogLevel level, const char *component, const char *format, ...)
{
  struct timeval now;
  gettimeofday(&now, NULL);
  data->mutex->lock();
  Thread::set_cancel_state(Thread::CANCEL_DISABLED, &(data->old_state));

  va_list va;
  va_start(va, format);
  for (data->logit = data->loggers.begin(); data->logit != data->loggers.end(); ++data->logit) {
    va_list vac;
    va_copy(vac, va);
    (*data->logit)->vtlog(level, &now, component, format, vac);
    va_end(vac);
  }
  va_end(va);
  Thread::set_cancel_state(data->old_state);
  data->mutex->unlock();
}


void
MultiLogger::log_debug(const char *component, const char *format, ...)
{
  struct timeval now;
  gettimeofday(&now, NULL);
  data->mutex->lock();
  Thread::set_cancel_state(Thread::CANCEL_DISABLED, &(data->old_state));

  va_list va;
  va_start(va, format);
  for (data->logit = data->loggers.begin(); data->logit != data->loggers.end(); ++data->logit) {
    va_list vac;
    va_copy(vac, va);
    (*data->logit)->vlog_debug(component, format, vac);
    va_end(vac);
  }
  va_end(va);
  Thread::set_cancel_state(data->old_state);
  data->mutex->unlock();
}


void
MultiLogger::log_info(const char *component, const char *format, ...)
{
  struct timeval now;
  gettimeofday(&now, NULL);
  data->mutex->lock();
  Thread::set_cancel_state(Thread::CANCEL_DISABLED, &(data->old_state));

  va_list va;
  va_start(va, format);
  for (data->logit = data->loggers.begin(); data->logit != data->loggers.end(); ++data->logit) {
    va_list vac;
    va_copy(vac, va);
    (*data->logit)->vlog_info(component, format, vac);
    va_end(vac);
  }
  va_end(va);
  Thread::set_cancel_state(data->old_state);
  data->mutex->unlock();
}


void
MultiLogger::log_warn(const char *component, const char *format, ...)
{
  struct timeval now;
  gettimeofday(&now, NULL);
  data->mutex->lock();
  Thread::set_cancel_state(Thread::CANCEL_DISABLED, &(data->old_state));

  va_list va;
  va_start(va, format);
  for (data->logit = data->loggers.begin(); data->logit != data->loggers.end(); ++data->logit) {
    va_list vac;
    va_copy(vac, va);
    (*data->logit)->vlog_warn(component, format, vac);
    va_end(vac);
  }
  va_end(va);
  Thread::set_cancel_state(data->old_state);
  data->mutex->unlock();
}


void
MultiLogger::log_error(const char *component, const char *format, ...)
{
  struct timeval now;
  gettimeofday(&now, NULL);
  data->mutex->lock();
  Thread::set_cancel_state(Thread::CANCEL_DISABLED, &(data->old_state));

  va_list va;
  va_start(va, format);
  for (data->logit = data->loggers.begin(); data->logit != data->loggers.end(); ++data->logit) {
    va_list vac;
    va_copy(vac, va);
    (*data->logit)->vlog_error(component, format, vac);
    va_end(vac);
  }
  va_end(va);
  Thread::set_cancel_state(data->old_state);
  data->mutex->unlock();
}


void
MultiLogger::log(LogLevel level, const char *component, Exception &e)
{
  struct timeval now;
  gettimeofday(&now, NULL);
  data->mutex->lock();
  Thread::set_cancel_state(Thread::CANCEL_DISABLED, &(data->old_state));

  for (data->logit = data->loggers.begin(); data->logit != data->loggers.end(); ++data->logit) {
    (*data->logit)->log(level, component, e);
  }
  Thread::set_cancel_state(data->old_state);
  data->mutex->unlock();
}


void
MultiLogger::log_debug(const char *component, Exception &e)
{
  struct timeval now;
  gettimeofday(&now, NULL);
  data->mutex->lock();
  Thread::set_cancel_state(Thread::CANCEL_DISABLED, &(data->old_state));

  for (data->logit = data->loggers.begin(); data->logit != data->loggers.end(); ++data->logit) {
    (*data->logit)->tlog_debug(&now, component, e);
  }

  Thread::set_cancel_state(data->old_state);
  data->mutex->unlock();
}

void
MultiLogger::log_info(const char *component, Exception &e)
{
  struct timeval now;
  gettimeofday(&now, NULL);
  data->mutex->lock();
  Thread::set_cancel_state(Thread::CANCEL_DISABLED, &(data->old_state));

  for (data->logit = data->loggers.begin(); data->logit != data->loggers.end(); ++data->logit) {
    (*data->logit)->tlog_info(&now, component, e);
  }
  Thread::set_cancel_state(data->old_state);
  data->mutex->unlock();
}


void
MultiLogger::log_warn(const char *component, Exception &e)
{
  struct timeval now;
  gettimeofday(&now, NULL);
  data->mutex->lock();
  Thread::set_cancel_state(Thread::CANCEL_DISABLED, &(data->old_state));

  for (data->logit = data->loggers.begin(); data->logit != data->loggers.end(); ++data->logit) {
    (*data->logit)->tlog_warn(&now, component, e);
  }
  Thread::set_cancel_state(data->old_state);
  data->mutex->unlock();
}


void
MultiLogger::log_error(const char *component, Exception &e)
{
  struct timeval now;
  gettimeofday(&now, NULL);
  data->mutex->lock();
  Thread::set_cancel_state(Thread::CANCEL_DISABLED, &(data->old_state));

  for (data->logit = data->loggers.begin(); data->logit != data->loggers.end(); ++data->logit) {
    (*data->logit)->tlog_error(&now, component, e);
  }
  Thread::set_cancel_state(data->old_state);
  data->mutex->unlock();
}


void
MultiLogger::vlog(LogLevel level,
		  const char *component, const char *format, va_list va)
{
  struct timeval now;
  gettimeofday(&now, NULL);
  data->mutex->lock();
  Thread::set_cancel_state(Thread::CANCEL_DISABLED, &(data->old_state));

  for (data->logit = data->loggers.begin(); data->logit != data->loggers.end(); ++data->logit) {
    va_list vac;
    va_copy(vac, va);
    (*data->logit)->vlog(level, component, format, vac);
    va_end(vac);
  }
  Thread::set_cancel_state(data->old_state);
  data->mutex->unlock();
}


void
MultiLogger::vlog_debug(const char *component, const char *format, va_list va)
{
  struct timeval now;
  gettimeofday(&now, NULL);
  data->mutex->lock();
  Thread::set_cancel_state(Thread::CANCEL_DISABLED, &(data->old_state));

  for (data->logit = data->loggers.begin(); data->logit != data->loggers.end(); ++data->logit) {
    va_list vac;
    va_copy(vac, va);
    (*data->logit)->vtlog_debug(&now, component, format, vac);
    va_end(vac);
  }
  Thread::set_cancel_state(data->old_state);
  data->mutex->unlock();
}


void
MultiLogger::vlog_info(const char *component, const char *format, va_list va)
{
  struct timeval now;
  gettimeofday(&now, NULL);
  data->mutex->lock();
  Thread::set_cancel_state(Thread::CANCEL_DISABLED, &(data->old_state));

  for (data->logit = data->loggers.begin(); data->logit != data->loggers.end(); ++data->logit) {
    va_list vac;
    va_copy(vac, va);
    (*data->logit)->vtlog_info(&now, component, format, vac);
    va_end(vac);
  }
  Thread::set_cancel_state(data->old_state);
  data->mutex->unlock();
}


void
MultiLogger::vlog_warn(const char *component, const char *format, va_list va)
{
  struct timeval now;
  gettimeofday(&now, NULL);
  data->mutex->lock();
  Thread::set_cancel_state(Thread::CANCEL_DISABLED, &(data->old_state));

  for (data->logit = data->loggers.begin(); data->logit != data->loggers.end(); ++data->logit) {
    va_list vac;
    va_copy(vac, va);
    (*data->logit)->vtlog_warn(&now, component, format, vac);
    va_end(vac);
  }
  Thread::set_cancel_state(data->old_state);
  data->mutex->unlock();
}


void
MultiLogger::vlog_error(const char *component, const char *format, va_list va)
{
  struct timeval now;
  gettimeofday(&now, NULL);
  data->mutex->lock();
  Thread::set_cancel_state(Thread::CANCEL_DISABLED, &(data->old_state));

  for (data->logit = data->loggers.begin(); data->logit != data->loggers.end(); ++data->logit) {
    va_list vac;
    va_copy(vac, va);
    (*data->logit)->vtlog_error(&now, component, format, vac);
    va_end(vac);
  }
  Thread::set_cancel_state(data->old_state);
  data->mutex->unlock();
}

void
MultiLogger::tlog(LogLevel level, struct timeval *t,
		  const char *component, const char *format, ...)
{
  data->mutex->lock();
  Thread::set_cancel_state(Thread::CANCEL_DISABLED, &(data->old_state));
  va_list va;
  va_start(va, format);
  for (data->logit = data->loggers.begin(); data->logit != data->loggers.end(); ++data->logit) {
    va_list vac;
    va_copy(vac, va);
    (*data->logit)->vtlog(level, t, component, format, vac);
    va_end(vac);
  }
  va_end(va);
  Thread::set_cancel_state(data->old_state);
  data->mutex->unlock();
}


void
MultiLogger::tlog_debug(struct timeval *t, const char *component, const char *format, ...)
{
  data->mutex->lock();
  Thread::set_cancel_state(Thread::CANCEL_DISABLED, &(data->old_state));
  va_list va;
  va_start(va, format);
  for (data->logit = data->loggers.begin(); data->logit != data->loggers.end(); ++data->logit) {
    va_list vac;
    va_copy(vac, va);
    (*data->logit)->vlog_debug(component, format, vac);
    va_end(vac);
  }
  va_end(va);
  Thread::set_cancel_state(data->old_state);
  data->mutex->unlock();
}


void
MultiLogger::tlog_info(struct timeval *t, const char *component, const char *format, ...)
{
  data->mutex->lock();
  Thread::set_cancel_state(Thread::CANCEL_DISABLED, &(data->old_state));

  va_list va;
  va_start(va, format);
  for (data->logit = data->loggers.begin(); data->logit != data->loggers.end(); ++data->logit) {
    va_list vac;
    va_copy(vac, va);
    (*data->logit)->vtlog_info(t, component, format, vac);
    va_end(vac);
  }
  va_end(va);
  Thread::set_cancel_state(data->old_state);
  data->mutex->unlock();
}


void
MultiLogger::tlog_warn(struct timeval *t, const char *component, const char *format, ...)
{
  data->mutex->lock();
  Thread::set_cancel_state(Thread::CANCEL_DISABLED, &(data->old_state));

  va_list va;
  va_start(va, format);
  for (data->logit = data->loggers.begin(); data->logit != data->loggers.end(); ++data->logit) {
    va_list vac;
    va_copy(vac, va);
    (*data->logit)->vtlog_warn(t, component, format, vac);
    va_end(vac);
  }
  va_end(va);
  Thread::set_cancel_state(data->old_state);
  data->mutex->unlock();
}


void
MultiLogger::tlog_error(struct timeval *t, const char *component, const char *format, ...)
{
  data->mutex->lock();
  Thread::set_cancel_state(Thread::CANCEL_DISABLED, &(data->old_state));

  va_list va;
  va_start(va, format);
  for (data->logit = data->loggers.begin(); data->logit != data->loggers.end(); ++data->logit) {
    va_list vac;
    va_copy(vac, va);
    (*data->logit)->vtlog_error(t, component, format, vac);
    va_end(vac);
  }
  va_end(va);
  Thread::set_cancel_state(data->old_state);
  data->mutex->unlock();
}


void
MultiLogger::tlog(LogLevel level, struct timeval *t, const char *component, Exception &e)
{
  data->mutex->lock();
  Thread::set_cancel_state(Thread::CANCEL_DISABLED, &(data->old_state));

  for (data->logit = data->loggers.begin(); data->logit != data->loggers.end(); ++data->logit) {
    (*data->logit)->tlog(level, t, component, e);
  }
  Thread::set_cancel_state(data->old_state);
  data->mutex->unlock();
}


void
MultiLogger::tlog_debug(struct timeval *t, const char *component, Exception &e)
{
  for (data->logit = data->loggers.begin(); data->logit != data->loggers.end(); ++data->logit) {
    (*data->logit)->tlog_error(t, component, e);
  }
}

void
MultiLogger::tlog_info(struct timeval *t, const char *component, Exception &e)
{
  data->mutex->lock();
  Thread::set_cancel_state(Thread::CANCEL_DISABLED, &(data->old_state));

  for (data->logit = data->loggers.begin(); data->logit != data->loggers.end(); ++data->logit) {
    (*data->logit)->tlog_error(t, component, e);
  }
  Thread::set_cancel_state(data->old_state);
  data->mutex->unlock();
}


void
MultiLogger::tlog_warn(struct timeval *t, const char *component, Exception &e)
{
  data->mutex->lock();
  Thread::set_cancel_state(Thread::CANCEL_DISABLED, &(data->old_state));

  for (data->logit = data->loggers.begin(); data->logit != data->loggers.end(); ++data->logit) {
    (*data->logit)->tlog_error(t, component, e);
  }
  Thread::set_cancel_state(data->old_state);
  data->mutex->unlock();
}


void
MultiLogger::tlog_error(struct timeval *t, const char *component, Exception &e)
{
  data->mutex->lock();
  Thread::set_cancel_state(Thread::CANCEL_DISABLED, &(data->old_state));

  for (data->logit = data->loggers.begin(); data->logit != data->loggers.end(); ++data->logit) {
    (*data->logit)->tlog_error(t, component, e);
  }
  Thread::set_cancel_state(data->old_state);
  data->mutex->unlock();
}




void
MultiLogger::vtlog(LogLevel level, struct timeval *t,
		   const char *component, const char *format, va_list va)
{
  data->mutex->lock();
  Thread::set_cancel_state(Thread::CANCEL_DISABLED, &(data->old_state));

  for (data->logit = data->loggers.begin(); data->logit != data->loggers.end(); ++data->logit) {
    va_list vac;
    va_copy(vac, va);
    (*data->logit)->vtlog(level, t, component, format, vac);
    va_end(vac);
  }
  Thread::set_cancel_state(data->old_state);
  data->mutex->unlock();
}


void
MultiLogger::vtlog_debug(struct timeval *t, const char *component, const char *format, va_list va)
{
  data->mutex->lock();
  Thread::set_cancel_state(Thread::CANCEL_DISABLED, &(data->old_state));

  for (data->logit = data->loggers.begin(); data->logit != data->loggers.end(); ++data->logit) {
    va_list vac;
    va_copy(vac, va);
    (*data->logit)->vtlog_debug(t, component, format, vac);
    va_end(vac);
  }
  Thread::set_cancel_state(data->old_state);
  data->mutex->unlock();
}


void
MultiLogger::vtlog_info(struct timeval *t, const char *component, const char *format, va_list va)
{
  data->mutex->lock();
  Thread::set_cancel_state(Thread::CANCEL_DISABLED, &(data->old_state));

  for (data->logit = data->loggers.begin(); data->logit != data->loggers.end(); ++data->logit) {
    va_list vac;
    va_copy(vac, va);
    (*data->logit)->vtlog_info(t, component, format, vac);
    va_end(vac);
  }
  Thread::set_cancel_state(data->old_state);
  data->mutex->unlock();
}


void
MultiLogger::vtlog_warn(struct timeval *t, const char *component, const char *format, va_list va)
{
  data->mutex->lock();
  Thread::set_cancel_state(Thread::CANCEL_DISABLED, &(data->old_state));

  for (data->logit = data->loggers.begin(); data->logit != data->loggers.end(); ++data->logit) {
    va_list vac;
    va_copy(vac, va);
    (*data->logit)->vtlog_warn(t, component, format, vac);
    va_end(vac);
  }
  Thread::set_cancel_state(data->old_state);
  data->mutex->unlock();
}


void
MultiLogger::vtlog_error(struct timeval *t, const char *component, const char *format, va_list va)
{
  data->mutex->lock();
  Thread::set_cancel_state(Thread::CANCEL_DISABLED, &(data->old_state));

  for (data->logit = data->loggers.begin(); data->logit != data->loggers.end(); ++data->logit) {
    va_list vac;
    va_copy(vac, va);
    (*data->logit)->vtlog_error(t, component, format, vac);
    va_end(vac);
  }
  Thread::set_cancel_state(data->old_state);
  data->mutex->unlock();
}


} // end namespace fawkes
