
/***************************************************************************
 *  multi.h - Fawkes multi logger
 *
 *  Created: Mon May 07 16:44:15 2007
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

#include <utils/logging/multi.h>
#include <utils/logging/logger.h>

/// @cond INTERNALS
#include <core/utils/lock_list.h>
#include <core/threading/thread.h>

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


/** @class MultiLogger <utils/logging/multi.h>
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

  for (data->logit = data->loggers.begin(); data->logit != data->loggers.end(); ++data->logit) {
    (*data->logit)->set_loglevel(level);
  }
  Thread::set_cancel_state(data->old_state);
  data->mutex->unlock();
}


/** Log message of given log level.
 * @param level log level
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 */
void
MultiLogger::log(LogLevel level, const char *component, const char *format, ...)
{
  data->mutex->lock();
  Thread::set_cancel_state(Thread::CANCEL_DISABLED, &(data->old_state));
  va_list va;
  va_start(va, format);
  for (data->logit = data->loggers.begin(); data->logit != data->loggers.end(); ++data->logit) {
    (*data->logit)->vlog(level, component, format, va);
  }
  va_end(va);
  Thread::set_cancel_state(data->old_state);
  data->mutex->unlock();
}


/** Log debug message.
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 */
void
MultiLogger::log_debug(const char *component, const char *format, ...)
{
  data->mutex->lock();
  Thread::set_cancel_state(Thread::CANCEL_DISABLED, &(data->old_state));
  va_list va;
  va_start(va, format);
  for (data->logit = data->loggers.begin(); data->logit != data->loggers.end(); ++data->logit) {
    (*data->logit)->vlog_debug(component, format, va);
  }
  va_end(va);
  Thread::set_cancel_state(data->old_state);
  data->mutex->unlock();
}


/** Log informational message.
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 */
void
MultiLogger::log_info(const char *component, const char *format, ...)
{
  data->mutex->lock();
  Thread::set_cancel_state(Thread::CANCEL_DISABLED, &(data->old_state));

  va_list va;
  va_start(va, format);
  for (data->logit = data->loggers.begin(); data->logit != data->loggers.end(); ++data->logit) {
    (*data->logit)->vlog_info(component, format, va);
  }
  va_end(va);
  Thread::set_cancel_state(data->old_state);
  data->mutex->unlock();
}


/** Log warning message.
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 */
void
MultiLogger::log_warn(const char *component, const char *format, ...)
{
  data->mutex->lock();
  Thread::set_cancel_state(Thread::CANCEL_DISABLED, &(data->old_state));

  va_list va;
  va_start(va, format);
  for (data->logit = data->loggers.begin(); data->logit != data->loggers.end(); ++data->logit) {
    (*data->logit)->vlog_warn(component, format, va);
  }
  va_end(va);
  Thread::set_cancel_state(data->old_state);
  data->mutex->unlock();
}


/** Log error message.
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 */
void
MultiLogger::log_error(const char *component, const char *format, ...)
{
  data->mutex->lock();
  Thread::set_cancel_state(Thread::CANCEL_DISABLED, &(data->old_state));

  va_list va;
  va_start(va, format);
  for (data->logit = data->loggers.begin(); data->logit != data->loggers.end(); ++data->logit) {
    (*data->logit)->vlog_error(component, format, va);
  }
  va_end(va);
  Thread::set_cancel_state(data->old_state);
  data->mutex->unlock();
}


/** Log message for given log level.
 * @param level log level
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 * @param va variadic argument list
 */
void
MultiLogger::vlog(LogLevel level,
		  const char *component, const char *format, va_list va)
{
  data->mutex->lock();
  Thread::set_cancel_state(Thread::CANCEL_DISABLED, &(data->old_state));

  for (data->logit = data->loggers.begin(); data->logit != data->loggers.end(); ++data->logit) {
    (*data->logit)->vlog(level, component, format, va);
  }
  Thread::set_cancel_state(data->old_state);
  data->mutex->unlock();
}


/** Log debug message.
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 * @param va variadic argument list
 */
void
MultiLogger::vlog_debug(const char *component, const char *format, va_list va)
{
  data->mutex->lock();
  Thread::set_cancel_state(Thread::CANCEL_DISABLED, &(data->old_state));

  for (data->logit = data->loggers.begin(); data->logit != data->loggers.end(); ++data->logit) {
    (*data->logit)->vlog_debug(component, format, va);
  }
  Thread::set_cancel_state(data->old_state);
  data->mutex->unlock();
}


/** Log informational message.
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 * @param va variadic argument list
 */
void
MultiLogger::vlog_info(const char *component, const char *format, va_list va)
{
  data->mutex->lock();
  Thread::set_cancel_state(Thread::CANCEL_DISABLED, &(data->old_state));

  for (data->logit = data->loggers.begin(); data->logit != data->loggers.end(); ++data->logit) {
    (*data->logit)->vlog_info(component, format, va);
  }
  Thread::set_cancel_state(data->old_state);
  data->mutex->unlock();
}


/** Log warning message.
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 * @param va variadic argument list
 */
void
MultiLogger::vlog_warn(const char *component, const char *format, va_list va)
{
  data->mutex->lock();
  Thread::set_cancel_state(Thread::CANCEL_DISABLED, &(data->old_state));

  for (data->logit = data->loggers.begin(); data->logit != data->loggers.end(); ++data->logit) {
    (*data->logit)->vlog_warn(component, format, va);
  }
  Thread::set_cancel_state(data->old_state);
  data->mutex->unlock();
}


/** Log error message.
 * @param component component, used to distuinguish logged messages
 * @param format format of the message, see man page of sprintf for available
 * tokens.
 * @param va variadic argument list
 */
void
MultiLogger::vlog_error(const char *component, const char *format, va_list va)
{
  data->mutex->lock();
  Thread::set_cancel_state(Thread::CANCEL_DISABLED, &(data->old_state));

  for (data->logit = data->loggers.begin(); data->logit != data->loggers.end(); ++data->logit) {
    (*data->logit)->vlog_error(component, format, va);
  }
  Thread::set_cancel_state(data->old_state);
  data->mutex->unlock();
}



/** Log exception for given log level.
 * @param level log level
 * @param component component, used to distuinguish logged messages
 * @param e exception to log, exception messages will be logged
 */
void
MultiLogger::log(LogLevel level, const char *component, Exception &e)
{
  data->mutex->lock();
  Thread::set_cancel_state(Thread::CANCEL_DISABLED, &(data->old_state));

  for (data->logit = data->loggers.begin(); data->logit != data->loggers.end(); ++data->logit) {
    (*data->logit)->log(level, component, e);
  }
  Thread::set_cancel_state(data->old_state);
  data->mutex->unlock();
}


/** Log debug message.
 * @param component component, used to distuinguish logged messages
 * @param e exception to log, exception messages will be logged
 */
void
MultiLogger::log_debug(const char *component, Exception &e)
{
  for (data->logit = data->loggers.begin(); data->logit != data->loggers.end(); ++data->logit) {
    (*data->logit)->log_error(component, e);
  }
}

/** Log informational message.
 * @param component component, used to distuinguish logged messages
 * @param e exception to log, exception messages will be logged
 */
void
MultiLogger::log_info(const char *component, Exception &e)
{
  data->mutex->lock();
  Thread::set_cancel_state(Thread::CANCEL_DISABLED, &(data->old_state));

  for (data->logit = data->loggers.begin(); data->logit != data->loggers.end(); ++data->logit) {
    (*data->logit)->log_error(component, e);
  }
  Thread::set_cancel_state(data->old_state);
  data->mutex->unlock();
}


/** Log warning message.
 * @param component component, used to distuinguish logged messages
 * @param e exception to log, exception messages will be logged
 */
void
MultiLogger::log_warn(const char *component, Exception &e)
{
  data->mutex->lock();
  Thread::set_cancel_state(Thread::CANCEL_DISABLED, &(data->old_state));

  for (data->logit = data->loggers.begin(); data->logit != data->loggers.end(); ++data->logit) {
    (*data->logit)->log_error(component, e);
  }
  Thread::set_cancel_state(data->old_state);
  data->mutex->unlock();
}


/** Log error message.
 * @param component component, used to distuinguish logged messages
 * @param e exception to log, exception messages will be logged
 */
void
MultiLogger::log_error(const char *component, Exception &e)
{
  data->mutex->lock();
  Thread::set_cancel_state(Thread::CANCEL_DISABLED, &(data->old_state));

  for (data->logit = data->loggers.begin(); data->logit != data->loggers.end(); ++data->logit) {
    (*data->logit)->log_error(component, e);
  }
  Thread::set_cancel_state(data->old_state);
  data->mutex->unlock();
}
