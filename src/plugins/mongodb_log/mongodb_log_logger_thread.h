
/***************************************************************************
 *  mongodb_logger_thread.h - MongoDB logger thread
 *
 *  Created: Tue Dec 07 22:54:59 2010
 *  Copyright  2006-2010  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#ifndef __PLUGINS_MONGODB_MONGODB_LOGGER_THREAD_H_
#define __PLUGINS_MONGODB_MONGODB_LOGGER_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/logger.h>
#include <aspect/configurable.h>
#include <aspect/clock.h>
#include <plugins/mongodb/aspect/mongodb.h>

#include <string>

namespace fawkes {
  class Mutex;
}

class MongoLogLoggerThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::LoggerAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::ClockAspect,
  public fawkes::MongoDBAspect,
  public fawkes::Logger
{
 public:
  MongoLogLoggerThread();
  virtual ~MongoLogLoggerThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

  virtual void log_debug(const char *component, const char *format, ...);
  virtual void log_info(const char *component, const char *format, ...);
  virtual void log_warn(const char *component, const char *format, ...);
  virtual void log_error(const char *component, const char *format, ...);

  virtual void vlog_debug(const char *component, const char *format, va_list va);
  virtual void vlog_info(const char *component, const char *format, va_list va);
  virtual void vlog_warn(const char *component, const char *format, va_list va);
  virtual void vlog_error(const char *component, const char *format, va_list va);

  virtual void log_debug(const char *component, fawkes::Exception &e);
  virtual void log_info(const char *component, fawkes::Exception &e);
  virtual void log_warn(const char *component, fawkes::Exception &e);
  virtual void log_error(const char *component, fawkes::Exception &e);

  virtual void tlog_debug(struct timeval *t, const char *component,
			  const char *format, ...);
  virtual void tlog_info(struct timeval *t, const char *component,
			 const char *format, ...);
  virtual void tlog_warn(struct timeval *t, const char *component,
			 const char *format, ...);
  virtual void tlog_error(struct timeval *t, const char *component,
			  const char *format, ...);

  virtual void tlog_debug(struct timeval *t, const char *component,
			  fawkes::Exception &e);
  virtual void tlog_info(struct timeval *t, const char *component,
			 fawkes::Exception &e);
  virtual void tlog_warn(struct timeval *t, const char *component,
			 fawkes::Exception &e);
  virtual void tlog_error(struct timeval *t, const char *component,
			  fawkes::Exception &e);

  virtual void vtlog_debug(struct timeval *t, const char *component,
			   const char *format, va_list va);
  virtual void vtlog_info(struct timeval *t, const char *component,
			  const char *format, va_list va);
  virtual void vtlog_warn(struct timeval *t, const char *component,
			  const char *format, va_list va);
  virtual void vtlog_error(struct timeval *t, const char *component,
			   const char *format, va_list va);

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  void insert_message(LogLevel ll, const char *component, const char *format,
		      va_list va);
  void insert_message(LogLevel ll, const char *component, fawkes::Exception &e);
  void tlog_insert_message(LogLevel ll, struct timeval *t, const char *component,
			   const char *format, va_list va);
  void tlog_insert_message(LogLevel ll, struct timeval *t, const char *component,
			   fawkes::Exception &);

 private:
  std::string    __collection;
  fawkes::Mutex *__mutex;
};

#endif
