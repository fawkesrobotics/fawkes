
/***************************************************************************
 *  cache.h - Fawkes cache logger
 *
 *  Created: Wed Feb 11 22:54:23 2009
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

#ifndef __UTILS_LOGGING_CACHE_H_
#define __UTILS_LOGGING_CACHE_H_

#include <logging/logger.h>
#include <ctime>

#include <string>
#include <list>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class Mutex;

class CacheLogger : public Logger
{
 public:
  CacheLogger(unsigned int num_entries = 20, LogLevel log_level = LL_DEBUG);
  virtual ~CacheLogger();

  virtual void log_debug(const char *component, const char *format, ...);
  virtual void log_info(const char *component, const char *format, ...);
  virtual void log_warn(const char *component, const char *format, ...);
  virtual void log_error(const char *component, const char *format, ...);

  virtual void vlog_debug(const char *component, const char *format, va_list va);
  virtual void vlog_info(const char *component, const char *format, va_list va);
  virtual void vlog_warn(const char *component, const char *format, va_list va);
  virtual void vlog_error(const char *component, const char *format, va_list va);

  virtual void log_debug(const char *component, Exception &e);
  virtual void log_info(const char *component, Exception &e);
  virtual void log_warn(const char *component, Exception &e);
  virtual void log_error(const char *component, Exception &e);

  virtual void tlog_debug(struct timeval *t, const char *component, const char *format, ...);
  virtual void tlog_info(struct timeval *t, const char *component, const char *format, ...);
  virtual void tlog_warn(struct timeval *t, const char *component, const char *format, ...);
  virtual void tlog_error(struct timeval *t, const char *component, const char *format, ...);

  virtual void tlog_debug(struct timeval *t, const char *component, Exception &e);
  virtual void tlog_info(struct timeval *t, const char *component, Exception &e);
  virtual void tlog_warn(struct timeval *t, const char *component, Exception &e);
  virtual void tlog_error(struct timeval *t, const char *component, Exception &e);

  virtual void vtlog_debug(struct timeval *t, const char *component,
			   const char *format, va_list va);
  virtual void vtlog_info(struct timeval *t, const char *component,
			  const char *format, va_list va);
  virtual void vtlog_warn(struct timeval *t, const char *component,
			  const char *format, va_list va);
  virtual void vtlog_error(struct timeval *t, const char *component,
			   const char *format, va_list va);

  /** Cache entry struct. */
  typedef struct {
    LogLevel       log_level;	/**< log level */
    std::string    component;	/**< component */
    struct timeval time;	/**< raw time */
    std::string    timestr;	/**< Time encoded as string */
    std::string    message;	/**< Message */
  } CacheEntry;

  /** Get messages.
   * @return reference to message list
   */
  std::list<CacheEntry> &  get_messages();

  /** Clear messages. */
  void clear();

  unsigned int size() const;
  void set_size(unsigned int new_size);

  void lock();
  void unlock();

 private:
  void push_message(LogLevel ll, const char *component, const char *format,
		    va_list va);
  void push_message(LogLevel ll, const char *component, Exception &e);
  void tlog_push_message(LogLevel ll, struct timeval *t, const char *component,
			 const char *format, va_list va);
  void tlog_push_message(LogLevel ll, struct timeval *t, const char *component,
			 Exception &);


 private:
  struct ::tm  *now_s;
  Mutex        *mutex;

  std::list<CacheEntry> __messages;
  unsigned int  __num_entries;
  unsigned int  __max_num_entries;

};


} // end namespace fawkes

#endif
