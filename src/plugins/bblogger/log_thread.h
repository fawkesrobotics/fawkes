
/***************************************************************************
 *  log_thread.h - BB Logger Thread
 *
 *  Created: Sat Nov 07 23:40:48 2009
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_BBLOGGER_LOG_THREAD_H_
#define __PLUGINS_BBLOGGER_LOG_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>
#include <aspect/clock.h>
#include <blackboard/interface_listener.h>

#include <core/utils/lock_queue.h>
#include <core/threading/thread_list.h>

#include <cstdio>

namespace fawkes {
  class BlackBoard;
  class Logger;
  class Mutex;
  class Time;
  class SwitchInterface;
}

class BBLoggerThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::ClockAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::BlackBoardInterfaceListener
{
 public:
  BBLoggerThread(const char *iface_uid,
		 const char *logdir, bool buffering, bool flushing,
		 const char *scenario, fawkes::Time *start_time);
  virtual ~BBLoggerThread();

  const char * get_filename() const;
  void set_threadlist(fawkes::ThreadList &thread_list);
  void set_enabled(bool enabled);

  virtual void init();
  virtual void finalize();
  virtual void loop();

  virtual bool bb_interface_message_received(fawkes::Interface *interface, fawkes::Message *message) throw();
  virtual void bb_interface_data_changed(fawkes::Interface *interface) throw();
  virtual void bb_interface_writer_added(fawkes::Interface *interface,
					 unsigned int instance_serial) throw();
  virtual void bb_interface_writer_removed(fawkes::Interface *interface,
					   unsigned int instance_serial) throw();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  void write_header();
  void update_header();
  void write_chunk(const void *chunk);


 private:
  fawkes::Interface  *__iface;

  unsigned int        __num_data_items;
  unsigned int        __session_start;

  bool                __enabled;
  bool                __buffering;
  bool                __flushing;
  size_t              __data_size;
  char               *__scenario;
  char               *__filename;
  char               *__logdir;
  char               *__uid;
  std::string         __type;
  std::string         __id;
  FILE               *__f_data;

  fawkes::Time       *__start;
  fawkes::Time       *__now;

  bool                __is_master;
  fawkes::ThreadList  __threads;
  fawkes::SwitchInterface *__switch_if;

  fawkes::Mutex      *__queue_mutex;
  unsigned int        __act_queue;
  fawkes::LockQueue<void *> __queues[2];
};


#endif
