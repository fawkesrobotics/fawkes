
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

namespace fawkes {
  class BlackBoard;
  class Logger;
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
		 const char *file_pattern);
  virtual ~BBLoggerThread();

  virtual void init();
  virtual void finalize();
  virtual void loop();

  virtual bool bb_interface_message_received(fawkes::Interface *interface, fawkes::Message *message) throw();
  virtual void bb_interface_data_changed(fawkes::Interface *interface) throw();

 private:
  fawkes::Interface  *__iface;

  char               *__filename;
  char               *__logdir;
  char               *__uid;
  int                 __fd_data;
  int                 __fd_msgs;
};


#endif
