
/***************************************************************************
 *  interface_dispatcher.cpp - BlackBoard listener and dispatcher
 *
 *  Created: Thu Oct 09 23:04:59 2008
 *  Copyright  2008  Tim Niemueller [www.niemueller.de]
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

#ifndef __LIBS_GUI_UTILS_INTERFACE_DISPATCHER_H_
#define __LIBS_GUI_UTILS_INTERFACE_DISPATCHER_H_

#include <cstddef>
#include <glibmm/dispatcher.h>
#include <blackboard/interface_listener.h>
#include <core/utils/lock_queue.h>

namespace fawkes {
class Interface;

class InterfaceDispatcher
: public BlackBoardInterfaceListener
{
 public:
  InterfaceDispatcher(const char *listener_name, fawkes::Interface *iface,
		      bool message_enqueueing = true);

  InterfaceDispatcher(const char *listener_name_prefix,
                      std::list<fawkes::Interface *> ifaces,
		      bool message_enqueueing = true);

  void set_message_enqueueing(bool enqueue);

  sigc::signal<void, Interface *>               signal_data_changed();
  sigc::signal<void, Interface *, Message *>    signal_message_received();
  sigc::signal<void, Interface *>               signal_writer_added();
  sigc::signal<void, Interface *>               signal_writer_removed();
  sigc::signal<void, Interface *>               signal_reader_added();
  sigc::signal<void, Interface *>               signal_reader_removed();

  virtual void bb_interface_data_changed(Interface *interface) throw();
  virtual bool bb_interface_message_received(Interface *interface, Message *message) throw();
  virtual void bb_interface_writer_added(Interface *interface,
					 unsigned int instance_serial) throw();
  virtual void bb_interface_writer_removed(Interface *interface,
					   unsigned int instance_serial) throw();
  virtual void bb_interface_reader_added(Interface *interface,
					 unsigned int instance_serial) throw();
  virtual void bb_interface_reader_removed(Interface *interface,
					   unsigned int instance_serial) throw();

 protected:
  virtual void on_data_changed();
  virtual void on_message_received();
  virtual void on_writer_added();
  virtual void on_writer_removed();
  virtual void on_reader_added();
  virtual void on_reader_removed();

 private:
  void setup_signals();

 private:
  bool                                           __message_enqueueing;

  Glib::Dispatcher                               __dispatcher_data_changed;
  Glib::Dispatcher                               __dispatcher_message_received;
  Glib::Dispatcher                               __dispatcher_writer_added;
  Glib::Dispatcher                               __dispatcher_writer_removed;
  Glib::Dispatcher                               __dispatcher_reader_added;
  Glib::Dispatcher                               __dispatcher_reader_removed;

  sigc::signal<void, Interface *>                __signal_data_changed;
  sigc::signal<void, Interface *, Message *>     __signal_message_received;
  sigc::signal<void, Interface *>                __signal_writer_added;
  sigc::signal<void, Interface *>                __signal_writer_removed;
  sigc::signal<void, Interface *>                __signal_reader_added;
  sigc::signal<void, Interface *>                __signal_reader_removed;

  LockQueue<Interface *>                         __queue_data_changed;
  LockQueue<std::pair<Interface *, Message *> >  __queue_message_received;
  LockQueue<Interface *>                         __queue_writer_added;
  LockQueue<Interface *>                         __queue_writer_removed;
  LockQueue<Interface *>                         __queue_reader_added;
  LockQueue<Interface *>                         __queue_reader_removed;
};

}

#endif
