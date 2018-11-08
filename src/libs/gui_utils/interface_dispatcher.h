
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

#ifndef _LIBS_GUI_UTILS_INTERFACE_DISPATCHER_H_
#define _LIBS_GUI_UTILS_INTERFACE_DISPATCHER_H_

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
  bool                                           message_enqueueing_;

  Glib::Dispatcher                               dispatcher_data_changed_;
  Glib::Dispatcher                               dispatcher_message_received_;
  Glib::Dispatcher                               dispatcher_writer_added_;
  Glib::Dispatcher                               dispatcher_writer_removed_;
  Glib::Dispatcher                               dispatcher_reader_added_;
  Glib::Dispatcher                               dispatcher_reader_removed_;

  sigc::signal<void, Interface *>                signal_data_changed_;
  sigc::signal<void, Interface *, Message *>     signal_message_received_;
  sigc::signal<void, Interface *>                signal_writer_added_;
  sigc::signal<void, Interface *>                signal_writer_removed_;
  sigc::signal<void, Interface *>                signal_reader_added_;
  sigc::signal<void, Interface *>                signal_reader_removed_;

  LockQueue<Interface *>                         queue_data_changed_;
  LockQueue<std::pair<Interface *, Message *> >  queue_message_received_;
  LockQueue<Interface *>                         queue_writer_added_;
  LockQueue<Interface *>                         queue_writer_removed_;
  LockQueue<Interface *>                         queue_reader_added_;
  LockQueue<Interface *>                         queue_reader_removed_;
};

}

#endif
