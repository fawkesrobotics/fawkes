
/***************************************************************************
 *  interface_dispatcher.cpp - BlackBoard listener and dispatcher
 *
 *  Created: Thu Oct 09 23:07:16 2008
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

#include <gui_utils/interface_dispatcher.h>
#include <interface/interface.h>

namespace fawkes {

/** @class InterfaceDispatcher <gui_utils/interface_dispatcher.h>
 * Interface listener with dispatcher.
 * An instance is used to react to a data changed event by triggering a
 * signal dispatcher (which is thread-safe and can be used across thread borders
 * in Glib/Gtk apps.
 * You have to register this listener with BlackBoard::BBIL_FLAGS_DATA flag by
 * yourself. Do not forget to unregister.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param listener_name name of the listener
 * @param iface interface to watch for data changes. Register this dispatcher as
 * listener by yourself!
 * @param message_enqueueing true to enqueue messages after the message received
 * event handler has been called, false to drop the message afterwards.
 */
InterfaceDispatcher::InterfaceDispatcher(const char *listener_name,
					 Interface *iface,
					 bool message_enqueueing)
  : BlackBoardInterfaceListener(listener_name)
{
  message_enqueueing_ = message_enqueueing;

  bbil_add_data_interface(iface);
  if ( iface->is_writer() ) {
    bbil_add_message_interface(iface);
  }
  bbil_add_writer_interface(iface);
  bbil_add_reader_interface(iface);

  setup_signals();
}

/** Multi interface constructor.
 * @param listener_name name of the listener
 * @param ifaces list of interfaces to watch for data
 * changes. Register this dispatcher as listener by yourself!
 * @param message_enqueueing true to enqueue messages after the
 * message received event handler has been called, false to drop the
 * message afterwards.
 */
InterfaceDispatcher::InterfaceDispatcher(const char *listener_name,
					 std::list<Interface *> ifaces,
					 bool message_enqueueing)
  : BlackBoardInterfaceListener(listener_name)
{
  message_enqueueing_ = message_enqueueing;

  std::list<Interface *>::iterator i;
  for (i = ifaces.begin(); i != ifaces.end(); ++i) {
    bbil_add_data_interface(*i);
    if ( (*i)->is_writer() ) {
      bbil_add_message_interface(*i);
    }
    bbil_add_writer_interface(*i);
    bbil_add_reader_interface(*i);
  }

  setup_signals();
}


void
InterfaceDispatcher::setup_signals()
{
  dispatcher_data_changed_.connect(sigc::mem_fun(*this, &InterfaceDispatcher::on_data_changed));
  dispatcher_message_received_.connect(sigc::mem_fun(*this, &InterfaceDispatcher::on_message_received));
  dispatcher_writer_added_.connect(sigc::mem_fun(*this, &InterfaceDispatcher::on_writer_added));
  dispatcher_writer_removed_.connect(sigc::mem_fun(*this, &InterfaceDispatcher::on_writer_removed));
  dispatcher_reader_added_.connect(sigc::mem_fun(*this, &InterfaceDispatcher::on_reader_added));
  dispatcher_reader_removed_.connect(sigc::mem_fun(*this, &InterfaceDispatcher::on_writer_removed));
}

/** Set if received messages should be enqueued or not.
 * The message received event handler can cause the message to be enqueued or not.
 * The default is to enqueue the messages.
 * @param enqueue true to cause messages to be enqueued, false to cause the
 * messages not to be enqueued after they have been processed
 */
void
InterfaceDispatcher::set_message_enqueueing(bool enqueue)
{
  message_enqueueing_ = enqueue;
}


/** Internal event handler.
 * Called by dispatcher to emit signal.
 */
void
InterfaceDispatcher::on_data_changed()
{
  queue_data_changed_.lock();
  while (! queue_data_changed_.empty()) {
    Interface *iface = queue_data_changed_.front();
    signal_data_changed_.emit(iface);
    queue_data_changed_.pop();
  }
  queue_data_changed_.unlock();
}


/** Internal event handler.
 * Called by dispatcher to emit signal.
 */
void
InterfaceDispatcher::on_message_received()
{
  queue_message_received_.lock();
  while (! queue_message_received_.empty()) {
    std::pair<Interface *, Message *> p = queue_message_received_.front();
    signal_message_received_.emit(p.first, p.second);
    p.second->unref();
    queue_message_received_.pop();
  }
  queue_message_received_.unlock();
}


/** Internal event handler.
 * Called by dispatcher to emit signal.
 */
void
InterfaceDispatcher::on_writer_added()
{
  queue_writer_added_.lock();
  while (! queue_writer_added_.empty()) {
    Interface *iface = queue_writer_added_.front();
    signal_writer_added_.emit(iface);
    queue_writer_added_.pop();
  }
  queue_writer_added_.unlock();
}


/** Internal event handler.
 * Called by dispatcher to emit signal.
 */
void
InterfaceDispatcher::on_writer_removed()
{
  queue_writer_removed_.lock();
  while (! queue_writer_removed_.empty()) {
    Interface *iface = queue_writer_removed_.front();
    signal_writer_removed_.emit(iface);
    queue_writer_removed_.pop();
  }
  queue_writer_removed_.unlock();
}


/** Internal event handler.
 * Called by dispatcher to emit signal.
 */
void
InterfaceDispatcher::on_reader_added()
{
  queue_reader_added_.lock();
  while (! queue_reader_added_.empty()) {
    Interface *iface = queue_reader_added_.front();
    signal_reader_added_.emit(iface);
    queue_reader_added_.pop();
  }
  queue_reader_added_.unlock();
}


/** Internal event handler.
 * Called by dispatcher to emit signal.
 */
void
InterfaceDispatcher::on_reader_removed()
{
  queue_reader_removed_.lock();
  while (! queue_reader_removed_.empty()) {
    Interface *iface = queue_reader_removed_.front();
    signal_reader_removed_.emit(iface);
    queue_reader_removed_.pop();
  }
  queue_reader_removed_.unlock();
}


void
InterfaceDispatcher::bb_interface_data_changed(Interface *interface) throw()
{
  queue_data_changed_.push_locked(interface);
  dispatcher_data_changed_();
}

bool
InterfaceDispatcher::bb_interface_message_received(Interface *interface, Message *message) throw()
{
  message->ref();
  queue_message_received_.push_locked(std::make_pair(interface, message));
  dispatcher_message_received_();
  return message_enqueueing_;
}

void
InterfaceDispatcher::bb_interface_writer_added(Interface *interface,
					       unsigned int instance_serial) throw()
{
  queue_writer_added_.push_locked(interface);
  dispatcher_writer_added_();
}

void
InterfaceDispatcher::bb_interface_writer_removed(Interface *interface,
						 unsigned int instance_serial) throw()
{
  queue_writer_removed_.push_locked(interface);
  dispatcher_writer_removed_();
}

void
InterfaceDispatcher::bb_interface_reader_added(Interface *interface,
					       unsigned int instance_serial) throw()
{
  queue_reader_added_.push_locked(interface);
  dispatcher_reader_added_();
}

void
InterfaceDispatcher::bb_interface_reader_removed(Interface *interface,
						 unsigned int instance_serial) throw()
{
  queue_reader_removed_.push_locked(interface);
  dispatcher_reader_removed_();
}

/** Get "data changed" signal.
 * The signal is emitted if the data of the interface has changed.
 * @return "data changed" signal.
 */
sigc::signal<void, Interface *>
InterfaceDispatcher::signal_data_changed()
{
  return signal_data_changed_;
}


/** Get "message received" signal.
 * The signal is emitted if a message has been received via the watched
 * interface. Note that this signal is only emitted on writing instances of
 * an interface.
 * @return "message received" signal.
 */
sigc::signal<void, Interface *, Message *>
InterfaceDispatcher::signal_message_received()
{
  return signal_message_received_;
}


/** Get "writer added" signal.
 * The signal is emitted if a writer has been added to the interface.
 * @return "writer added" signal.
 */
sigc::signal<void, Interface *>
InterfaceDispatcher::signal_writer_added()
{
  return signal_writer_added_;
}


/** Get "writer removed" signal.
 * The signal is emitted if a writer has been removed from the interface.
 * @return "writer removed" signal.
 */
sigc::signal<void, Interface *>
InterfaceDispatcher::signal_writer_removed()
{
  return signal_writer_removed_;
}


/** Get "reader added" signal.
 * The signal is emitted if a reader has been added to the interface.
 * @return "reader added" signal.
 */
sigc::signal<void, Interface *>
InterfaceDispatcher::signal_reader_added()
{
  return signal_reader_added_;
}


/** Get "reader removed" signal.
 * The signal is emitted if a reader has been removed from the interface.
 * @return "reader added" signal.
 */
sigc::signal<void, Interface *>
InterfaceDispatcher::signal_reader_removed()
{
  return signal_reader_removed_;
}


} // end of namespace fawkes
