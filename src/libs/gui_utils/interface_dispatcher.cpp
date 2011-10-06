
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
  __message_enqueueing = message_enqueueing;

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
  __message_enqueueing = message_enqueueing;

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
  __dispatcher_data_changed.connect(sigc::mem_fun(*this, &InterfaceDispatcher::on_data_changed));
  __dispatcher_message_received.connect(sigc::mem_fun(*this, &InterfaceDispatcher::on_message_received));
  __dispatcher_writer_added.connect(sigc::mem_fun(*this, &InterfaceDispatcher::on_writer_added));
  __dispatcher_writer_removed.connect(sigc::mem_fun(*this, &InterfaceDispatcher::on_writer_removed));
  __dispatcher_reader_added.connect(sigc::mem_fun(*this, &InterfaceDispatcher::on_reader_added));
  __dispatcher_reader_removed.connect(sigc::mem_fun(*this, &InterfaceDispatcher::on_writer_removed));
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
  __message_enqueueing = enqueue;
}


/** Internal event handler.
 * Called by dispatcher to emit signal.
 */
void
InterfaceDispatcher::on_data_changed()
{
  __queue_data_changed.lock();
  while (! __queue_data_changed.empty()) {
    Interface *iface = __queue_data_changed.front();
    __signal_data_changed.emit(iface);
    __queue_data_changed.pop();
  }
  __queue_data_changed.unlock();
}


/** Internal event handler.
 * Called by dispatcher to emit signal.
 */
void
InterfaceDispatcher::on_message_received()
{
  __queue_message_received.lock();
  while (! __queue_message_received.empty()) {
    std::pair<Interface *, Message *> p = __queue_message_received.front();
    __signal_message_received.emit(p.first, p.second);
    p.second->unref();
    __queue_message_received.pop();
  }
  __queue_message_received.unlock();
}


/** Internal event handler.
 * Called by dispatcher to emit signal.
 */
void
InterfaceDispatcher::on_writer_added()
{
  __queue_writer_added.lock();
  while (! __queue_writer_added.empty()) {
    Interface *iface = __queue_writer_added.front();
    __signal_writer_added.emit(iface);
    __queue_writer_added.pop();
  }
  __queue_writer_added.unlock();
}


/** Internal event handler.
 * Called by dispatcher to emit signal.
 */
void
InterfaceDispatcher::on_writer_removed()
{
  __queue_writer_removed.lock();
  while (! __queue_writer_removed.empty()) {
    Interface *iface = __queue_writer_removed.front();
    __signal_writer_removed.emit(iface);
    __queue_writer_removed.pop();
  }
  __queue_writer_removed.unlock();
}


/** Internal event handler.
 * Called by dispatcher to emit signal.
 */
void
InterfaceDispatcher::on_reader_added()
{
  __queue_reader_added.lock();
  while (! __queue_reader_added.empty()) {
    Interface *iface = __queue_reader_added.front();
    __signal_reader_added.emit(iface);
    __queue_reader_added.pop();
  }
  __queue_reader_added.unlock();
}


/** Internal event handler.
 * Called by dispatcher to emit signal.
 */
void
InterfaceDispatcher::on_reader_removed()
{
  __queue_reader_removed.lock();
  while (! __queue_reader_removed.empty()) {
    Interface *iface = __queue_reader_removed.front();
    __signal_reader_removed.emit(iface);
    __queue_reader_removed.pop();
  }
  __queue_reader_removed.unlock();
}


void
InterfaceDispatcher::bb_interface_data_changed(Interface *interface) throw()
{
  __queue_data_changed.push_locked(interface);
  __dispatcher_data_changed();
}

bool
InterfaceDispatcher::bb_interface_message_received(Interface *interface, Message *message) throw()
{
  message->ref();
  __queue_message_received.push_locked(std::make_pair(interface, message));
  __dispatcher_message_received();
  return __message_enqueueing;
}

void
InterfaceDispatcher::bb_interface_writer_added(Interface *interface,
					       unsigned int instance_serial) throw()
{
  __queue_writer_added.push_locked(interface);
  __dispatcher_writer_added();
}

void
InterfaceDispatcher::bb_interface_writer_removed(Interface *interface,
						 unsigned int instance_serial) throw()
{
  __queue_writer_removed.push_locked(interface);
  __dispatcher_writer_removed();
}

void
InterfaceDispatcher::bb_interface_reader_added(Interface *interface,
					       unsigned int instance_serial) throw()
{
  __queue_reader_added.push_locked(interface);
  __dispatcher_reader_added();
}

void
InterfaceDispatcher::bb_interface_reader_removed(Interface *interface,
						 unsigned int instance_serial) throw()
{
  __queue_reader_removed.push_locked(interface);
  __dispatcher_reader_removed();
}

/** Get "data changed" signal.
 * The signal is emitted if the data of the interface has changed.
 * @return "data changed" signal.
 */
sigc::signal<void, Interface *>
InterfaceDispatcher::signal_data_changed()
{
  return __signal_data_changed;
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
  return __signal_message_received;
}


/** Get "writer added" signal.
 * The signal is emitted if a writer has been added to the interface.
 * @return "writer added" signal.
 */
sigc::signal<void, Interface *>
InterfaceDispatcher::signal_writer_added()
{
  return __signal_writer_added;
}


/** Get "writer removed" signal.
 * The signal is emitted if a writer has been removed from the interface.
 * @return "writer removed" signal.
 */
sigc::signal<void, Interface *>
InterfaceDispatcher::signal_writer_removed()
{
  return __signal_writer_removed;
}


/** Get "reader added" signal.
 * The signal is emitted if a reader has been added to the interface.
 * @return "reader added" signal.
 */
sigc::signal<void, Interface *>
InterfaceDispatcher::signal_reader_added()
{
  return __signal_reader_added;
}


/** Get "reader removed" signal.
 * The signal is emitted if a reader has been removed from the interface.
 * @return "reader added" signal.
 */
sigc::signal<void, Interface *>
InterfaceDispatcher::signal_reader_removed()
{
  return __signal_reader_removed;
}


} // end of namespace fawkes
