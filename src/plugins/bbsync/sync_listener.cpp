
/***************************************************************************
 *  sync_listener.cpp - Sync Interface Listener
 *
 *  Created: Fri Jun 05 11:01:23 2009
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

#include "sync_listener.h"

#include <blackboard/blackboard.h>
#include <logging/logger.h>

using namespace fawkes;

/** @class SyncInterfaceListener "sync_listener.h"
 * Synchronize two interfaces.
 * This class synchronizes two interfaces, a reading and a writing instance
 * of the same type. To accomplish this it listens for data changed and message
 * events and forwards them as appropriate to "the other side".
 * @author Tim Niemueller
 */

/** Constructor.
 * Automatically registers the listener with the (two) blackboards as
 * appropriate. It also automatically unregisters in the destructor.
 * @param logger logger to write informational output to
 * @param reader reading interface instance
 * @param writer writing interface instance of the same type as \p reader
 * @param reader_bb the BlackBoard instance the reading instance has been
 * created on
 * @param writer_bb the BlackBoard instance the writing instance has been
 * created on
 */
SyncInterfaceListener::SyncInterfaceListener(fawkes::Logger *logger,
					     fawkes::Interface *reader,
					     fawkes::Interface *writer,
					     fawkes::BlackBoard *reader_bb,
					     fawkes::BlackBoard *writer_bb)
  : BlackBoardInterfaceListener("SyncInterfaceListener(%s-%s)", writer->uid(), reader->id())
{
  __logger    = logger;
  __reader    = reader;
  __writer    = writer;
  __reader_bb = reader_bb;
  __writer_bb = writer_bb;

  bbil_add_data_interface(__reader);
  bbil_add_message_interface(__writer);
  bb_interface_data_changed(__reader);

  __reader_bb->register_listener(this, BlackBoard::BBIL_FLAG_DATA);
  __writer_bb->register_listener(this, BlackBoard::BBIL_FLAG_MESSAGES);
}


/** Destructor. */
SyncInterfaceListener::~SyncInterfaceListener()
{
  __reader_bb->unregister_listener(this);
  __writer_bb->unregister_listener(this);
}


bool
SyncInterfaceListener::bb_interface_message_received(Interface *interface,
						     Message *message) throw()
{
  try {
    if ( interface == __writer ) {
      //__logger->log_debug(bbil_name(), "Forwarding message");
      Message *m = message->clone();
      m->set_hops(message->hops());
      m->ref();
      __reader->msgq_enqueue(m);
      message->set_id(m->id());
      m->unref();
      return false;
    } else {
      // Don't know why we were called, let 'em enqueue
	__logger->log_error(bbil_name(), "Message received for unknown interface");
      return true;
    }
  } catch (Exception &e) {
      __logger->log_error(bbil_name(), "Exception when message received");
    __logger->log_error("SyncInterfaceListener", e);
    return false;
  }
}


void
SyncInterfaceListener::bb_interface_data_changed(Interface *interface) throw()
{
  try {
    if ( interface == __reader ) {
      //__logger->log_debug(bbil_name(), "Copying data");
      __reader->read();
      __writer->copy_values(__reader);
      __writer->write();
    } else {
      // Don't know why we were called, let 'em enqueue
      __logger->log_error(bbil_name(), "Data changed for unknown interface");
    }
  } catch (Exception &e) {
    __logger->log_error(bbil_name(), "Exception when data changed");
    __logger->log_error(bbil_name(), e);
  }
}
