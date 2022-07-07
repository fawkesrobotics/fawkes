
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
SyncInterfaceListener::SyncInterfaceListener(fawkes::Logger     *logger,
                                             fawkes::Interface  *reader,
                                             fawkes::Interface  *writer,
                                             fawkes::BlackBoard *reader_bb,
                                             fawkes::BlackBoard *writer_bb)
: BlackBoardInterfaceListener("SyncInterfaceListener(%s-%s)", writer->uid(), reader->id())
{
	logger_    = logger;
	reader_    = reader;
	writer_    = writer;
	reader_bb_ = reader_bb;
	writer_bb_ = writer_bb;

	bbil_add_data_interface(reader_);
	bbil_add_message_interface(writer_);
	bb_interface_data_refreshed(reader_);

	reader_bb_->register_listener(this, BlackBoard::BBIL_FLAG_DATA);
	writer_bb_->register_listener(this, BlackBoard::BBIL_FLAG_MESSAGES);
}

/** Destructor. */
SyncInterfaceListener::~SyncInterfaceListener()
{
	reader_bb_->unregister_listener(this);
	// The listener needs an update in order to restore its bbil_maps, which
	// is necessary in order to clean up the notifier of the second bb
	writer_bb_->update_listener(this, BlackBoard::BBIL_FLAG_ALL);
	writer_bb_->unregister_listener(this);
}

bool
SyncInterfaceListener::bb_interface_message_received(Interface *interface,
                                                     Message   *message) noexcept
{
	try {
		if (interface == writer_) {
			//logger_->log_debug(bbil_name(), "Forwarding message");
			logger_->log_debug(bbil_name(),
			                   "Forwarding message from sender %s, source %s",
			                   message->sender_id().get_string().c_str(),
			                   message->source_id().get_string().c_str());
			Message *m = message->clone();
			m->set_hops(message->hops());
			m->ref();
			reader_->msgq_enqueue(m, true);
			message->set_id(m->id());
			logger_->log_debug(bbil_name(),
			                   "Sender after enqueueing: %s",
			                   m->sender_id().get_string().c_str());
			logger_->log_debug(bbil_name(),
			                   "Source after enqueueing: %s",
			                   m->source_id().get_string().c_str());
			m->unref();
			return false;
		} else {
			// Don't know why we were called, let 'em enqueue
			logger_->log_error(bbil_name(), "Message received for unknown interface");
			return true;
		}
	} catch (Exception &e) {
		logger_->log_error(bbil_name(), "Exception when message received");
		logger_->log_error("SyncInterfaceListener", e);
		return false;
	}
}

void
SyncInterfaceListener::bb_interface_data_refreshed(Interface *interface) noexcept
{
	try {
		if (interface == reader_) {
			//logger_->log_debug(bbil_name(), "Copying data");
			reader_->read();
			writer_->copy_values(reader_);
			writer_->write();
		} else {
			// Don't know why we were called, let 'em enqueue
			logger_->log_error(bbil_name(), "Data changed for unknown interface");
		}
	} catch (Exception &e) {
		logger_->log_error(bbil_name(), "Exception when data changed");
		logger_->log_error(bbil_name(), e);
	}
}
