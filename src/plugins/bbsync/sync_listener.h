
/***************************************************************************
 *  sync_listener.h - Sync Interface Listener
 *
 *  Created: Fri Jun 05 10:58:22 2009
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

#ifndef _PLUGINS_BBSYNC_SYNC_LISTENER_H_
#define _PLUGINS_BBSYNC_SYNC_LISTENER_H_

#include <blackboard/interface_listener.h>

namespace fawkes {
class BlackBoard;
class Logger;
} // namespace fawkes

class SyncInterfaceListener : public fawkes::BlackBoardInterfaceListener
{
public:
	SyncInterfaceListener(fawkes::Logger *    logger,
	                      fawkes::Interface * reader,
	                      fawkes::Interface * writer,
	                      fawkes::BlackBoard *reader_bb,
	                      fawkes::BlackBoard *writer_bb);
	virtual ~SyncInterfaceListener();

	virtual bool bb_interface_message_received(fawkes::Interface *interface,
	                                           fawkes::Message *  message) throw();
	virtual void bb_interface_data_refreshed(fawkes::Interface *interface) throw();

private:
	fawkes::Logger *logger_;

	fawkes::Interface *writer_;
	fawkes::Interface *reader_;

	fawkes::BlackBoard *writer_bb_;
	fawkes::BlackBoard *reader_bb_;
};

#endif
