
/***************************************************************************
 *  remotebb.h - Fawkes remote blackboard processor
 *
 *  Created: Fri Apr 09 23:58:11 2010
 *  Copyright  2010  Tim Niemueller [www.niemueller.de]
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

#ifndef _PLUGINS_REFBOXCOMM_PROCESSOR_REMOTEBB_H_
#define _PLUGINS_REFBOXCOMM_PROCESSOR_REMOTEBB_H_

#include "processor.h"
#include "state_handler.h"

namespace fawkes {
class Logger;
class BlackBoard;
class GameStateInterface;
} // namespace fawkes

class RemoteBlackBoardRefBoxProcessor : public RefBoxProcessor
{
public:
	RemoteBlackBoardRefBoxProcessor(fawkes::Logger *   logger,
	                                const char *       bb_host,
	                                unsigned short int bb_port,
	                                const char *       iface_id);
	~RemoteBlackBoardRefBoxProcessor();

	bool check_connection();
	void refbox_process();

private: // methods
	void reconnect();

private:
	fawkes::Logger *    logger_;
	fawkes::BlackBoard *rbb_;

	fawkes::GameStateInterface *gamestate_if_;

	const char *name_;

	char *             bb_host_;
	unsigned short int bb_port_;
	char *             iface_id_;

	bool message_shown_;
};

#endif
