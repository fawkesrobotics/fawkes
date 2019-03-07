
/***************************************************************************
 *  remotebb.cpp - Fawkes remote blackboard processor
 *
 *  Created: Wed Apr 09 10:38:16 2008
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

#include "remotebb.h"

#include <blackboard/remote.h>
#include <interfaces/GameStateInterface.h>
#include <logging/logger.h>

#include <cstdlib>
#include <cstring>

using namespace fawkes;

/** @class RemoteBlackBoardRefBoxProcessor "processor/remotebb.h"
 * Remote BlackBoard refbox repeater.
 * This class will establish the connection to a remote blackboard and copy
 * the refbox information from there to the local state handler.
 * It can be used as a fallback for unicast communcation to a central
 * repeater host.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param logger logger for output
 * @param bb_host remote blackboard host
 * @param bb_port remote blackboard port
 * @param iface_id ID of the GameStateInterface on the remote blackboard
 */
RemoteBlackBoardRefBoxProcessor::RemoteBlackBoardRefBoxProcessor(Logger *           logger,
                                                                 const char *       bb_host,
                                                                 unsigned short int bb_port,
                                                                 const char *       iface_id)
: name_("RBBRefBoxRep")
{
	logger_       = logger;
	rbb_          = NULL;
	gamestate_if_ = NULL;

	message_shown_ = false;

	bb_host_  = strdup(bb_host);
	bb_port_  = bb_port;
	iface_id_ = strdup(iface_id);

	try {
		reconnect();
	} catch (Exception &e) {
		logger_->log_warn(name_,
		                  "Could not connect to remote blackboard, "
		                  "will keep trying");
	}
}

/** Destructor. */
RemoteBlackBoardRefBoxProcessor::~RemoteBlackBoardRefBoxProcessor()
{
	free(bb_host_);
	free(iface_id_);
	if (rbb_) {
		rbb_->close(gamestate_if_);
		delete rbb_;
	}
}

/** Reconnect to refbox. */
void
RemoteBlackBoardRefBoxProcessor::reconnect()
{
	if (rbb_) {
		rbb_->close(gamestate_if_);
		delete rbb_;
	}
	rbb_ = NULL;

	//  logger_->log_info(name_, "Trying to connect to blackboard at %s:%u",
	//		     bb_host_, bb_port_);
	try {
		rbb_          = new RemoteBlackBoard(bb_host_, bb_port_);
		gamestate_if_ = rbb_->open_for_reading<GameStateInterface>(iface_id_);
	} catch (Exception &e) {
		delete rbb_;
		rbb_ = NULL;
		throw;
	}
}

void
RemoteBlackBoardRefBoxProcessor::refbox_process()
{
	if (rbb_ && rbb_->is_alive() && gamestate_if_->is_valid()) {
		try {
			gamestate_if_->read();
			_rsh->set_gamestate(gamestate_if_->game_state(),
			                    (worldinfo_gamestate_team_t)gamestate_if_->state_team());
			_rsh->set_score(gamestate_if_->score_cyan(), gamestate_if_->score_magenta());
			_rsh->set_team_goal((worldinfo_gamestate_team_t)gamestate_if_->our_team(),
			                    (worldinfo_gamestate_goalcolor_t)gamestate_if_->our_goal_color());
			_rsh->set_half((worldinfo_gamestate_half_t)gamestate_if_->half(),
			               gamestate_if_->is_kickoff());

		} catch (Exception &e) {
			logger_->log_warn(name_, "Processing BB data failed, exception follows");
			logger_->log_warn(name_, e);
		}
	}
}

bool
RemoteBlackBoardRefBoxProcessor::check_connection()
{
	if (!(rbb_ && rbb_->is_alive() && gamestate_if_->is_valid())) {
		try {
			reconnect();
			message_shown_ = false;
		} catch (Exception &e) {
			if (!message_shown_) {
				logger_->log_warn(name_, "Reconnect failed, exception follows");
				logger_->log_warn(name_, e);
				message_shown_ = true;
			}
			return false;
		}
	}
	return true;
}
