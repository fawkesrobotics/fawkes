
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
#include <utils/logging/logger.h>

#include <blackboard/remote.h>
#include <interfaces/GameStateInterface.h>

#include <cstring>
#include <cstdlib>

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
RemoteBlackBoardRefBoxProcessor::RemoteBlackBoardRefBoxProcessor(
                                   Logger *logger,
				   const char *bb_host,
				   unsigned short int bb_port,
				   const char *iface_id)
  : __name("RBBRefBoxRep")
{
  __logger = logger;
  __rbb = NULL;
  __gamestate_if = NULL;

  __message_shown = false;

  __bb_host  = strdup(bb_host);
  __bb_port  = bb_port;
  __iface_id = strdup(iface_id);

  try {
    reconnect();
  } catch (Exception &e) {
    __logger->log_warn(__name, "Could not connect to remote blackboard, "
		       "will keep trying");
  }
}


/** Destructor. */
RemoteBlackBoardRefBoxProcessor::~RemoteBlackBoardRefBoxProcessor()
{
  free(__bb_host);
  free(__iface_id);
  if (__rbb) {
    __rbb->close(__gamestate_if);
    delete __rbb;
  }
}


/** Reconnect to refbox. */
void
RemoteBlackBoardRefBoxProcessor::reconnect()
{
  if ( __rbb ) {
    __rbb->close(__gamestate_if);
    delete __rbb;
  }
  __rbb = NULL;

  //  __logger->log_info(__name, "Trying to connect to blackboard at %s:%u",
  //		     __bb_host, __bb_port);
  try {
    __rbb = new RemoteBlackBoard(__bb_host, __bb_port);
    __gamestate_if = __rbb->open_for_reading<GameStateInterface>(__iface_id);
  } catch (Exception &e) {
    delete __rbb;
    __rbb = NULL;
    throw;
  }
}

void
RemoteBlackBoardRefBoxProcessor::refbox_process()
{
  if (__rbb && __rbb->is_alive() && __gamestate_if->is_valid()) {
    try {
      __gamestate_if->read();
      _rsh->set_gamestate(__gamestate_if->game_state(),
			  (worldinfo_gamestate_team_t)__gamestate_if->state_team());
      _rsh->set_score(__gamestate_if->score_cyan(),
		      __gamestate_if->score_magenta());
      _rsh->set_team_goal((worldinfo_gamestate_team_t)__gamestate_if->our_team(),
			  (worldinfo_gamestate_goalcolor_t)__gamestate_if->our_goal_color());
      _rsh->set_half((worldinfo_gamestate_half_t)__gamestate_if->half(),
		     __gamestate_if->is_kickoff());

    } catch (Exception &e) {
      __logger->log_warn(__name, "Processing BB data failed, exception follows");
      __logger->log_warn(__name, e);
    }
  }
}

bool
RemoteBlackBoardRefBoxProcessor::check_connection()
{
  if (! (__rbb && __rbb->is_alive() && __gamestate_if->is_valid())) {
    try {
      reconnect();
      __message_shown = false;
    } catch (Exception &e) {
      if (! __message_shown) {
	__logger->log_warn(__name, "Reconnect failed, exception follows");
	__logger->log_warn(__name, e);
	__message_shown = true;
      }
      return false;
    }
  }
  return true;
}
