
/***************************************************************************
 *  refbox_state_writer.h - Fawkes RefBox state writer
 *
 *  Created: Wed Apr 22 02:32:52 2009
 *  Copyright  2009       Christpoh Schwering
 *             2008-2009  Tim Niemueller [www.niemueller.de]
 *
 *  $Id: refbox_state_writer.h 1083 2008-05-21 15:34:51Z tim $
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

#ifndef __TOOLS_REFBOXREP_REFBOX_STATE_WRITER_H_
#define __TOOLS_REFBOXREP_REFBOX_STATE_WRITER_H_

#include "refbox_state_sender.h"

#include <blackboard/remote.h>
#include <interfaces/GameStateInterface.h>
#include <netcomm/worldinfo/enums.h>

#include <vector>
#include <string>
#include <map>

class RefBoxStateBBWriter : public RefBoxStateSender
{
 public:
  RefBoxStateBBWriter(std::vector<std::string> hosts, bool debug = false);
  virtual ~RefBoxStateBBWriter();

  virtual void send();
  virtual void set_gamestate(int game_state,
			     fawkes::worldinfo_gamestate_team_t state_team);
  virtual void set_score(unsigned int score_cyan, unsigned int score_magenta);
  virtual void set_team_goal(fawkes::worldinfo_gamestate_team_t our_team,
			     fawkes::worldinfo_gamestate_goalcolor_t goal_color);
  virtual void set_half(fawkes::worldinfo_gamestate_half_t half);

 private:
  void connect(const std::string &host);

  unsigned int __counter;

  std::map<fawkes::RemoteBlackBoard *, std::string> __rbbs;
  std::map<fawkes::RemoteBlackBoard *, fawkes::GameStateInterface *> __giss;

  bool                                    __debug;
  int                                     __game_state;
  fawkes::worldinfo_gamestate_team_t      __state_team;
  unsigned int                            __score_cyan;
  unsigned int                            __score_magenta;
  fawkes::worldinfo_gamestate_team_t      __our_team;
  fawkes::worldinfo_gamestate_goalcolor_t __our_goal_color;
  fawkes::worldinfo_gamestate_half_t      __half;
};

#endif
