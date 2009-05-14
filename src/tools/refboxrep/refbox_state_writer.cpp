
/***************************************************************************
 *  refbox_state_writer.cpp - Fawkes RefBox state writer
 *
 *  Created: Wed Apr 09 10:19:27 2008
 *  Copyright  2008  Tim Niemueller [www.niemueller.de]
 *
 *  $Id: refbox_state_writer.cpp 2032 2009-03-27 19:09:57Z tim $
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

#include "refbox_state_writer.h"

#include <netcomm/worldinfo/transceiver.h>
#include <utils/time/time.h>
#include <utils/time/clock.h>

#include <set>
#include <cstdio>

#define log(...)    if (__debug) {\
                      printf("%3u %s ", __counter, get_time().c_str());\
                      printf(__VA_ARGS__);\
                      fflush(stdout);\
                    }

using namespace std;
using namespace fawkes;

namespace {
std::string get_time() throw()
{
  Clock* c = Clock::instance();
  Time t = c->now();
  char* buf = new char[Time::TIMESTR_SIZE];
  t.str_r(buf, true);
  std::string str = buf;
  delete buf;
  std::string::size_type from =
    1+str.find_first_of(' ',
      1+str.find_first_of(' ',
        1+str.find_first_of(' ')));
  std::string::size_type to = str.find_last_of(' ');
  return str.substr(from, to - from + 1);
}
}

/** @class RefBoxStateBBWriter "refbox_state_writer.h"
 * RefBox repeater state writer.
 * Writes to GameStateInterfaces "WM GameState" of given hosts.
 * @author Christoph Schwering
 */

/** Constructor.
 * @param hosts hosts to connect to to send game state info via remote
 * blackboard
 * @param debug true to enable debug output
 */
RefBoxStateBBWriter::RefBoxStateBBWriter(vector<string> hosts, bool debug)
{
  __counter = 0;
  __debug = debug;

  __game_state = GS_FROZEN;
  __state_team = TEAM_BOTH;
  __score_cyan = 0;
  __score_magenta = 0;
  __our_team = TEAM_CYAN;
  __our_goal_color = GOAL_BLUE;
  __half = HALF_FIRST;

  for (vector<string>::const_iterator it = hosts.begin(); it != hosts.end();
      it++) {
    connect(*it);
  }
}


/** Destructor. */
RefBoxStateBBWriter::~RefBoxStateBBWriter()
{
  for (map<RemoteBlackBoard*, GameStateInterface*>::iterator it = __giss.begin();
      it != __giss.end(); it++) {
    RemoteBlackBoard* rbb = it->first;
    GameStateInterface* gis = it->second;
    rbb->close(gis);
    delete rbb;
  }
}


/* Connects to a host and opens and stores the interface at the right place. */
void RefBoxStateBBWriter::connect(const string& host) 
{
  try {
    RemoteBlackBoard* rbb = new RemoteBlackBoard(host.c_str(), 1910);
    __rbbs[rbb] = host;
    GameStateInterface* gis = static_cast<GameStateInterface*>(rbb->open_for_writing("GameStateInterface", "WM GameState"));
    __giss[rbb] = gis;
    log("Successfully connected to %s\n", host.c_str());
    set_gamestate(__game_state, __state_team);
    set_score(__score_cyan, __score_magenta);
    set_team_goal(__our_team, __our_goal_color);
    set_half(__half);
    gis->write();
  } catch (Exception& e) {
    log("Connecting to %s failed\n", host.c_str());
    e.print_trace();
    log("\n");
    log("\n");
  }
}

/** Set current game state.
 * @param game_state current game state
 * @param state_team team referenced by the game state
 */
void
RefBoxStateBBWriter::set_gamestate(worldinfo_gamestate_t game_state,
    worldinfo_gamestate_team_t state_team)
{
  log("Setting gamestate to '%s' for team '%s'\n",
      worldinfo_gamestate_tostring(game_state),
      worldinfo_gamestate_team_tostring(state_team));

  __game_state = game_state;
  __state_team = state_team;

  for (map<RemoteBlackBoard*,GameStateInterface*>::iterator it = __giss.begin(); it != __giss.end(); it++) {
    GameStateInterface* gis = it->second;
    switch (game_state)
      {
      case(GS_FROZEN):
        gis->set_game_state( GameStateInterface::GS_FROZEN );
        break;

      case(GS_PLAY):
        gis->set_game_state( GameStateInterface::GS_PLAY );
        break;

      case(GS_KICK_OFF):
        gis->set_game_state( GameStateInterface::GS_KICK_OFF );
        break;

      case(GS_DROP_BALL):
        gis->set_game_state( GameStateInterface::GS_DROP_BALL );
        break;

      case(GS_PENALTY):
        gis->set_game_state( GameStateInterface::GS_PENALTY );
        break;

      case(GS_CORNER_KICK):
        gis->set_game_state( GameStateInterface::GS_CORNER_KICK );
        break;

      case(GS_THROW_IN):
        gis->set_game_state( GameStateInterface::GS_THROW_IN );
        break;

      case(GS_FREE_KICK):
        gis->set_game_state( GameStateInterface::GS_FREE_KICK );
        break;

      case(GS_GOAL_KICK):
        gis->set_game_state( GameStateInterface::GS_GOAL_KICK );
        break;

      case(GS_HALF_TIME):
        gis->set_game_state( GameStateInterface::GS_HALF_TIME );
        break;
      }

    switch (state_team)
      {
      case(TEAM_NONE):
        gis->set_state_team( GameStateInterface::TEAM_NONE );
        break;

      case(TEAM_CYAN):
        gis->set_state_team( GameStateInterface::TEAM_CYAN );
        break;

      case(TEAM_MAGENTA):
        gis->set_state_team( GameStateInterface::TEAM_MAGENTA );
        break;

      case(TEAM_BOTH):
        gis->set_state_team( GameStateInterface::TEAM_BOTH );
        break;
      }
  }
}


/** Set score.
 * @param score_cyan current score of team cyan
 * @param score_magenta current score of team magenta
 */
void
RefBoxStateBBWriter::set_score(unsigned int score_cyan, unsigned int score_magenta)
{
  log("Setting score to %u:%u (cyan:magenta)\n", score_cyan, score_magenta);

  __score_cyan = score_cyan;
  __score_magenta = score_magenta;

  for (map<RemoteBlackBoard*,GameStateInterface*>::iterator it = __giss.begin(); it != __giss.end(); it++) {
    GameStateInterface* gis = it->second;
    gis->set_score_cyan( score_cyan );
    gis->set_score_magenta( score_magenta );
  }
}


/** Set team and goal info.
 * @param our_team our team color
 * @param goal_color our goal color
 */
void
RefBoxStateBBWriter::set_team_goal(worldinfo_gamestate_team_t our_team,
    worldinfo_gamestate_goalcolor_t goal_color)
{
  log("Setting team color to '%s' and goal color to '%s'\n",
      worldinfo_gamestate_team_tostring(our_team),
      worldinfo_gamestate_goalcolor_tostring(goal_color));

  __our_team = our_team;
  __our_goal_color = goal_color;

  for (map<RemoteBlackBoard*,GameStateInterface*>::iterator it = __giss.begin(); it != __giss.end(); it++) {
    GameStateInterface* gis = it->second;
    if (our_team == TEAM_CYAN) {
      gis->set_our_team( GameStateInterface::TEAM_CYAN );
    } else {
      gis->set_our_team( GameStateInterface::TEAM_MAGENTA );
    }

    if (goal_color == GOAL_BLUE) {
      gis->set_our_goal_color( GameStateInterface::GOAL_BLUE );
    } else {
      gis->set_our_goal_color( GameStateInterface::GOAL_YELLOW );
    }
  }
}


/** Set current half of the game time.
 * @param half current half
 */
void
RefBoxStateBBWriter::set_half(worldinfo_gamestate_half_t half)
{
  log("Setting half to '%s'\n",
      worldinfo_gamestate_half_tostring(half));

  __half = half;

  for (map<RemoteBlackBoard*,GameStateInterface*>::iterator it = __giss.begin(); it != __giss.end(); it++) {
    GameStateInterface* gis = it->second;
    switch (half) {
      case HALF_FIRST:
        gis->set_half(GameStateInterface::HALF_FIRST);
        break;
      case HALF_SECOND:
        gis->set_half(GameStateInterface::HALF_SECOND);
        break;
    }
  }
}


/** Send worldinfo. */
void
RefBoxStateBBWriter::send()
{
  ++__counter;
  log("Sending worldinfo\n");

  set<RemoteBlackBoard*> erase_rbbs;
  set<string> reconnect_hosts;

  unsigned int i = 0;
  for (map<RemoteBlackBoard*,GameStateInterface*>::iterator it = __giss.begin(); it != __giss.end(); it++) {
    RemoteBlackBoard* rbb = it->first;
    GameStateInterface* gis = it->second;
    const string host = __rbbs[rbb].c_str();
    try {
      gis->set_score_cyan(gis->score_cyan() + 1); // just for checking at the recipient's side whether the data ankommt
      gis->write();
      log("%u. Successfully wrote game state on %s\n", ++i, __rbbs[rbb].c_str());
    } catch (Exception& e) {
      log("%u. Writing game state on %s failed, reason:\n", ++i, __rbbs[rbb].c_str());
      e.print_trace();
      log("I will reconnect after this loop\n");
      erase_rbbs.insert(rbb);
      reconnect_hosts.insert(host);
    }
  }
  for (set<RemoteBlackBoard*>::iterator it = erase_rbbs.begin(); it != erase_rbbs.end(); it++) {
    RemoteBlackBoard* rbb = *it;
    __rbbs.erase(rbb);
    __giss.erase(rbb);
  }
  for (set<std::string>::iterator it = reconnect_hosts.begin(); it != reconnect_hosts.end(); it++) {
    std::string host = *it;
    log("Reconnecting to %s\n", host.c_str());
    connect(host);
  }

  log("Sending worldinfo done\n");
}
