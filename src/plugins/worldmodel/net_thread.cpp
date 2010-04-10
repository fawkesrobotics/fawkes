
/***************************************************************************
 *  net_thread.cpp - Fawkes WorldModel Plugin Network Thread
 *
 *  Created: Fri Jun 29 16:56:15 2007 (on flight to RoboCup 2007, Atlanta)
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

#include "net_thread.h"

#include <netcomm/worldinfo/transceiver.h>
#include <interfaces/ObjectPositionInterface.h>
#include <interfaces/GameStateInterface.h>

#include <string>
#include <cstring>
#include <cstdlib>
#include <cstdio>

using namespace std;
using namespace fawkes;

/** @class WorldModelNetworkThread "net_thread.h"
 * Network thread of worldmodel plugin.
 * @author Tim Niemueller
 */

/** Constructor.
 */
WorldModelNetworkThread::WorldModelNetworkThread()
  : Thread("WorldModelNetworkThread", Thread::OPMODE_CONTINUOUS)
{
  __worldinfo_transceiver = NULL;
  set_prepfin_conc_loop(true);
  __opponent_id = 0;
}


/** Destructor. */
WorldModelNetworkThread::~WorldModelNetworkThread()
{
}


void
WorldModelNetworkThread::init()
{
  std::string multicast_addr;
  unsigned int port;
  std::string encryption_key;
  std::string encryption_iv;
  bool        cfg_multicast_loopback;
  try {
    multicast_addr = config->get_string("/worldinfo/multicast_addr");
    port = config->get_uint("/worldinfo/udp_port");
    encryption_key = config->get_string("/worldinfo/encryption_key");
    encryption_iv  = config->get_string("/worldinfo/encryption_iv");
    __cfg_sleep_time_msec   = config->get_uint("/worldinfo/sleep_time_msec");
    __cfg_max_msgs_per_recv = config->get_uint("/worldinfo/max_msgs_per_recv");
    __cfg_flush_time_sec    = config->get_uint("/worldinfo/flush_time_sec");
    cfg_multicast_loopback  = config->get_bool("/worldinfo/multicast_loopback");
  } catch (Exception &e) {
    e.append("Could not get required configuration data for worldmodel");
    e.print_trace();
    throw;
  }

  __worldinfo_transceiver = new WorldInfoTransceiver(WorldInfoTransceiver::MULTICAST,
                                                     multicast_addr.c_str(), port,
						     encryption_key.c_str(), encryption_iv.c_str(),
						     nnresolver);

  __worldinfo_transceiver->add_handler(this);
  __worldinfo_transceiver->set_loop(cfg_multicast_loopback);

  try {
    __gamestate_if = blackboard->open_for_writing<GameStateInterface>("WI GameState");
  } catch (Exception &e) {
    delete __worldinfo_transceiver;
    e.print_trace();
    throw;
  }
}


void
WorldModelNetworkThread::finalize()
{
  // close all WI pose interfaces
  for (LockMap<string, ObjectPositionInterface*>::iterator i = __pose_ifs.begin();
       i != __pose_ifs.end();
       ++i) {
    blackboard->close(i->second);
  }

  // close all WI ball interfaces
  for (LockMap<string, ObjectPositionInterface*>::iterator i = __ball_ifs.begin();
       i != __ball_ifs.end();
       ++i) {
    blackboard->close(i->second);
  }

  // close all WI opponent interfaces
  for (LockMap<string, UidTimeObjPosMap>::iterator i = __opponent_ifs.begin();
       i != __opponent_ifs.end();
       ++i) {
    for (UidTimeObjPosMap::iterator j = i->second.begin();
	 j != i->second.end();
	 ++j) {
      blackboard->close(j->second.second);
    }
  }

  blackboard->close(__gamestate_if);
  delete __worldinfo_transceiver;
}


void
WorldModelNetworkThread::loop()
{
  __worldinfo_transceiver->flush_sequence_numbers(__cfg_flush_time_sec);
  __worldinfo_transceiver->recv(false, __cfg_max_msgs_per_recv);
  usleep( __cfg_sleep_time_msec * 1000 );
  // check for dead ones
  std::map<std::string, fawkes::Time>::iterator lsi = __last_seen.begin();
  Time now;
  __last_seen.lock();
  while (lsi != __last_seen.end()) {
    if (now - &lsi->second > 3.0) {
      logger->log_info("WorldModelNetworkThread", "Expiring host %s", lsi->first.c_str());
      // this is is as dead as the chair I'm sitting on
      __pose_ifs.lock();
      if (__pose_ifs.find(lsi->first) != __pose_ifs.end()) {
	blackboard->close(__pose_ifs[lsi->first]);
	__pose_ifs.erase(lsi->first);
      }
      __pose_ifs.unlock();
      __ball_ifs.lock();
      if (__ball_ifs.find(lsi->first) != __ball_ifs.end()) {
	blackboard->close(__ball_ifs[lsi->first]);
	__ball_ifs.erase(lsi->first);
      }
      __ball_ifs.unlock();
      __opponent_ifs.lock();
      if (__opponent_ifs.find(lsi->first) != __opponent_ifs.end()) {
	std::map<unsigned int, std::pair<Time, ObjectPositionInterface *> >::iterator i;
	for (i = __opponent_ifs[lsi->first].begin(); i != __opponent_ifs[lsi->first].end(); ++i) {
	  blackboard->close(i->second.second);
	}
	__opponent_ifs.erase(lsi->first);
      }
      __opponent_ifs.unlock();
      std::map<std::string, fawkes::Time>::iterator tmp = lsi;
      ++lsi;
      __last_seen.erase(tmp);
    } else {
      ++lsi;
    }
  }
  __last_seen.unlock();

  __opponent_ifs.lock();
  std::map<std::string, UidTimeObjPosMap>::iterator o = __opponent_ifs.begin();
  while (o != __opponent_ifs.end()) {
    UidTimeObjPosMap::iterator top = o->second.begin();
    while (top != o->second.end()) {
      if (now - &(top->second.first) > 3.0) {
	logger->log_info("WorldModelNetworkThread", "Expiring Opponent %s:%u", o->first.c_str(), top->first);
	blackboard->close(top->second.second);
	UidTimeObjPosMap::iterator tmp = top;
	++top;
	o->second.erase(tmp);
      } else {
	++top;
      }
    }
    if (o->second.empty()) {
      std::map<std::string, UidTimeObjPosMap>::iterator tmp = o;
      ++o;
      __opponent_ifs.erase(tmp);
    } else {
      ++o;
    }
  }
  __opponent_ifs.unlock();

}


/** Access the WI transceiver.
 * @return pointer to the WI transceiver
 */
WorldInfoTransceiver*
WorldModelNetworkThread::get_transceiver()
{
  return __worldinfo_transceiver;
}


void
WorldModelNetworkThread::pose_rcvd(const char *from_host,
				   float x, float y, float theta,
				   float *covariance)
{
  __pose_ifs.lock();
  if (__pose_ifs.find(from_host) == __pose_ifs.end()) {
    try {
      std::string id = std::string("WI RoboPos ") + from_host;
      __pose_ifs[from_host] = blackboard->open_for_writing<ObjectPositionInterface>(id.c_str());
    } catch (Exception &e) {
      logger->log_warn("WorldModelNetworkThread", "Failed to create ObjectPositionInterface "
		       "for pose of %s, exception follows", from_host);
      logger->log_warn("WorldModelNetworkThread", e);
      return;
    }
  }

  // Pose is our aliveness indicator
  __last_seen.lock();
  __last_seen[from_host].stamp();
  __last_seen.unlock();

  ObjectPositionInterface *iface = __pose_ifs[from_host];
  iface->set_world_x(x);
  iface->set_world_y(y);
  iface->set_world_z(theta);
  iface->set_world_xyz_covariance(covariance);
  iface->write();
  __pose_ifs.unlock();
}


void
WorldModelNetworkThread::velocity_rcvd(const char *from_host, float vel_x,
				       float vel_y, float vel_theta, float *covariance)
{
  // TODO
}


void
WorldModelNetworkThread::ball_pos_rcvd(const char *from_host,
				       bool visible, int visibility_history,
				       float dist, float bearing, float slope,
				       float *covariance)
{
  __ball_ifs.lock();
  if (__ball_ifs.find(from_host) == __ball_ifs.end()) {
    try {
      std::string id = std::string("WI BPos ") + from_host;
      __ball_ifs[from_host] = blackboard->open_for_writing<ObjectPositionInterface>(id.c_str());
    } catch (Exception &e) {
      logger->log_warn("WorldModelNetworkThread", "Failed to create ObjectPositionInterface "
		       "for ball pos of %s, exception follows", from_host);
      logger->log_warn("WorldModelNetworkThread", e);
      return;
    }
  }

  ObjectPositionInterface *iface = __ball_ifs[from_host];
  iface->set_flags( iface->flags() |
		    ObjectPositionInterface::TYPE_BALL |
		    ObjectPositionInterface::FLAG_HAS_RELATIVE_POLAR |
		    ObjectPositionInterface::FLAG_HAS_COVARIANCES );
  iface->set_visible(visible);
  iface->set_visibility_history(visibility_history);
  iface->set_distance(dist);
  iface->set_bearing(bearing);
  iface->set_slope(slope);
  iface->set_dbs_covariance(covariance);
  iface->write();
  __ball_ifs.unlock();
}


void
WorldModelNetworkThread::global_ball_pos_rcvd(const char *from_host,
					      bool visible, int visibility_history,
					      float x, float y, float z,
					      float *covariance)
{
  __ball_ifs.lock();
  if (__ball_ifs.find(from_host) == __ball_ifs.end()) {
    try {
      std::string id = std::string("WI BPos ") + from_host;
      __ball_ifs[from_host] = blackboard->open_for_writing<ObjectPositionInterface>(id.c_str());
    } catch (Exception &e) {
      logger->log_warn("WorldModelNetworkThread", "Failed to create ObjectPositionInterface "
		       "for ball pos of %s, exception follows", from_host);
      logger->log_warn("WorldModelNetworkThread", e);
      return;
    }
  }

  ObjectPositionInterface *iface = __ball_ifs[from_host];
  iface->set_flags( iface->flags() | 
		    ObjectPositionInterface::TYPE_BALL |
		    ObjectPositionInterface::FLAG_HAS_WORLD |
		    ObjectPositionInterface::FLAG_HAS_Z_AS_ORI |
		    ObjectPositionInterface::FLAG_HAS_COVARIANCES );
  iface->set_visible(visible);
  iface->set_visibility_history(visibility_history);
  iface->set_world_x(x);
  iface->set_world_y(y);
  iface->set_world_z(z);
  iface->set_world_xyz_covariance(covariance);
  iface->write();
  __ball_ifs.unlock();
}


void
WorldModelNetworkThread::ball_velocity_rcvd(const char *from_host,
					    float vel_x, float vel_y, float vel_z,
					    float *covariance)
{
  // TODO
}


void
WorldModelNetworkThread::global_ball_velocity_rcvd(const char *from_host,
						   float vel_x, float vel_y, float vel_z,
						   float *covariance)
{
  // TODO
}


void
WorldModelNetworkThread::opponent_pose_rcvd(const char *from_host,
					    unsigned int uid,
					    float distance, float bearing,
					    float *covariance)
{
  __opponent_ifs.lock();
  std::map<std::string, std::map<unsigned int, std::pair<Time, ObjectPositionInterface *> > >::iterator f;

  bool iface_exists = true;
  if ( ((f = __opponent_ifs.find(from_host)) == __opponent_ifs.end()) ||
       (f->second.find(uid) == f->second.end()) ) {

    char *tmp;
    if (asprintf(&tmp, "WI Opp %u %s", ++__opponent_id, from_host) != -1) {
      try {
	std::string id = tmp;
	free(tmp);
	logger->log_debug("WorldModelNetworkThread", "Opening new interface for %s:%u", from_host, uid);
	__opponent_ifs[from_host][uid] = make_pair(Time(), blackboard->open_for_writing<ObjectPositionInterface>(id.c_str()));
      } catch (Exception &e) {
	logger->log_warn("WorldModelNetworkThread", "Failed to create ObjectPositionInterface "
			 "for opponent %s:%u, exception follows", from_host, uid);
	logger->log_warn("WorldModelNetworkThread", e);
        iface_exists = false;
      }
    } else {
      logger->log_error("WorldModelNetworkThread", "Could not create interface ID string, out of memory during asprintf().");
      iface_exists = false;
    }
  }

  if (iface_exists) {
    logger->log_debug("WorldModelNetworkThread", "Setting opponent %s:%u", from_host, uid);
    ObjectPositionInterface *iface = __opponent_ifs[from_host][uid].second;
    iface->set_distance(distance);
    iface->set_bearing(bearing);
    iface->set_dbs_covariance(covariance);
    iface->write();

    __opponent_ifs[from_host][uid].first.stamp();
  } else {
    logger->log_warn("WorldModelNetworkThread", "Opponent pose interface does not exist, ignoring");
  }
  __opponent_ifs.unlock();
}


void
WorldModelNetworkThread::opponent_disapp_rcvd(const char *from_host, unsigned int uid)
{
  __opponent_ifs.lock();
  std::map<std::string, std::map<unsigned int, std::pair<Time, ObjectPositionInterface *> > >::iterator f;
  if ( ((f = __opponent_ifs.find(from_host)) != __opponent_ifs.end()) &&
       (f->second.find(uid) != f->second.end()) ) {
    blackboard->close(f->second[uid].second);
    f->second.erase(uid);
  }
  __opponent_ifs.unlock();
}


void
WorldModelNetworkThread::gamestate_rcvd(const char *from_host,
					unsigned int game_state,
					fawkes::worldinfo_gamestate_team_t state_team,
					unsigned int score_cyan, unsigned int score_magenta,
					fawkes::worldinfo_gamestate_team_t our_team,
					fawkes::worldinfo_gamestate_goalcolor_t our_goal_color,
					fawkes::worldinfo_gamestate_half_t half)
{
  logger->log_debug("WorldModelNetworkThread", "Received Gamestate %i from %s, state team %i, score %u:%u, our team: %i, our goal: %i, half: %i",
                    game_state, from_host, state_team, score_magenta, our_team, our_goal_color, half);
  switch (game_state) {
    case GS_FROZEN:
      __gamestate_if->set_game_state(GameStateInterface::GS_FROZEN);       break;
    case GS_PLAY:
      __gamestate_if->set_game_state(GameStateInterface::GS_PLAY);         break;
    case GS_KICK_OFF:
      __gamestate_if->set_game_state(GameStateInterface::GS_KICK_OFF);     break;
    case GS_DROP_BALL:
      __gamestate_if->set_game_state(GameStateInterface::GS_DROP_BALL);    break;
    case GS_PENALTY:
      __gamestate_if->set_game_state(GameStateInterface::GS_PENALTY);      break;
    case GS_CORNER_KICK:
      __gamestate_if->set_game_state(GameStateInterface::GS_CORNER_KICK);  break;
    case GS_THROW_IN:
      __gamestate_if->set_game_state(GameStateInterface::GS_THROW_IN);     break;
    case GS_FREE_KICK:
      __gamestate_if->set_game_state(GameStateInterface::GS_FREE_KICK);    break;
    case GS_GOAL_KICK:
      __gamestate_if->set_game_state(GameStateInterface::GS_GOAL_KICK);    break;
    case GS_HALF_TIME:
      __gamestate_if->set_game_state(GameStateInterface::GS_HALF_TIME);    break;
  }

  switch (state_team) {
    case TEAM_NONE:
      __gamestate_if->set_state_team(GameStateInterface::TEAM_NONE);       break;
    case TEAM_CYAN:
      __gamestate_if->set_state_team(GameStateInterface::TEAM_CYAN);       break;
    case TEAM_MAGENTA:
      __gamestate_if->set_state_team(GameStateInterface::TEAM_MAGENTA);    break;
    case TEAM_BOTH:
      __gamestate_if->set_state_team(GameStateInterface::TEAM_BOTH);       break;
  }

  switch (our_team) {
    case TEAM_NONE:
      __gamestate_if->set_our_team(GameStateInterface::TEAM_NONE);         break;
    case TEAM_CYAN:
      __gamestate_if->set_our_team(GameStateInterface::TEAM_CYAN);         break;
    case TEAM_MAGENTA:
      __gamestate_if->set_our_team(GameStateInterface::TEAM_MAGENTA);      break;
    case TEAM_BOTH:
      __gamestate_if->set_our_team(GameStateInterface::TEAM_BOTH);         break;
  }

  switch (our_goal_color) {
    case GOAL_BLUE:
      __gamestate_if->set_our_goal_color(GameStateInterface::GOAL_BLUE);   break;
    case GOAL_YELLOW:
      __gamestate_if->set_our_goal_color(GameStateInterface::GOAL_YELLOW); break;
  }

  switch (half) {
    case HALF_FIRST:
      __gamestate_if->set_half(GameStateInterface::HALF_FIRST);            break;
    case HALF_SECOND:
      __gamestate_if->set_half(GameStateInterface::HALF_SECOND);           break;
  }

  __gamestate_if->set_score_cyan(score_cyan);
  __gamestate_if->set_score_magenta(score_magenta);

  __gamestate_if->write();
}


void
WorldModelNetworkThread::penalty_rcvd(const char *from_host,
				      unsigned int player, unsigned int penalty,
				      unsigned int seconds_remaining)
{
  // TBD
}

