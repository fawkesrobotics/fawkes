
/***************************************************************************
 *  data_container.h - World info data container
 *
 *  Created: Thu April 10 16:16:17 2008
 *  Copyright  2008  Daniel Beck
 *
 *  $Id$
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

#ifndef __TOOLS_WORLDINFO_VIEWER_DATA_CONTAINER_H_
#define __TOOLS_WORLDINFO_VIEWER_DATA_CONTAINER_H_

#include <core/utils/lock_map.h>
#include <geometry/hom_vector.h>
#include <geometry/hom_point.h>
#include <geometry/hom_pose.h>
#include <netcomm/worldinfo/enums.h>
#include <string>
#include <vector>
#include <map>

class Clock;

class WorldInfoDataContainer
{
 public:
  WorldInfoDataContainer(Clock* clock);
  ~WorldInfoDataContainer();

  /** Container struct for momentary game state infos. */
  struct GameState
  {
    worldinfo_gamestate_t game_state;       /**< current game state */
    worldinfo_gamestate_team_t state_team;  /**< team association of the game state */
    unsigned int score_cyan;                /**< socre of the cyan-colored team */
    unsigned int score_magenta;             /**< score of the magenta-colored team */ 
    worldinfo_gamestate_half_t half;        /**< first or second half */
  };

  /** Map that assigns positions to ids. */
  typedef std::map<unsigned int, HomVector> PosMap;

  void reset();

  std::vector<std::string> get_hosts();

  void set_robot_pose( const char* from_host, float x, float y, float theta,
		       float* covariance );
  bool delete_robot_pose(const char* from_host);
  bool get_robot_pose(const char* host, HomPose& robot_pose);

  void set_ball_pos_relative( const char* from_host, float dist, 
			      float bearing, float slope, float* covariance );
  bool get_ball_pos_relative(const char* host, HomVector& ball_pos);
  bool get_ball_pos_global(const char* host, HomPoint& ball_pos);
  bool delete_ball_pos(const char* from_host);

  void set_opponent_pos( const char* from_host, unsigned int uid,
			 float distance, float angle, float* covariance );
  bool get_opponent_pos(const char* host, PosMap& opponent_pos);

  void set_game_state( worldinfo_gamestate_t game_state,
		       worldinfo_gamestate_team_t state_team,
		       unsigned int score_cyan,
		       unsigned int score_magenta,
		       worldinfo_gamestate_team_t own_team,
		       worldinfo_gamestate_goalcolor_t own_goal_color,
		       worldinfo_gamestate_half_t half );

  GameState get_game_state() const;
  std::string get_game_state_string() const;
  std::string get_half_string() const;
  unsigned int get_own_score() const;
  unsigned int get_other_score() const;

  worldinfo_gamestate_team_t get_own_team_color() const;
  std::string get_own_team_color_string() const;
  worldinfo_gamestate_goalcolor_t get_own_goal_color() const;
  std::string get_own_goal_color_string() const;
  
 private:
  typedef LockMap<std::string, unsigned int> HostLockMap;
  typedef LockMap<unsigned int, HomVector> RelPosLockMap;
  typedef LockMap<unsigned int, HomPoint> GlobPosLockMap;
  typedef LockMap<unsigned int, HomPose> PoseLockMap;
    
  HostLockMap m_hosts;
  LockMap<unsigned int, long> m_last_seen;
  PoseLockMap m_robot_pose;
  RelPosLockMap m_ball_pos_rel;
  GlobPosLockMap m_ball_pos_global;
  LockMap<unsigned int, PosMap> m_opponent_pos;

  unsigned int m_host_id;

  GameState m_game_state;
  worldinfo_gamestate_team_t m_own_team_color;
  worldinfo_gamestate_goalcolor_t m_own_goal_color;

  Clock* m_clock;
};

#endif /* __TOOLS_WORLDINFO_VIEWER_DATA_CONTAINER_H_ */ 
