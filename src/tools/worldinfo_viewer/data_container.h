
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
#include <netcomm/worldinfo/enums.h>
#include <string>
#include <vector>

class WorldInfoDataContainer
{
 public:
  WorldInfoDataContainer();
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

  void reset();

  std::vector<std::string> get_hosts();

  void set_robot_pose( const char* from_host, float x, float y, float theta,
		       float* covariance );

  void set_ball_pos( const char* from_host, float dist, 
		     float pitch, float yaw, float* covariance );

  HomVector get_ball_pos(const char* host);
  HomVector get_robot_pos(const char* host);

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
  typedef LockMap<unsigned int, HomVector> PosLockMap;
    
  HostLockMap m_hosts;
  PosLockMap m_robot_pos;
  PosLockMap m_ball_pos;

  unsigned int m_host_id;

  GameState m_game_state;
  worldinfo_gamestate_team_t m_own_team_color;
  worldinfo_gamestate_goalcolor_t m_own_goal_color;
};

#endif /* __TOOLS_WORLDINFO_VIEWER_DATA_CONTAINER_H_ */ 
