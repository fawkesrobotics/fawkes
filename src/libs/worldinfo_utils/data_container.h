
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

#include <geometry/matrix.h>
#include <geometry/hom_point.h>
#include <geometry/hom_polar.h>
#include <geometry/hom_pose.h>
#include <core/utils/lock_map.h>
#include <netcomm/worldinfo/enums.h>

#include <string>
#include <vector>
#include <map>

namespace fawkes {

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

  /** Map that assigns positions (in polar coordinates) to ids. */
  typedef std::map<unsigned int, HomPolar> PosMap;

  void reset();

  std::vector<std::string> get_hosts();
  bool host_added();
  bool host_removed();

  // (own) pose
  void set_robot_pose( const char* from_host, float x, float y, float theta,
		       float* covariance );
  bool delete_robot_pose(const char* from_host);
  bool get_robot_pose(const char* host, HomPose& robot_pose, Matrix &robot_pose_cov);

  // (own) velocity
  void set_robot_velocity( const char* from_host, float vel_x, float vel_y, float vel_theta,
			   float* covariance );
  void delete_robot_velocity(const char* from_host);
  bool get_robot_velocity(const char* host, HomVector& robot_vel);

  // ball position
  void set_ball_pos( const char* from_host, float dist,
			      float bearing, float slope, float* covariance );
  bool get_ball_pos(const char* host, HomPolar& ball_pos, Matrix &ball_pos_cov);
  bool get_ball_pos_global(const char* host, HomPoint& ball_pos);
  bool delete_ball_pos(const char* from_host);

  // ball velocity
  void set_ball_velocity( const char* from_host, float vel_x, float vel_y, float vel_z,
			  float* covariance );
  void delete_ball_velocity(const char* from_host);
  bool get_ball_velocity(const char* from_host, HomVector& ball_vel);

  // opponents
  void set_opponent_pos( const char* from_host, unsigned int uid,
			 float distance, float angle, float* covariance );
  void delete_opponent_pos(const char* from_host, unsigned int uid);
  void delete_all_opponent_pos(const char* from_host);
  bool get_opponent_pos(const char* host, PosMap& opponent_pos);

  // game state
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
  unsigned int get_host_id(std::string host);
  void clock_in_host(unsigned int id);

  typedef LockMap<std::string, unsigned int> HostLockMap;
  typedef LockMap<unsigned int, HomPolar>    PolarLockMap;
  typedef LockMap<unsigned int, HomPoint>    PointLockMap;
  typedef LockMap<unsigned int, HomPose>     PoseLockMap;
  typedef LockMap<unsigned int, Matrix>      MatrixLockMap;

  HostLockMap m_hosts;
  LockMap<unsigned int, long> m_last_seen;

  PoseLockMap    m_robot_pose;
  MatrixLockMap  m_robot_pose_cov;

  PoseLockMap    m_robot_vel;
  MatrixLockMap  m_robot_vel_cov;

  PolarLockMap   m_ball_pos;
  PointLockMap   m_ball_pos_global;
  MatrixLockMap  m_ball_pos_rel_cov;

  LockMap<unsigned int, PosMap> m_opponent_pos;

  unsigned int m_host_id;

  bool m_host_added;
  bool m_host_removed;

  GameState m_game_state;
  worldinfo_gamestate_team_t m_own_team_color;
  worldinfo_gamestate_goalcolor_t m_own_goal_color;

  Clock* m_clock;
};

} // end namespace fawkes

#endif /* __TOOLS_WORLDINFO_VIEWER_DATA_CONTAINER_H_ */
