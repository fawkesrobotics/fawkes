
/***************************************************************************
 *  data_container.cpp - World info data container
 *
 *  Created: Thu April 10 22:23:27 2008
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

#include <tools/worldinfo_viewer/data_container.h>
#include <cmath>

using namespace std;

/** @class WorldInfoDataContainer <tools/worldinfo_viewer/data_container.h>
 * Central data container of the worldinfo viewer application.
 * @author Daniel Beck
 */

/** Constructor. */
WorldInfoDataContainer::WorldInfoDataContainer()
{
  reset();
}

/** Destructor. */
WorldInfoDataContainer::~WorldInfoDataContainer()
{
}

/** Delete all stored information. */
void
WorldInfoDataContainer::reset()
{
  m_hosts.clear();
  m_ball_pos.clear();

  m_host_id = 0;
}

/** Get the names of all registered hosts.
 * @return vector containing the hostnames of all registered hosts
 */
vector<string>
WorldInfoDataContainer::get_hosts()
{
  vector<string> hosts;
  
  m_hosts.lock();
  HostLockMap::iterator iter;
  for (iter = m_hosts.begin(); iter != m_hosts.end(); ++iter)
    {
      hosts.push_back( iter->first );
    }
  m_hosts.unlock();
  
  return hosts;
}

/** Set the pose of a robot.
 * @param host the hostname of the robot
 * @param x the x-coordinate of the robot's global position
 * @param y the y-coordinate of the robot's global position
 * @param theta the global orientation of the robot
 * @param covariance covariance associated with the position estimation
 */
void
WorldInfoDataContainer::set_robot_pose( const char* host, float x, float y, float theta,
					float* covariance )
{
  string host_string(host);
  unsigned int id;

  m_hosts.lock();
  HostLockMap::iterator iter = m_hosts.find(host_string);
  if ( iter == m_hosts.end() )
    {
      id =  m_host_id++;
      m_hosts[ string(host) ] = id;
    }
  else
    { 
      id = iter->second; 
    }
  m_hosts.unlock();

  HomVector v(x, y);
  m_robot_pos.lock();
  m_robot_pos[id] = v;
  m_robot_pos.unlock();
}

/** Get the position of a certain robot.
 * @param host the hostname of the robot
 * @return global position of the robot
 */
HomVector
WorldInfoDataContainer::get_robot_pos(const char* host)
{
  HomVector pos;
  unsigned int id;

  m_hosts.lock();
  id = m_hosts[ string(host) ];
  m_hosts.unlock();

  m_robot_pos.lock();
  pos = m_robot_pos[id];
  m_robot_pos.unlock();

  return pos;
}

/** Set the ball position estimation of a robot.
 * @param host the hostname of the robot
 * @param dist distance to the robot
 * @param pitch vertical angle to the ball
 * @param yaw the horizontal angle to the ball
 * @param covariance covariance associated with the position estimation
 */
void
WorldInfoDataContainer::set_ball_pos( const char* host, float dist, 
				      float pitch, float yaw, float* covariance )
{
  string host_string(host);
  unsigned int id;

  m_hosts.lock();
  HostLockMap::iterator iter = m_hosts.find(host_string);
  if ( iter == m_hosts.end() )
    {
      id =  m_host_id++;
      m_hosts[ string(host) ] = id;
    }
  else
    { 
      id = iter->second; 
    }
  m_hosts.unlock();

  float x = dist * sin(yaw);;
  float y = dist * sin(pitch);;
  float z = sqrt(dist * dist - y * y - z * z);;
  
  HomVector v(x, y, z);
  m_ball_pos.lock();
  m_ball_pos[id] = v;
  m_ball_pos.unlock();
}

/** Get the ball position estimation of a certain robot.
 * @param host the hostname of the robot
 * @return relative position of the ball wrt. the robot
 */
HomVector
WorldInfoDataContainer::get_ball_pos(const char* host)
{
  HomVector pos;
  unsigned int id;

  m_hosts.lock();
  id = m_hosts[ string(host) ];
  m_hosts.unlock();

  m_ball_pos.lock();
  pos = m_ball_pos[id];
  m_ball_pos.unlock();

  return pos;
}

/** Set the gamestate.
 * @param game_state the current game state
 * @param state_team team association of the game state
 * @param score_cyan score of the cyan-colored team
 * @param score_magenta score of the magenta-colored team
 * @param own_team own team color
 * @param own_goal_color own goal color
 * @param half first or second half
 */
void
WorldInfoDataContainer::set_game_state( worldinfo_gamestate_t game_state,
					worldinfo_gamestate_team_t state_team,
					unsigned int score_cyan,
					unsigned int score_magenta,
					worldinfo_gamestate_team_t own_team,
					worldinfo_gamestate_goalcolor_t own_goal_color,
					worldinfo_gamestate_half_t half )
{
  m_game_state.game_state    = game_state;
  m_game_state.state_team    = state_team;
  m_game_state.score_cyan    = score_cyan;
  m_game_state.score_magenta = score_magenta;
  m_game_state.half          = half;
  
  m_own_team_color = own_team;
  m_own_goal_color = own_goal_color;
}

/** Obtain the game state.
 * @return the current game state
 */
WorldInfoDataContainer::GameState
WorldInfoDataContainer::get_game_state() const
{
  return m_game_state;
}

/** Get the current game state as string.
 * @return the current game mode
 */
std::string
WorldInfoDataContainer::get_game_state_string() const
{
  const char* game_state = worldinfo_gamestate_tostring(m_game_state.game_state);

  return string(game_state);
}

/** Get the current half as string.
 * @return the current half
 */
std::string
WorldInfoDataContainer::get_half_string() const
{
  const char* half = worldinfo_gamestate_half_tostring(m_game_state.half);

  return string(half);
}

/** Get own score.
 * @return own score
 */
unsigned int
WorldInfoDataContainer::get_own_score() const
{
  if (m_own_team_color == TEAM_CYAN)
    { return m_game_state.score_cyan; }
  else
    { return m_game_state.score_magenta; }
}

/** Get score of the other team.
 * @return the other team's score
 */
unsigned int
WorldInfoDataContainer::get_other_score() const
{
  if (m_own_team_color == TEAM_CYAN)
    { return m_game_state.score_magenta; }
  else
    { return m_game_state.score_cyan; }
}

/** Get own team color.
 * @return struct containing the own team color
 */
worldinfo_gamestate_team_t
WorldInfoDataContainer::get_own_team_color() const
{
  return m_own_team_color;
}

/** Get own team color as string.
 * @return string with the own team color
 */
std::string
WorldInfoDataContainer::get_own_team_color_string() const
{
  const char* team_color = worldinfo_gamestate_team_tostring(m_own_team_color);
  
  return string(team_color);
}

/** Get own goal color.
 * @return struct containing the own goal color
 */
worldinfo_gamestate_goalcolor_t
WorldInfoDataContainer::get_own_goal_color() const
{
  return m_own_goal_color;
}

/** Get own goal color as string.
 * @return string with the current goal color
 */
std::string
WorldInfoDataContainer::get_own_goal_color_string() const
{
  const char* goal_color = worldinfo_gamestate_goalcolor_tostring(m_own_goal_color);
  
  return string(goal_color);
}
