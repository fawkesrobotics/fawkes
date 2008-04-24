
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

#include <worldinfo_utils/data_container.h>
#include <utils/time/clock.h>
#include <utils/time/time.h>
#include <cmath>

using namespace std;

/** @class WorldInfoDataContainer <worldinfo_utils/data_container.h>
 * Data container to store and exchange worldinfo data.
 * @author Daniel Beck
 */

/** Constructor.
 * @param clock pointer to a Clock
 */
WorldInfoDataContainer::WorldInfoDataContainer(Clock* clock)
{
  reset();
  m_clock = clock;

  m_own_team_color = TEAM_CYAN;
  m_own_goal_color = GOAL_BLUE;

  m_game_state.game_state    = GS_FROZEN;
  m_game_state.state_team    = TEAM_BOTH;
  m_game_state.score_cyan    = 0;
  m_game_state.score_magenta = 0;
  m_game_state.half          = HALF_FIRST;
}

/** Destructor. */
WorldInfoDataContainer::~WorldInfoDataContainer()
{
}


/** Delete all stored information. */
void
WorldInfoDataContainer::reset()
{
  m_hosts.lock();
  m_hosts.clear();
  m_hosts.unlock();

  m_last_seen.lock();
  m_last_seen.clear();
  m_last_seen.unlock();

  m_ball_pos.lock();
  m_ball_pos.clear();
  m_ball_pos.unlock();

  m_ball_pos_global.lock();
  m_ball_pos_global.clear();
  m_ball_pos_global.unlock();

  m_opponent_pos.lock();
  m_opponent_pos.clear();
  m_opponent_pos.unlock();

  m_host_id = 0;

  m_host_added   = false;
  m_host_removed = false;
}

/** Get the names of all registered hosts.
 * @return vector containing the hostnames of all registered hosts
 */
vector<string>
WorldInfoDataContainer::get_hosts()
{
  vector<string> hosts;
  Time now(m_clock);
  now.stamp();

  m_hosts.lock();
  HostLockMap::iterator iter = m_hosts.begin();
  while ( iter != m_hosts.end() )
    {
      if ( now.in_msec() - m_last_seen[ iter->second ] < 3000 )
	{ hosts.push_back( iter->first ); ++iter; }
      else
	{
	  unsigned int id = iter->second;
	  m_last_seen.lock();
	  m_last_seen.erase(id);
	  m_last_seen.unlock();

	  m_robot_pose.lock();
	  m_robot_pose.erase(id);
	  m_robot_pose.unlock();

	  m_ball_pos.lock();
	  m_ball_pos.erase(id);
	  m_ball_pos.unlock();

	  m_ball_pos_global.lock();
	  m_ball_pos_global.erase(id);
	  m_ball_pos_global.unlock();

	  m_hosts.erase(iter++);

	  m_host_removed = true;
	}
    }
  m_hosts.unlock();

  return hosts;
}

/** Check whether a host was added since the last time checked.
 * @return true if a host was added
 */
bool
WorldInfoDataContainer::host_added()
{
  if (m_host_added)
    {
      m_host_added = false;
      return true;
    }

  return false;
}

/** Check whether a host was removed since the last time checked.
 * @return true if a host was removed
 */
bool
WorldInfoDataContainer::host_removed()
{
  if (m_host_removed)
    {
      m_host_removed = false;
      return true;
    }

  return false;
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
  unsigned int id = get_host_id(host_string);

  clock_in_host(id);

  HomPose p(x, y, theta);
  m_robot_pose.lock();
  m_robot_pose[id] = p;
  m_robot_pose.unlock();

  m_robot_pose_cov.lock();
  m_robot_pose_cov[id] = Matrix( 3, 3, covariance );
  m_robot_pose_cov.unlock();
}


/** Delete a stored robot pose.
 * @param host hostname of the robot whose pose shall be deleted
 */
bool
WorldInfoDataContainer::delete_robot_pose(const char* host)
{
  string host_string(host);
  m_hosts.lock();
  HostLockMap::iterator iter = m_hosts.find(host_string);
  if ( iter == m_hosts.end() )
    {
      m_hosts.unlock();
      return false;
    }

  m_hosts.erase(iter);
  m_hosts.unlock();
  m_host_removed = true;
  return true;
}


/** Get the position of a certain robot.
 * @param host the hostname of the robot
 * @param robot_pose reference to a HomPoint where the global position of the robot is
 *        written to
 * @param robot_pose_cov reference to a Matrix where the covariance of the robot position
          is written to
 * @return true if a pose for the robot could be found
 */
bool
WorldInfoDataContainer::get_robot_pose(const char* host, HomPose& robot_pose, Matrix &robot_pose_cov)
{
  HostLockMap::iterator hit;
  PoseLockMap::iterator pit;
  m_hosts.lock();
  hit = m_hosts.find( string(host) );
  if ( hit == m_hosts.end() )
    // no id for given robot
    {
      m_hosts.unlock();
      return false;
    }
  m_hosts.unlock();

  m_robot_pose.lock();
  pit = m_robot_pose.find( hit->second );
  if ( pit == m_robot_pose.end() )
    // no pose for given robot
    {
      m_robot_pose.unlock();
      return false;
    }

  robot_pose = pit->second;
  m_robot_pose.unlock();

  m_robot_pose_cov.lock();
  MatrixLockMap::const_iterator it = m_robot_pose_cov.find( hit->second );
  if ( it == m_robot_pose_cov.end() )
    {
      m_robot_pose_cov.unlock();
      return false;
    }
  robot_pose_cov = it->second;
  m_robot_pose_cov.unlock();

  return true;
}

/** Set the velocity of the robot.
 * @param host the hostname of the robot
 * @param vel_x the current forward velocity of the robot
 * @param vel_y the current sideward velocity of the robot
 * @param vel_theta the current rotational velociy of the robot
 * @param covariance the velocity covariance
 */
void
WorldInfoDataContainer::set_robot_velocity( const char* host, 
					    float vel_x, float vel_y, float vel_theta,
					    float* covariance )
{
  string host_string = string(host);
  unsigned int id;

  id = get_host_id(host_string);

  clock_in_host(id);
  // TODO
}

/** Remove velocity information for the specified robot.
 * @param host hostname of the robot
 */
void
WorldInfoDataContainer::delete_robot_velocity(const char* host)
{
  // TODO
}

/** Obtain current velocity of the specified robot.
 * @param host the hostname of the robot
 * @param robot_vel reference to a HomVector where the velocity information is written to
 * @return true, if velocity information for the specified host are available
 */
bool
WorldInfoDataContainer::get_robot_velocity(const char* host, HomVector& robot_vel)
{
  // TODO
  return true;
}

/** Set the ball position estimation of a robot.
 * @param host the hostname of the robot
 * @param dist distance to the robot
 * @param bearing vertical angle to the ball
 * @param slope the horizontal angle to the ball
 * @param covariance covariance associated with the position estimation
 */
void
WorldInfoDataContainer::set_ball_pos( const char* host, float dist,
				      float bearing, float slope, float* covariance )
{
  string host_string(host);
  unsigned int id = get_host_id(host_string);

  clock_in_host(id);

  // compute global ball position in case a global pose for the robot is known
  HomPolar pos(dist, bearing, slope);
  m_ball_pos.lock();
  m_ball_pos[id] = pos;
  m_ball_pos.unlock();

  m_ball_pos_rel_cov.lock();
  m_ball_pos_rel_cov[id] = Matrix( 3, 3, covariance );
  m_ball_pos_rel_cov.unlock();

  HomPose robot_pose;
  Matrix robot_pose_cov( 3, 3 );
  if ( get_robot_pose(host, robot_pose, robot_pose_cov) )
    {
      HomPoint glob_pos;
      glob_pos = robot_pose.pos() + pos.rotate_z( robot_pose.yaw() );
      m_ball_pos_global.lock();
      m_ball_pos_global[id] = glob_pos;
      m_ball_pos_global.unlock();
    }
}


/** Get the ball position estimation of a certain robot.
 * @param host the hostname of the robot
 * @param ball_pos reference to a HomPolar where the position is written to
 * @param ball_pos_cov reference to a Matrix where the ball position covariance is written to
 * @return true if a global ball position was found
 */
bool
WorldInfoDataContainer::get_ball_pos( const char* host, HomPolar& ball_pos, 
				      Matrix &ball_pos_cov )
{
  HostLockMap::iterator hit;
  PolarLockMap::iterator bit;
  m_hosts.lock();
  hit = m_hosts.find( string(host) );
  if ( hit == m_hosts.end() )
    // no id for given robot
    {
      m_hosts.unlock();
      return false;
    }
  m_hosts.unlock();

  m_ball_pos.lock();
  bit = m_ball_pos.find(hit->second);
  if ( bit == m_ball_pos.end() )
    // no relative ball position for given robot
    {
      m_ball_pos.unlock();
      return false;
    }

  ball_pos = bit->second;
  m_ball_pos.unlock();

  m_ball_pos_rel_cov.lock();
  MatrixLockMap::const_iterator it = m_ball_pos_rel_cov.find( hit->second );
  if ( it == m_ball_pos_rel_cov.end() )
    {
      m_ball_pos_rel_cov.unlock();
      return false;
    }
  ball_pos_cov = it->second;
  m_ball_pos_rel_cov.unlock();

  return true;
}


/** Get the global position of the ball as it is estimated by the specified robot.
 * @param host the robot's hostname
 * @param ball_pos refercence to a HomPoint where the position of the ball written to
 */
bool
WorldInfoDataContainer::get_ball_pos_global(const char* host, HomPoint& ball_pos)
{
  HostLockMap::iterator hit;
  PointLockMap::iterator bit;
  m_hosts.lock();
  hit = m_hosts.find( string(host) );
  if ( hit == m_hosts.end() )
    // no id for given robot
    {
      m_hosts.unlock();
      return false;
    }
  m_hosts.unlock();

  m_ball_pos_global.lock();
  bit = m_ball_pos_global.find(hit->second);
  if ( bit == m_ball_pos_global.end() )
    // no global ball position for given robot
    {
      m_ball_pos_global.unlock();
      return false;
    }

  ball_pos = bit->second;
  m_ball_pos_global.unlock();

  return true;
}

/** Delete a certain ball position.
 * @param host hostname of the robot whose ball estimation shall be deleted
 */
bool
WorldInfoDataContainer::delete_ball_pos(const char* host)
{
  string host_string(host);
  HostLockMap::iterator iter;

  m_hosts.lock();
  iter = m_hosts.find(host_string);
  if ( iter == m_hosts.end() )
    {
      m_hosts.unlock();
      return false;
    }

  m_ball_pos.lock();
  m_ball_pos.erase( iter->second );
  m_ball_pos.unlock();
  m_hosts.unlock();
  m_host_removed = true;

  return true;
}

/** Set the ball velocity as it is estimated by the specified robot.
 * @param host the hostname of the robot
 * @param vel_x the ball velocity in x-direction of the robot-centered coordinate system
 * @param vel_y the ball velocity in y-direction of the robot-centered coordinate system
 * @param vel_z the ball velocity in z-direction of the robot-centered coordinate system
 * @param covariance ball velocity covariance
 */
void
WorldInfoDataContainer::set_ball_velocity( const char* host, 
					   float vel_x, float vel_y, float vel_z, 
					   float* covariance )
{
  // TODO
}

/** Delete ball velocity information from specified robot.
 * @param host the hostname of the robot
 */
void
WorldInfoDataContainer::delete_ball_velocity(const char* host)
{
  // TODO
}

/** Obtain ball velocity information for specified robot.
 * @param host the hostname of the robot
 * @param ball_vel refrence to a HomVector where the velocity information is written to
 * @return true if ball velocity information from the specified robot are available
 */
bool
WorldInfoDataContainer::get_ball_velocity(const char* host, HomVector& ball_vel)
{
  // TODO
  return true;
}

/** Set the position of a detected opponent.
 * @param host hostname of the robot that detected the robot
 * @param uid opponent id
 * @param distance distance to the robot
 * @param angle angle at which the opponent is detected
 * @param covariance corresponding covariance matrix
 */
void
WorldInfoDataContainer::set_opponent_pos( const char* host, unsigned int uid,
					  float distance, float angle, float* covariance )
{
  string host_string(host);
  unsigned int id = get_host_id(host_string);

  clock_in_host(id);

  HomPolar pos(distance, angle);

  LockMap<unsigned int, PosMap>::iterator iter;
  m_opponent_pos.lock();
  iter = m_opponent_pos.find(id);
  if ( iter == m_opponent_pos.end() )
    {
      PosMap opponents;
      opponents[uid] = pos;
      m_opponent_pos[id] = opponents;
    }
  else
    {
      PosMap& opponents = iter->second;
      opponents[uid] = pos;
    }
  m_opponent_pos.unlock();

  // TODO: covariance
}

/** Delete opponent position information for specified robot.
 * @param host hostname of the robot
 * @param uid the id of the opponent
 */
void
WorldInfoDataContainer::delete_opponent_pos(const char* host, unsigned int uid)
{
  string host_string(host);
  unsigned int id = get_host_id(host_string);

  LockMap<unsigned int, PosMap>::iterator iter;
  m_opponent_pos.lock();
  iter = m_opponent_pos.find(id);
  if ( iter != m_opponent_pos.end() )
    {
      PosMap& obstacles = iter->second;
      PosMap::iterator opit;
      opit = obstacles.find(uid);
      if ( opit != obstacles.end() )
	{
	  obstacles.erase(opit);
	}
    }
  m_opponent_pos.unlock();  
}

/** Delete all opponents seen by the specified robot.
 * @param host the hostname of the robot
 */
void
WorldInfoDataContainer::delete_all_opponent_pos(const char* host)
{
  string host_string(host);
  unsigned int id = get_host_id(host_string);

  LockMap<unsigned int, PosMap>::iterator iter;
  m_opponent_pos.lock();
  iter = m_opponent_pos.find(id);
  if ( iter != m_opponent_pos.end() )
    {
      m_opponent_pos.erase(iter);
    }
  m_opponent_pos.unlock();
}

/** Get all oppenents detected by a certain robot.
 * @param host hostname of the robot
 * @param opp_positions map containing the positions of the detected opponents
 */
bool
WorldInfoDataContainer::get_opponent_pos( const char* host,
					  WorldInfoDataContainer::PosMap& opp_positions )
{
  string host_string(host);
  unsigned int id = get_host_id(host_string);

  LockMap<unsigned int, PosMap>::iterator iter;
  m_opponent_pos.lock();
  iter = m_opponent_pos.find(id);
  if ( iter == m_opponent_pos.end() )
    {
      m_opponent_pos.unlock();
      return false;
    }

  m_opponent_pos.unlock();
  opp_positions = iter->second;
  return true;
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

unsigned int
WorldInfoDataContainer::get_host_id(std::string host)
{
  unsigned int id;

  m_hosts.lock();
  HostLockMap::iterator iter = m_hosts.find(host);
  if ( iter == m_hosts.end() )
    {
      id            = m_host_id++;
      m_hosts[host] = id;
      m_host_added  = true;
    }
  else
    { 
      id = iter->second; 
    }
  m_hosts.unlock();

  return id;
}

void
WorldInfoDataContainer::clock_in_host(unsigned int id)
{
  Time now(m_clock);
  now.stamp();
  
  m_last_seen.lock();
  m_last_seen[id] = now.in_msec();
  m_last_seen.unlock();
}
