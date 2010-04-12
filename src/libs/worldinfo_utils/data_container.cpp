
/***************************************************************************
 *  data_container.cpp - World info data container
 *
 *  Created: Thu April 10 22:23:27 2008
 *  Copyright  2008  Daniel Beck
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
#include <core/exceptions/system.h>
#include <cstdlib>
#include <cstdio>
#include <cmath>

using namespace std;
namespace fawkes {

WorldInfoDataContainer::BallRecord::BallRecord()
{
  m_is_global = false;
}

WorldInfoDataContainer::BallRecord::~BallRecord()
{
}

void
WorldInfoDataContainer::BallRecord::set_pos( float dist,
					     float bearing,
					     float slope,
					     float* covariance )
{
  m_rel_pos.r(dist);
  m_rel_pos.phi(bearing);
  // TODO: slope, covariance
}

void
WorldInfoDataContainer::BallRecord::set_pos_global( float x,
						    float y,
						    float z,
						    float* covariance )
{
  m_is_global = true;
  m_glob_pos.x( x );
  m_glob_pos.y( y );
  m_glob_pos.z( z );
  // TODO: covarince
}

void
WorldInfoDataContainer::BallRecord::set_visible( bool visible,
						 int visibility_history )
{
  m_visible = visible;
  m_visibility_history = visibility_history;
}

void
WorldInfoDataContainer::BallRecord::set_velocity( float vel_x,
						  float vel_y,
						  float vel_z,
						  float* covariance )
{
  m_rel_vel.x(vel_x);
  m_rel_vel.y(vel_y);
  m_rel_vel.z(vel_z);
  // TODO: covariance
}

bool
WorldInfoDataContainer::BallRecord::visible() const
{
  return m_visible;
}

int
WorldInfoDataContainer::BallRecord::visibility_history() const
{
  return m_visibility_history;
}

HomPolar
WorldInfoDataContainer::BallRecord::pos_relative()
{
  return m_rel_pos;
}

HomVector
WorldInfoDataContainer::BallRecord::vel_relative()
{
  return m_rel_vel;
}

Matrix
WorldInfoDataContainer::BallRecord::covariance_relative()
{
  return m_rel_cov;
}

HomPoint
WorldInfoDataContainer::BallRecord::pos_global( float ref_x,
						float ref_y,
						float ref_theta )
{
  if ( !m_is_global )
  {
    HomPoint p( m_rel_pos.x(), m_rel_pos.y() );
    p.rotate_z( ref_theta );
    p.x() += ref_x;
    p.y() += ref_y;
    return p;
  }
  else
  {
    return m_glob_pos;
  }
}

HomVector
WorldInfoDataContainer::BallRecord::vel_global( float vel_x,
						float vel_y,
						float vel_theta,
						float ref_theta )
{
  // TODO
  return HomVector(0.0, 0.0, 0.0);
}

WorldInfoDataContainer::PoseRecord::PoseRecord()
{
}

WorldInfoDataContainer::PoseRecord::~PoseRecord()
{
}

void
WorldInfoDataContainer::PoseRecord::set_pose( float x,
					      float y,
					      float theta,
					      float* covariance )
{
  m_pose.x( x );
  m_pose.y( y );
  m_pose.yaw( theta );
  // TODO: covariance
}

void
WorldInfoDataContainer::PoseRecord::set_velocity( float vel_x,
						  float vel_y,
						  float vel_theta,
						  float* covariance )
{
  m_velocity.x() = vel_x;
  m_velocity.y() = vel_y;
  m_velocity.z() = vel_theta;
  // TODO: covariance
}

HomPose2d
WorldInfoDataContainer::PoseRecord::pose()
{
  return m_pose;
}

Matrix
WorldInfoDataContainer::PoseRecord::pose_covariance()
{
  return m_pose_covariance;
}

HomVector
WorldInfoDataContainer::PoseRecord::velocity()
{
  return m_velocity;
}

WorldInfoDataContainer::OpponentsRecord::OpponentsRecord()
{
}

WorldInfoDataContainer::OpponentsRecord::~OpponentsRecord()
{
}

void
WorldInfoDataContainer::OpponentsRecord::set_pos( unsigned int id,
						  float distance,
						  float bearing,
						  float* covariance )
{
  // TODO
}

void
WorldInfoDataContainer::OpponentsRecord::set_pos( HomPose2d robot_pose,
						  unsigned int opp_id,
						  float rel_distance,
						  float rel_bearing,
						  float* rel_covariance )
{
  HomTransform local_to_global;
  local_to_global.rotate_z( robot_pose.yaw() );
  local_to_global.trans( robot_pose.x(), robot_pose.y() );
  HomPoint o = local_to_global * HomPoint( cos( rel_bearing ) * rel_distance,
					   sin( rel_bearing ) * rel_distance );

  m_glob_opp_positions[ opp_id ] = o;

  // TODO: covariance
}

void
WorldInfoDataContainer::OpponentsRecord::disappeared( unsigned int opp_id )
{
  m_glob_opp_positions.erase( opp_id );
}

map<unsigned int, HomPoint>
WorldInfoDataContainer::OpponentsRecord::positions()
{
  return m_glob_opp_positions;
}


/** @class WorldInfoDataContainer <worldinfo_utils/data_container.h>
 * Data container to store and exchange worldinfo data.
 * @author Daniel Beck
 */

/** Constructor.
 * @param clock pointer to a Clock
 * @param timeout_msec timeout in milliseconds
 */
WorldInfoDataContainer::WorldInfoDataContainer( Clock* clock,
						long timeout_msec )
{
  m_clock        = clock;
  m_timeout_msec = timeout_msec;

  m_host_id = 0;

  m_own_team_color = TEAM_CYAN;
  m_own_goal_color = GOAL_BLUE;

  m_game_state.game_state    = GS_FROZEN;
  m_game_state.state_team    = TEAM_BOTH;
  m_game_state.score_cyan    = 0;
  m_game_state.score_magenta = 0;
  m_game_state.half          = HALF_FIRST;

  m_new_data_available = false;
  m_new_host           = false;
  m_host_timedout      = false;
}

/** Destructor. */
WorldInfoDataContainer::~WorldInfoDataContainer()
{
}

/** Check for timed out hosts.
 * This method should be called regularly to remove hosts from the
 * data container from which no data has been received in a certain
 * amount of time.
 * @return true if there are timed out hosts
 */
bool
WorldInfoDataContainer::check_timeout()
{
  Time now(m_clock);
  now.stamp();

  m_timedout_hosts.lock();
  m_timedout_hosts.clear();
  m_timedout_hosts.unlock();

  m_hosts.lock();
  HostLockMap::iterator iter = m_hosts.begin();
  while ( iter != m_hosts.end() )
    {
      unsigned int id = iter->second;

      if ( now.in_msec() - m_last_seen[id] < m_timeout_msec )
	{ ++iter; }
      else
	{
	  m_last_seen.lock();
	  m_last_seen.erase(id);
	  m_last_seen.unlock();

 	  m_ball_positions.lock();
	  m_ball_positions.erase(id);
	  m_ball_positions.unlock();

	  m_robot_poses.lock();
	  m_robot_poses.erase(id);
	  m_robot_poses.unlock();

	  m_timedout_hosts.lock();
	  m_timedout_hosts.push_back(iter->first);
	  m_timedout_hosts.unlock();

	  m_hosts.erase(iter++);
	  m_host_timedout = true;
	}
    }

  m_hosts.unlock();

  return m_timedout_hosts.size() != 0;
}

/** Set the time out.
 * @param msec time out value in milliseconds
 */
void
WorldInfoDataContainer::set_timeout(long msec)
{
  m_timeout_msec = msec;
}

/** Obtain the list of active hosts.
 * @param check_timeout_first if true check_timeout() is called before
 * the list is compiled
 * @return the list of active hosts
 */
std::list<std::string>
WorldInfoDataContainer::get_hosts(bool check_timeout_first)
{
  if (check_timeout_first)
    { check_timeout(); }

  list<string> hosts;

  m_hosts.lock();
  for ( HostLockMap::iterator iter = m_hosts.begin();
	iter != m_hosts.end();
	++iter )
    { hosts.push_back( iter->first ); }
  m_hosts.unlock();

  return hosts;  
}

/** Obtain the list of timedout hosts.
 * Hosts that have been marked as timedout in the last call of
 * check_timeout().
 * @return the list of timedout hosts
 */
std::list<std::string>
WorldInfoDataContainer::get_timedout_hosts()
{
  list<string> timedout_hosts;

  m_timedout_hosts.lock();
  for ( HostLockList::iterator iter = m_timedout_hosts.begin();
	iter != m_timedout_hosts.end();
	++iter )
    { timedout_hosts.push_back( *iter ); }
  m_timedout_hosts.unlock();

  return timedout_hosts;
}

/** Check whehter new data is available.
 * @return true if new data is available.
 */
bool
WorldInfoDataContainer::new_data_available()
{
  bool new_data = m_new_data_available;
  m_new_data_available = false;
  return new_data;  
}

/** Check whether a new host has been added recently.
 * @return true if a new host has been added recently
 */
bool
WorldInfoDataContainer::new_host()
{
  bool new_host = m_new_host;
  m_new_host = false;
  return new_host;
}

/** Check whether a host has timed out.
 * @return true if a host has timed out recently
 */
bool
WorldInfoDataContainer::host_timedout()
{
  bool host_timedout = m_host_timedout;
  m_host_timedout = false;
  return host_timedout;
}


/** Set the pose of a robot.
 * @param host the hostname of the robot
 * @param x the x-coordinate of the robot's global position
 * @param y the y-coordinate of the robot's global position
 * @param theta the global orientation of the robot
 * @param covariance covariance associated with the position
 * estimation
 */
void
WorldInfoDataContainer::set_robot_pose( const char* host,
					float x,
					float y,
					float theta,
					float* covariance )
{
  PoseLockMap::iterator iter;
  unsigned int id = get_host_id( host );
  clock_in_host( id );

  m_robot_poses.lock();
  iter = m_robot_poses.find( id );
  if ( iter == m_robot_poses.end() )
    {
      PoseRecord pose_record;
      pose_record.set_pose( x, y, theta, covariance );
      m_robot_poses[ id ] = pose_record;
    }
  else
    {
      iter->second.set_pose( x, y, theta, covariance );
    }
  m_robot_poses.unlock();

  m_new_data_available = true;
}

/** Obtain the pose of the given robot.
 * @param host the hostname of the robot
 * @param pose reference to a HomPose where the pose will be stored
 * @return false if no pose for the requested robot could be found
 */
bool
WorldInfoDataContainer::get_robot_pose( const char* host,
					HomPose2d& pose )
{
  bool found = false;
  unsigned int id = get_host_id( host );

  m_robot_poses.lock();
  PoseLockMap::iterator iter = m_robot_poses.find( id );

  if ( iter != m_robot_poses.end() )
    {
      pose = iter->second.pose();
      found = true;
    }
  m_robot_poses.unlock();

  return found;
}


/** Get the position of a certain robot.
 * @param host the hostname of the robot
 * @param pose reference to a HomPoint where the global position of
 * the robot is written to
 * @param pose_cov reference to a Matrix where the covariance of the
 * robot position is written to
 * @return true if a pose for the robot could be found
 */
bool
WorldInfoDataContainer::get_robot_pose( const char* host,
					HomPose2d& pose,
					Matrix& pose_cov )
{
  bool found = false;
  unsigned int id = get_host_id( host );

  m_robot_poses.lock();
  PoseLockMap::iterator iter = m_robot_poses.find( id );

  if ( iter != m_robot_poses.end() )
    {
      pose = iter->second.pose();
      pose_cov = iter->second.pose_covariance();
      found = true;
    }
  m_robot_poses.unlock();

  return found;
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
					    float vel_x,
					    float vel_y,
					    float vel_theta,
					    float* covariance )
{
  PoseLockMap::iterator iter;
  unsigned int id = get_host_id( host );
  clock_in_host( id );

  m_robot_poses.lock();
  iter = m_robot_poses.find( id );
  if ( iter == m_robot_poses.end() )
    {
      PoseRecord pose_record;
      pose_record.set_velocity( vel_x, vel_y, vel_theta, covariance );
      m_robot_poses[ id ] = pose_record;
    }
  else
    {
      iter->second.set_velocity( vel_x, vel_y, vel_theta, covariance );
    }
  m_robot_poses.unlock();

  m_new_data_available = true;
}


/** Obtain current velocity of the specified robot.
 * @param host the hostname of the robot
 * @param robot_vel reference to a HomVector where the velocity
 * information is written to
 * @return true, if velocity information for the specified host are
 * available
 */
bool
WorldInfoDataContainer::get_robot_velocity( const char* host,
					    HomVector& robot_vel )
{
  // TODO
  return true;
}


/** Set the ball position estimation of a robot.
 * @param host the hostname of the robot
 * @param visible visible or not
 * @param visibility_history visible/not visible for n iterations
 * @param dist distance to the robot
 * @param bearing vertical angle to the ball
 * @param slope the horizontal angle to the ball
 * @param covariance covariance associated with the position estimation
 */
void
WorldInfoDataContainer::set_ball_pos( const char* host,
				      bool visible,
				      int visibility_history,
				      float dist,
				      float bearing,
				      float slope,
				      float* covariance )
{
  BallLockMap::iterator iter;
  unsigned int id = get_host_id( host );
  clock_in_host( id );

  m_ball_positions.lock();
  iter = m_ball_positions.find( id );
  if ( iter == m_ball_positions.end() )
    {
      BallRecord ball_record;
      ball_record.set_visible( visible, visibility_history );
      ball_record.set_pos( dist, bearing, slope, covariance );
      m_ball_positions[ id ] = ball_record;
    }
  else
    {
      iter->second.set_visible( visible, visibility_history );
      iter->second.set_pos( dist, bearing, slope, covariance );
    }
  m_ball_positions.unlock();

  m_new_data_available = true;
}

/** Set the global ball position estimation of a robot.
 * @param host the hostname of the robot
 * @param visible visible or not
 * @param visibility_history visible/not visible for n iterations
 * @param x the x-coordinte of the global ball position
 * @param y the y-coordinte of the global ball position
 * @param z the z-coordinte of the global ball position
 * @param covariance covariance associated with the position estimation
 */
void
WorldInfoDataContainer::set_ball_pos_global( const char* host,
					     bool visible,
					     int visibility_history,
					     float x,
					     float y,
					     float z,
					     float* covariance )
{
  BallLockMap::iterator iter;
  unsigned int id = get_host_id( host );
  clock_in_host( id );

  m_ball_positions.lock();
  iter = m_ball_positions.find( id );
  if ( iter == m_ball_positions.end() )
    {
      BallRecord ball_record;
      ball_record.set_visible( visible, visibility_history );
      ball_record.set_pos_global( x, y, z, covariance );
      m_ball_positions[ id ] = ball_record;
    }
  else
    {
      iter->second.set_visible( visible, visibility_history );
      iter->second.set_pos_global( x, y, z, covariance );
    }
  m_ball_positions.unlock();

  m_new_data_available = true;
}


/** Get the ball position estimation of a certain robot.
 * @param host the hostname of the robot
 * @param pos reference to a HomPolar where the position is written to
 * @return true if a global ball position was found
 */
bool
WorldInfoDataContainer::get_ball_pos_relative( const char* host,
					       HomPolar& pos )
{
  bool found = false;
  unsigned int id = get_host_id( host );

  m_ball_positions.lock();
  BallLockMap::iterator iter = m_ball_positions.find( id );

  if ( iter != m_ball_positions.end() )
    {
      pos = iter->second.pos_relative();
      found = iter->second.visible();
    }
  m_ball_positions.unlock();

  return found;
}


/** Get the ball position estimation of a certain robot.
 * @param host the hostname of the robot
 * @param pos reference to a HomPolar where the position is written to
 * @param pos_cov reference to a Matrix where the ball position
 * covariance is written to
 * @return true if a global ball position was found
 */
bool
WorldInfoDataContainer::get_ball_pos_relative( const char* host,
					       HomPolar& pos, 
					       Matrix& pos_cov )
{
  bool found = false;
  unsigned int id = get_host_id( host );

  m_ball_positions.lock();
  BallLockMap::iterator iter = m_ball_positions.find( id );

  if ( iter != m_ball_positions.end() )
    {
      pos = iter->second.pos_relative();
      pos_cov = iter->second.covariance_relative();
      found = iter->second.visible();
    }
  m_ball_positions.unlock();

  return found;
}


/** Get the global position of the ball as it is estimated by the
 * specified robot.
 * @param host the robot's hostname
 * @param pos refercence to a HomPoint where the position of the ball
 * written to
 */
bool
WorldInfoDataContainer::get_ball_pos_global( const char* host,
					     HomPoint& pos )
{
  bool found = false;
  unsigned int id = get_host_id( host );

  m_ball_positions.lock();
  m_robot_poses.lock();
  BallLockMap::iterator ball_iter = m_ball_positions.find( id );
  PoseLockMap::iterator pose_iter = m_robot_poses.find( id );

  if ( ball_iter != m_ball_positions.end() &&
       pose_iter != m_robot_poses.end() )
    {
      HomPose2d robot_pose = pose_iter->second.pose();
      pos = ball_iter->second.pos_global( robot_pose.x(),
					  robot_pose.y(),
					  robot_pose.yaw() );
      found = ball_iter->second.visible();
    }
  m_robot_poses.unlock();
  m_ball_positions.unlock();

  return found;
}


/** Set the ball velocity as it is estimated by the specified robot.
 * @param host the hostname of the robot
 * @param vel_x the ball velocity in x-direction of the robot-centered
 * coordinate system
 * @param vel_y the ball velocity in y-direction of the robot-centered
 * coordinate system
 * @param vel_z the ball velocity in z-direction of the robot-centered
 * coordinate system
 * @param covariance ball velocity covariance
 */
void
WorldInfoDataContainer::set_ball_velocity( const char* host, 
					   float vel_x,
					   float vel_y,
					   float vel_z, 
					   float* covariance )
{
  BallLockMap::iterator iter;
  unsigned int id = get_host_id( host );
  clock_in_host( id );

  m_ball_positions.lock();
  iter = m_ball_positions.find( id );
  if ( iter == m_ball_positions.end() )
    {
      BallRecord ball_record;
      ball_record.set_velocity( vel_x, vel_y, vel_z, covariance );
      m_ball_positions[ id ] = ball_record;
    }
  else
    {
      iter->second.set_velocity( vel_x, vel_y, vel_z, covariance );
    }
  m_ball_positions.unlock();

  m_new_data_available = true;
}


/** Obtain ball velocity information for specified robot.
 * @param host the hostname of the robot
 * @param ball_vel refrence to a HomVector where the velocity
 * information is written to
 * @return true if ball velocity information from the specified robot
 * are available
 */
bool
WorldInfoDataContainer::get_ball_velocity( const char* host,
					   HomVector& ball_vel )
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
WorldInfoDataContainer::set_opponent_pos( const char* host,
					  unsigned int uid,
					  float distance,
					  float angle,
					  float* covariance )
{
  unsigned int id = get_host_id( host );
  clock_in_host( id );

  m_opponents.lock();
  m_robot_poses.lock();
  OpponentsLockMap::iterator oit = m_opponents.find( id );
  PoseLockMap::iterator      pit = m_robot_poses.find( id );

  HomPose2d pose;
  if ( pit != m_robot_poses.end() )
  { pose = pit->second.pose(); }

  if ( oit == m_opponents.end() )
  {
    OpponentsRecord opponents_record;
    opponents_record.set_pos( pose, uid, distance, angle, covariance );
    m_opponents[ id ] = opponents_record;
  }
  else
  {
    oit->second.set_pos( pose, uid, distance, angle, covariance );
  }
  m_robot_poses.unlock();
  m_opponents.unlock();

  m_new_data_available = true;
}


/** Remove the opponent with the given ID form the list of opponents
 * seen by the given robot.
 * @param host the hostname of the robot
 * @param uid the uid of the opponent
 */
void
WorldInfoDataContainer::opponent_disappeared( const char* host, unsigned int uid )
{
  unsigned int id = get_host_id( host );

  m_opponents.lock();
  OpponentsLockMap::iterator iter = m_opponents.find( id );
  if ( iter != m_opponents.end() )
    { iter->second.disappeared( uid ); }
  m_opponents.unlock();

  m_new_data_available = true;
}


/** Get all oppenents detected by a certain robot.
 * @param host hostname of the robot
 * @param opp_positions map containing the positions of the detected
 * opponents
 * @return false if no data about opponents is available from the
 * given robot
 */
bool
WorldInfoDataContainer::get_opponent_pos( const char* host,
					  map<unsigned int, HomPoint>& opp_positions )
{
  bool found = false;
  unsigned int id = get_host_id( host );

  m_opponents.lock();
  OpponentsLockMap::iterator iter = m_opponents.find( id );
  if ( iter != m_opponents.end() )
  {
    opp_positions = iter->second.positions();
    found = true;
  }
  m_opponents.unlock();

  return found;
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
WorldInfoDataContainer::set_game_state( int game_state,
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
  char* game_state;
  if (asprintf( &game_state, "%s [%s]",
		worldinfo_msl_gamestate_tostring((worldinfo_msl_gamestate_t)m_game_state.game_state),
		worldinfo_gamestate_team_tostring(m_game_state.state_team) ) == -1) {
    throw OutOfMemoryException("Failed to allocate game state string");
  }

  string state_string(game_state);
  free(game_state);
  return state_string;
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
      m_new_host    = true;
    }
  else
    { id = iter->second; }
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

} // end namespace fawkes
