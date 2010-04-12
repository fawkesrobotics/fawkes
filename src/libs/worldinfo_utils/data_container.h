
/***************************************************************************
 *  data_container.h - World info data container
 *
 *  Created: Thu April 10 16:16:17 2008
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

#ifndef __TOOLS_WORLDINFO_VIEWER_DATA_CONTAINER_H_
#define __TOOLS_WORLDINFO_VIEWER_DATA_CONTAINER_H_

#include <geometry/matrix.h>
#include <geometry/hom_point.h>
#include <geometry/hom_polar.h>
#include <geometry/hom_pose_2d.h>
#include <core/utils/lock_map.h>
#include <core/utils/lock_list.h>
#include <netcomm/worldinfo/enums.h>

#include <string>
#include <list>
#include <vector>
#include <map>

namespace fawkes {

class Clock;

class WorldInfoDataContainer
{
 public:
  WorldInfoDataContainer(Clock* clock, long timeout_msec = 3000);
  ~WorldInfoDataContainer();

  /** Container struct for momentary game state infos. */
  struct GameState
  {
    int game_state;       /**< current game state */
    worldinfo_gamestate_team_t state_team;  /**< team association of the game state */
    unsigned int score_cyan;                /**< socre of the cyan-colored team */
    unsigned int score_magenta;             /**< score of the magenta-colored team */
    worldinfo_gamestate_half_t half;        /**< first or second half */
  };

  // management
  bool                   check_timeout();
  void                   set_timeout(long msec);
  std::list<std::string> get_hosts(bool check_timeout_first = false);
  std::list<std::string> get_timedout_hosts();

  bool new_data_available();
  bool new_host();
  bool host_timedout();

  // (own) pose
  void set_robot_pose( const char* from_host, float x, float y, float theta,
		       float* covariance );
  bool get_robot_pose( const char* host, HomPose2d& robot_pose );
  bool get_robot_pose( const char* host, HomPose2d& robot_pose,
		       Matrix& robot_pose_cov );

  // (own) velocity
  void set_robot_velocity( const char* from_host,
			   float vel_x, float vel_y, float vel_theta,
			   float* covariance );
  bool get_robot_velocity( const char* host, HomVector& robot_vel );

  // ball position
  void set_ball_pos( const char* from_host, bool visible, int visibility_history,
		     float dist, float bearing, float slope, float* covariance );
  void set_ball_pos_global( const char* from_host,
			    bool visible, int visibility_history,
			    float x, float y, float z, float* covariance );
  bool get_ball_pos_relative( const char* host, HomPolar& ball_pos );
  bool get_ball_pos_relative( const char* host, HomPolar& ball_pos,
			      Matrix &ball_pos_cov );
  bool get_ball_pos_global(const char* host, HomPoint& ball_pos);

  // ball velocity
  void set_ball_velocity( const char* from_host,
			  float vel_x, float vel_y, float vel_z,
			  float* covariance );
  bool get_ball_velocity( const char* from_host, HomVector& ball_vel );

  // opponents
  void set_opponent_pos( const char* from_host, unsigned int uid,
			 float distance, float angle, float* covariance );
  void opponent_disappeared( const char* from_host, unsigned int uid );
  bool get_opponent_pos( const char* host,
			 std::map<unsigned int, HomPoint>& opp_positions );

  // gamestate
  void set_game_state( int game_state,
		       worldinfo_gamestate_team_t state_team,
		       unsigned int score_cyan,
		       unsigned int score_magenta,
		       worldinfo_gamestate_team_t own_team,
		       worldinfo_gamestate_goalcolor_t own_goal_color,
		       worldinfo_gamestate_half_t half );


  GameState                       get_game_state() const;
  std::string                     get_game_state_string() const;
  std::string                     get_half_string() const;
  unsigned int                    get_own_score() const;
  unsigned int                    get_other_score() const;
  worldinfo_gamestate_team_t      get_own_team_color() const;
  std::string                     get_own_team_color_string() const;
  worldinfo_gamestate_goalcolor_t get_own_goal_color() const;
  std::string                     get_own_goal_color_string() const;


 private:

  /* data structures for internal storage */
  /* Ball */
  class BallRecord
  {
  public:
    BallRecord();
    virtual ~BallRecord();

    void set_pos( float dist, float bearing, float slope,
		  float* covariance = NULL );
    void set_pos_global( float x, float y, float z,
			 float* covariance = NULL );
    void set_visible( bool visible, int visibility_history );
    void set_velocity( float vel_x, float vel_y, float vel_z,
		       float* covariance = NULL );
    
    bool      visible() const;
    int       visibility_history() const;
    HomPolar  pos_relative();
    HomVector vel_relative();
    Matrix    covariance_relative();
    HomPoint  pos_global();
    HomPoint  pos_global( float ref_x, float ref_y, float ref_theta );
    HomVector vel_global( float vel_x, float vel_y, float vel_theta,
			  float ref_theta );

  private:
    bool      m_is_global;
    HomPolar  m_rel_pos;
    HomVector m_rel_vel;
    Matrix    m_rel_cov;
    HomPoint  m_glob_pos;
    bool      m_visible;
    int       m_visibility_history;
  };

  /* Pose */
  class PoseRecord
  {
  public:
    PoseRecord();
    virtual ~PoseRecord();

    void set_pose( float x, float y, float theta,
		   float* covariance = NULL );
    void set_velocity( float vel_x, float vel_y, float vel_theta,
		       float* covariance = NULL );

    HomPose2d pose();
    Matrix    pose_covariance();
    HomVector velocity();
    Matrix    velocity_covariance();

  private:
    HomPose2d m_pose;
    Matrix    m_pose_covariance;
    HomVector m_velocity;
    Matrix    m_velocity_covariance;
  };

  /* Opponents */
  class OpponentsRecord
  {
  public:
    OpponentsRecord();
    virtual ~OpponentsRecord();

    void set_pos( unsigned int opp_id, float distance, float bearing,
		  float* covariance = NULL );
    void set_pos( HomPose2d robot_pose,
		  unsigned int opp_id, float rel_dist, float rel_bearing,
		  float* rel_covariance = NULL );
    void disappeared( unsigned int opp_id );

    std::map<unsigned int, HomPoint> positions();

  private:
    std::map<unsigned int, HomPoint> m_glob_opp_positions;
  };

  /* private methods */
  unsigned int get_host_id(std::string host);
  void         clock_in_host(unsigned int id);


  /* type definitions */
  typedef LockMap<std::string, unsigned int>     HostLockMap;
  typedef LockList<std::string>                  HostLockList;
  typedef LockMap<unsigned int, long>            TimeLockMap;
  typedef LockMap<unsigned int, BallRecord>      BallLockMap;
  typedef LockMap<unsigned int, PoseRecord>      PoseLockMap;
  typedef LockMap<unsigned int, OpponentsRecord> OpponentsLockMap;

  /* member variables */
  unsigned int     m_host_id;

  HostLockMap      m_hosts;
  HostLockList     m_timedout_hosts;
  TimeLockMap      m_last_seen;
  BallLockMap      m_ball_positions;
  PoseLockMap      m_robot_poses;
  OpponentsLockMap m_opponents;
  
  GameState                       m_game_state;
  worldinfo_gamestate_team_t      m_own_team_color;
  worldinfo_gamestate_goalcolor_t m_own_goal_color;

  Clock* m_clock;
  long   m_timeout_msec;

  bool m_new_data_available;
  bool m_new_host;
  bool m_host_timedout;

};

} // end namespace fawkes

#endif /* __TOOLS_WORLDINFO_VIEWER_DATA_CONTAINER_H_ */
