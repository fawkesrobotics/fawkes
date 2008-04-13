
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
