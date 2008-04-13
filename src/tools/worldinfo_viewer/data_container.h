
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
#include <string>
#include <vector>

class WorldInfoDataContainer
{
 public:
  WorldInfoDataContainer();
  ~WorldInfoDataContainer();

  void reset();

  std::vector<std::string> get_hosts();

  void set_robot_pose( const char* from_host, float x, float y, float theta,
		       float* covariance );

  void set_ball_pos( const char* from_host, float dist, 
		     float pitch, float yaw, float* covariance );

  HomVector get_ball_pos(const char* host);
  HomVector get_robot_pos(const char* host);

 private:
  typedef LockMap<std::string, unsigned int> HostLockMap;
  typedef LockMap<unsigned int, HomVector> PosLockMap;
    
  HostLockMap m_hosts;
  PosLockMap m_robot_pos;
  PosLockMap m_ball_pos;

  unsigned int m_host_id;
};

#endif /* __TOOLS_WORLDINFO_VIEWER_DATA_CONTAINER_H_ */ 
