/***************************************************************************
 *  pathfinder.h - The Pathfinder of the navigator
 *
 *  Generated: Tue Jun 05 13:50:17 2007
 *  Copyright  2007  Martin Liebenberg
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#ifndef __NAVIGATOR_PATHFINDER_H_
#define __NAVIGATOR_PATHFINDER_H_

extern "C"
{
#include <gts.h>
}

#include <vector>
#include <math.h>

#include "gts/gts_obstacle.h"
#include <plugins/navigator/libnavi/obstacle.h>

using namespace std;


class Pathfinder
{
 public:
  Pathfinder();
  Pathfinder(double robot_width, gint scanning_area_width, gint scanning_area_height);
  ~Pathfinder();
        
  void setTarget(double distance, double direction_rad);
  void setTargetPoint(GtsPoint * target_point);
  void setTargetPolar(double distance, double direction_degree);
  void setTarget_cartesian(double x, double y);
        
  void setObstacles(std::vector< Obstacle > obstacles);

  std::vector< GtsPoint * > getPath();
        
        
  //Obstacle * getNearestGtsObstacle();
  GtsPoint * getNearestPoint();
  GtsObstacle * getNearestGtsObstacle();
  GtsSurface * getSurface();
  GtsPoint * getRobotPoint();
  GtsPoint * getTargetPoint();

 private:
        
  std::vector< Obstacle > map;


  gint scanning_area_width;
  gint scanning_area_height;

  
  GtsSurface * surface;
   
  GtsPoint * robot_point;
  GtsPoint * target_point;
   
  double robot_width;
  
  std::vector< GtsObstacle *> obstacles;  
  
  void initSurface();
        
  //after the deletion of the surface, we have to regain the obstacles
  void regainObstacles();
  
  void calculateDelaunay();
  void delaunay(GtsSurface * surface, std::vector< GtsObstacle *> obstacles);
 
  static void getVertexes(GtsVertex *vertex, GtsFifo * fifo);
 
  bool test_straight_ahead();
  bool out_of_area(double x, double y);
};

#endif
