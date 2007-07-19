/***************************************************************************
 *  navigator.h - The navigator of fawkes
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

#ifndef __NAVIGATOR_NAVIGATOR_H_
#define __NAVIGATOR_NAVIGATOR_H_

extern "C" 
{
#include <gts.h>
}
#include <vector> 
#include <list>
#include <iostream>
 
 
#include <plugins/navigator/libnavi/obstacle.h>
#include <plugins/navigator/gts/gts_obstacle.h>
 
class NPoint;
class NLine;
class Pathfinder;


class Navigator
{
 public:
  

    
  Navigator();
  ~Navigator();
        
  void goTo_cartesian(double x, double y);
  void goTo_cartesian(double x, double y, double velocity);
  /*
    void goTo_degree(double ori, double distance);
    void goTo_rad(double ori, double distance);
    void goTo(double ori, double distance, std::vector< Obstacle * >);
  */
  void setObstacles(std::vector< Obstacle  >);
  //void addObstacle(Obstacle obstacle);
  
  void setVelocity(double velocity);
  void setVelocityRotation(double velocity_rotation);
  double getVelocity();
    
  double getVelocityX();
  double getVelocityY();
  // double getVelocityRotation();
  double getOrientation();
    
  void setElapsedTime(double elapsedTime);
  double getElapsedTime();
    
                                        
  void setRoute(std::vector<GtsPoint *> route);
  void mainLoop();
   
  std::list<NPoint *>  *get_surface_points();
  std::list<NLine *> *get_surface_lines();
   
  //fuer NavigatorTest
  //evtl. direkt vom pathfinder holen?
 protected:
  int getCount();
  std::vector< GtsPoint * > getPath();
  GtsSurface * getSurface();
  
   
  
  GtsObstacle * nearestObstacle();
 
  GtsPoint * getTargetPoint();
  GtsPoint * getRobotPoint();
   
  int binomialCoefficient(int n, int k);
  double bernstein(int i, int n, double t);
   
  //z.B. bei navigator_test beim Routezeichnen
  std::vector<GtsPoint *>  getRoute();
    
 private:
    
  Pathfinder * pathfinder; 
   
  //beinhaltet die Target Punkte, der einzelnen Abschnitte
  std::vector<GtsPoint*> route;
   
  static void getEdges(GtsEdge *edge, GtsFifo * fifo);

  static void getVertexes(GtsVertex *vertex, GtsFifo * fifo);
   
  bool running_route;
    
  //Abtastbereich der Sensorik
  gint scanning_area_width;
  gint scanning_area_height;
     
  double robot_width;
  // GtsPoint * robot_point;
   
  //count of the points of the bezier
  int count;
  //index of the bezier
  double t;
  //beinhaltet die Punkte der Bezier Kurve
   
  //set if the smoothController should not control
  //it avoids orbits
  //umbenennen in avoid smoothControl
  bool newDirection;
   
  std::vector< GtsPoint * > path;
  std::vector< Obstacle > map;
   
  GTimer * time_emitter;
  double elapsed_time;
  double last_time;
      
  double last_degree;
  double current_degree;
   
  //cm/sec 
  double velocity;
   
  double velocity_x;
  double velocity_y;
   
  double step_x;
  double step_y;
   
  //degree/sec
  double velocity_rotation;
   
  double orientation;
   
  bool running;
   
  void destroy_path();
   
  double s(double t);
};
#endif
