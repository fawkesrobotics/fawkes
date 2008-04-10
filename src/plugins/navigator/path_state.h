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

#ifndef __NAVIGATOR_PATH_STATE_H_
#define __NAVIGATOR_PATH_STATE_H_

#include <libs/utils/search/astar_state.h>

#include <vector>

extern "C"
{
#include <gts.h>
}

class PathState : public AStarState
{
 public:

  PathState();
  PathState(GtsSurface *surface, GtsEdge *current_edge, GtsPoint *current_point, GtsFace *current_face, GtsPoint *target,
            GtsPoint *start_point, double pastCost, PathState *father);
  ~PathState();
  
  GtsPoint *getPoint();
  long calculateKey();
  double estimate();
  bool isGoal();
  std::vector< AStarState * > generateChildren();
  GtsEdge *getEdge();
  
  
 private:
    
  void addChild(GtsPoint *p1, GtsPoint *p2, GtsEdge *next_edge,  std::vector< AStarState * > &children);
  GtsPoint *nextPoint(GtsPoint *p1, GtsPoint *p2, double &newCost);
  double shortest_difference_between_two_angles(double angle1, double angle2);
   
  static long keyCount;
  GtsPoint *current_point;
  GtsFace *current_face;
  GtsEdge *current_edge;
  GtsPoint *target_point;
  GtsPoint *start_point;
  GtsSurface *surface;
  gdouble robot_width;
  gdouble obstacle1_radius; 
  gdouble obstacle2_radius; 
};

#endif
