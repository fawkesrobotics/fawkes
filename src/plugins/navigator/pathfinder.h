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
#include <map>
#include <math.h>

#include <plugins/navigator/gts/gts_obstacle.h>
#include <plugins/navigator/libnavi/obstacle.h>

class Mutex;

class Pathfinder
  {
  public:
    Pathfinder();
    Pathfinder(double robot_width, double scanning_area_width, double scanning_area_height);
    ~Pathfinder();

    void set_target(double x, double y);

    void set_obstacles(std::vector< Obstacle > obstacles);
    void add_obstacle(Obstacle obstacle);

    std::vector< GtsPoint * > get_path();

    Obstacle* get_nearest_obstacle();
    GtsSurface * get_surface();
    GtsPoint * get_robot_point();
    GtsPoint * get_target_point();
    bool out_of_area(double x, double y);

  private:
    GtsVertex* v0;
    GtsVertex* v1;
    GtsVertex* v2;
    GtsVertex* v3;

    GtsEdge*  e0;
    GtsEdge*  e1;
    GtsEdge*  e2;
    GtsEdge*  e3;
    GtsEdge*  e4;

    std::vector< Obstacle > map;

    double scanning_area_width;
    double scanning_area_height;

    GtsSurface * surface;

    GtsPoint * robot_point;
    GtsPoint * target_point;
    GtsFace* start_face;

    float robot_width;
    float minimum_distance;

    //maybe it's needless
    Mutex* surface_mutex;

    std::vector< GtsObstacle *> obstacles;

    void init_surface();

    //after the deletion of the surface, we have to regain the obstacles
    void regain_obstacles();

    void calculate_delaunay();
    void delaunay(GtsSurface * surface, std::vector< GtsObstacle *> obstacles);
    void find_bisector(GtsVertex* current_vertex, std::map<GtsVertex*, GtsVertex*>* map);

    static void get_vertexes(GtsVertex *vertex, GtsFifo * fifo);
    static void remove_face(GtsFace *face, GtsEdge* edge);
    double p_sgn(double x);
    bool test_straight_ahead();
    bool is_in_rectangle(double point_x, double point_y,
                         double vertex1_x, double vertex1_y,
                         double vertex2_x, double vertex2_y,
                         double vertex3_x, double vertex3_y,
                         double vertex4_x, double vertex4_y);
  };

#endif
