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
class Mutex;
class Configuration;


class Navigator
  {
  public:

    Navigator();
    ~Navigator();

    std::list<NPoint *>  *get_surface_points();
    std::list<NLine *> *get_surface_lines();
    std::list<NPoint *> *get_path_points();
    std::list<Obstacle *> *get_obstacles();
    NPoint * getTargetPoint();
  protected:
    void goto_cartesian(double x, double y);
    void goto_cartesian(double x, double y, double velocity);
    void goto_cartesian_ori(double x, double y, double ori);
    void goto_polar_ori(float phi, float dist, float ori);
    /*
      void goTo_degree(double ori, double distance);
      void goTo_rad(double ori, double distance);
      void goTo(double ori, double distance, std::vector< Obstacle * >);
    */
    void set_target_tolerance(float tolerance);
    void set_odometry_velocity_x(double velocity_x);
    void set_odometry_velocity_y(double velocity_y);
    void set_odometry_velocity_rotation(double rotation);
    void set_obstacles(std::vector< Obstacle  >);
    void erase_all_obstacles();
    void add_obstacle(Obstacle obstacle);

    void set_max_velocity(double velocity);
    void set_velocity_rotation(double velocity_rotation);

    double get_velocity_x();
    double get_velocity_y();
    double get_velocity_rotation();
    double get_orientation();

    void set_route(std::vector<GtsPoint *> route);
    void main_loop();


    // int binomialCoefficient(int n, int k);
    double bernstein(unsigned int i, unsigned int n, double t);

    //z.B. bei navigator_test beim Routezeichnen
    std::vector<GtsPoint *>  get_route();

  protected: /* members */
    /** Destination X coordinate */
    double dest_x;
    /** Destination Y coordinate */
    double dest_y;
    /** Orientation at destination */
    double dest_ori;

  private:

    Mutex *surface_mutex;
    Mutex *path_mutex;
    Mutex *velocity_mutex;

    Pathfinder * pathfinder;

    //beinhaltet die Target Punkte, der einzelnen Abschnitte
    std::vector<GtsPoint*> route;

    static void get_edges(GtsEdge *edge, GtsFifo * fifo);

    static void get_vertexes(GtsVertex *vertex, GtsFifo * fifo);

    bool running_route;

    float target_tolerance;

    //Abtastbereich der Sensorik
    double scanning_area_width;
    double scanning_area_height;

    double odometry_velocity_x;
    double odometry_velocity_y;
    double odometry_velocity_rotation;

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
    bool new_direction;

    std::vector< GtsPoint * > path;
    std::vector< Obstacle > map;

    GTimer * time_emitter;
    double elapsed_time;
    double last_time;

    double last_degree;
    double current_degree;

    //m/sec
    double current_velocity;
    double max_velocity;

    double velocity_x;
    double velocity_y;

    double step_x;
    double step_y;

    //degree/sec
    double velocity_rotation;

    double orientation;
    double desired_orientation;

    bool running;

    void destroy_path();

    double s(double t);
  };
#endif
