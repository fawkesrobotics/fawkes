
/***************************************************************************
 *  navigator.h - The navigator of Fawkes
 *
 *  Created: Tue Jun 05 13:50:17 2007
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

#include <plugins/navigator/navigator.h>

#include <plugins/navigator/fuzzy/potential_field_controller.h>
#include <plugins/navigator/pathfinder.h>

#include <plugins/navigator/fuzzy/smoothing_controller.h>

#include <plugins/navigator/libnavi/npoint.h>
#include <plugins/navigator/libnavi/nline.h>
#include <utils/math/binomial_coefficient.h>
#include <utils/math/angle.h>
#include <geometry/vector.h>
#include <core/threading/mutex.h>

#include <cmath>

#define FUZZY_POTENTIAL_FIELD
#define FUZZY_SMOOTHING

/** @class Navigator plugins/navigator/navigator.h
 *   The navigator of fawkes.
 *   
 *   @author Martin Liebenberg
 */

/** Constructor */
Navigator::Navigator()
{
  //this is the area where the objects can be handled
  scanning_area_width = 50.; // m
  scanning_area_height = 50.; //m

  //the robot's width in m
  robot_width = 0.5;

  //the desired velocity of the robot
  current_velocity = 0; // m/s

  //the maximum velocity
  max_velocity = 0; // m/s

  //the velocities in the x and y direction
  velocity_x = 0;
  velocity_y = 0;

  odometry_velocity_x = 0;
  odometry_velocity_y = 0;
  odometry_velocity_rotation = 0;

  //the velocity of the rotation of the robot
  velocity_rotation = 0;
  orientation = 0;

  dest_x = dest_y = dest_ori = 0.f;

  //the index of the formula of the bezier curve
  t = 0;

  //the pathfinder object to get the pathes
  pathfinder = new Pathfinder(robot_width, scanning_area_width, scanning_area_height);

  //path needs not to be NULL
  path = pathfinder->get_path();

  count = (int)path.size();

  new_direction = false;

  time_emitter = g_timer_new();
  elapsed_time = 0;

  step_x = 0;
  step_y = 0;

  surface_mutex = new Mutex();
  path_mutex = new Mutex();

  target_tolerance = 0.02;
}

/** Destructor */
Navigator::~Navigator()
{
  delete pathfinder;
  delete surface_mutex;
  delete path_mutex;
  g_timer_destroy(time_emitter);
  destroy_path();
}

/** Pushes an edge out of a GTS fifo.
 *  It is needed for the gts_surface_foreach_edge function of GTS.
 * @param edge the pushed edge
 * @param fifo the GTS fifo with the edges
 */
void
Navigator::get_edges(GtsEdge *edge, GtsFifo * fifo)
{
  gts_fifo_push(fifo, edge);
}

/** Pushes a vertex out of a GTS fifo.
 *  It is needed for the gts_surface_foreach_edge function of GTS.
 * @param vertex the pushed vertex
 * @param fifo the GTS fifo with the vertexes
 */
void
Navigator::get_vertexes(GtsVertex *vertex, GtsFifo * fifo)
{
  gts_fifo_push(fifo, GTS_OBJECT(vertex));
}

/** Gets the points of the surface of the triangulation.
 * @return a list of all points of the surface.
 */
std::list<NPoint *> *
Navigator::get_surface_points()
{
  std::list<NPoint *> *p_list = new std::list<NPoint *>;
  surface_mutex->lock();

  GtsSurface *surface = pathfinder->get_surface();
  GtsFifo *vertexes = gts_fifo_new();

  gts_surface_foreach_vertex(surface, (GtsFunc) get_vertexes, vertexes);

  while(!gts_fifo_is_empty(vertexes))
    {
      GtsVertex * v = (GtsVertex *)gts_fifo_pop(vertexes);

      if(GTS_IS_OBSTACLE(v))
        {
          NPoint *p = new Obstacle(GTS_OBSTACLE(v)->width, GTS_POINT (v)->x, GTS_POINT (v)->y, 0);

          p_list->push_back(p);
        }
      else
        {
          NPoint *p = new NPoint();
          p->x = GTS_POINT (v)->x;
          p->y = GTS_POINT (v)->y;

          p_list->push_back(p);
        }
    }

  surface_mutex->unlock();
  gts_fifo_destroy(vertexes);
  return p_list;
}

/** Gets the obstacles maintained by the navigator.
 * @return a list of all obstacles.
 */
std::list<Obstacle *> *
Navigator::get_obstacles()
{
  std::list<Obstacle *> *o_list = new std::list<Obstacle *>;

  for(unsigned int i = 0; i < map.size(); i++)
    {
      Obstacle *o = new Obstacle(map[i].width, map[i].x, map[i].y, 0);
      o_list->push_back(o);
    }
  return o_list;
}

/** Gets the points of the path.
 * @return a list of all points of the path.
 */
std::list<NPoint *> *
Navigator::get_path_points()
{
  std::list<NPoint *> *p_list = new std::list<NPoint *>;
  path_mutex->lock();
  for(unsigned int i = 0; i < path.size(); i++)
    {
      NPoint *p = new NPoint();
      p->x = path[i]->x;
      p->y = path[i]->y;
      p_list->push_back(p);
    }
  path_mutex->unlock();
  return p_list;
}


/** Gets the lines of the surface of the triangulation.
 * @return a list of all lines of the surface.
 */
std::list<NLine *> *
Navigator::get_surface_lines()
{
  std::list<NLine *> *lines = new std::list<NLine *>;
  surface_mutex->lock();

  GtsSurface* surface = pathfinder->get_surface();
  GtsFifo* edges = gts_fifo_new ();
  gts_surface_foreach_edge(surface, (GtsFunc) get_edges, edges);

  while(!gts_fifo_is_empty(edges))
    {
      GtsEdge* e = (GtsEdge * )gts_fifo_pop(edges);
      NPoint* p1 = new NPoint();
      NPoint* p2 = new NPoint();

      GtsVertex* v1 = GTS_SEGMENT (e)->v1;
      p1->x = GTS_POINT (v1)->x;
      p1->y = GTS_POINT (v1)->y;

      GtsVertex* v2 = GTS_SEGMENT (e)->v2;
      p2->x = GTS_POINT (v2)->x;
      p2->y = GTS_POINT (v2)->y;

      lines->push_back(new NLine(p1, p2));
      delete p1;
      delete p2;
    }

  surface_mutex->unlock();
  gts_fifo_destroy(edges);
  return lines;
}

/** Sets the recognized obstacles to the navigator.
 * @param map a vector of obstacles
 */
void
Navigator::set_obstacles(std::vector< Obstacle > map)
{
  this->map = map;
  surface_mutex->lock();
  pathfinder->set_obstacles(map);
  surface_mutex->unlock();
}

/** Removes all obstacles from the surface.
 */
void
Navigator::erase_all_obstacles()
{
  surface_mutex->lock();
  map.clear();
  surface_mutex->unlock();
}

/** Adds an obstacle to the pathfinder.
 * @param obstacle an obstacle
 */
void
Navigator::add_obstacle(Obstacle obstacle)
{
  map.push_back(obstacle);
  pathfinder->add_obstacle(obstacle);
}

/** Sets the target tolerance.
 * @param tolerance the target tolerance
 */
void
Navigator::set_target_tolerance(float tolerance)
{
  target_tolerance = tolerance;
}

/** Sets the odometry velocity in x-direction.
 * @param velocity_x the velocity x
 */
void
Navigator::set_odometry_velocity_x(double velocity_x)
{
  odometry_velocity_x = velocity_x;
}

/** Sets the odometry velocity in y-direction.
 * @param velocity_y the velocity y
 */
void Navigator::set_odometry_velocity_y(double velocity_y)
{
  odometry_velocity_y = velocity_y;
}


/** Sets the odometry rotation velocity.
 * @param rotation rotation velocity
 */
void
Navigator::set_odometry_velocity_rotation(double rotation)
{
  odometry_velocity_rotation = rotation;
}

/** Sets the maximum velocity of the robot.
 * @param velocity the maximum velocity
 */
void
Navigator::set_max_velocity(double velocity)
{
  if(current_velocity > velocity)
    {
      current_velocity = velocity;
    }
  this->max_velocity = velocity;
}

/** Sets the velocity of the rotation of the robot.
 * @param velocity_rotation the velocity of the rotation
 */
void
Navigator::set_velocity_rotation(double velocity_rotation)
{
  this->velocity_rotation = velocity_rotation;
}

/** Returns the velocity of the rotation of the robot.
 * @return the velocity of the rotation
 */
double
Navigator::get_velocity_rotation()
{
  return velocity_rotation;
}


/** Returns the velocity of the robot in the x-direction.
 * @return the velocity in the x-direction
 */
double
Navigator::get_velocity_x()
{
  return velocity_x;
}

/** Returns the velocity of the robot in the y-direction.
 * @return the velocity in the y-direction
 */
double
Navigator::get_velocity_y()
{
  return velocity_y;
}


/** Returns the orientation of the robot.
 * @return the orientation of the robot
 */
double
Navigator::get_orientation()
{
  return orientation;
}

/** Sets a list of targets for a route.
 * @param route a vector of target-points
 */
void
Navigator::set_route(std::vector<GtsPoint*> route)
{
  if(route.size() > 0)
    {
      this->route = route;
      pathfinder->set_target(route.front()->x, route.front()->y);
      gts_object_destroy(GTS_OBJECT(path[path.size() - 1]));
      path[path.size() - 1] = gts_point_new(gts_point_class(), route.front()->x,
                                            route.front()->y, 0);
    }
}

/** Returns the current route of the navigator.
 * @return a vector of target points
 */
std::vector<GtsPoint *>
Navigator::get_route()
{
  return route;
}

/** Returns the target point of the navigator.
 * @return the target point
 */
NPoint *
Navigator::get_target_point()
{
  return new NPoint(pathfinder->get_target_point()->x, pathfinder->get_target_point()->y);
}

/** Bernstein polynomial for the bezier curve.
 * @param i
 * @param n
 * @param t
 * @return the value of the bernstein polynomial
 */
double
Navigator::bernstein(unsigned int i, unsigned int n, double t)
{
  //this would be not needed anymore because there is only
  //a cubic bezier curve to be calculated which can be done with a simpler formula
  return BinomialCoefficient::binoc(n, i) * pow(t, (int)i) * pow(1. - t, (int)(n - i));
}


/** Auxilliary function for calculating the velocities.
 * @param t
 * @return s
 */
double
Navigator::s(double t)
{
  //I gess this is not needed anymore and very inacurate
  //it was needed to recalculate the t for a driven distance on the curve
  double sum_x = 0;
  double sum_y = 0;
  //count -1 because of k + 1
  for(int k = 0; k < count - 1; k++)
    {
      sum_x += (path[k + 1]->x - path[k]->x) * bernstein(k, count - 2, t);
      sum_y += (path[k + 1]->y - path[k]->y) * bernstein(k, count - 2, t);
    }
  if(sqrt(pow(sum_x, 2.) + pow(sum_y, 2.)) == 0)
    return 0.0000000000001;
  else
    return sqrt(pow(sum_x, 2.) + pow(sum_y, 2.));
}

/** Sets a target.
 * @param x the x-coordinate of the target
 * @param y the y-coordinate of the target
 */
void
Navigator::goto_cartesian(double x, double y)
{
  goto_cartesian(x, y, max_velocity);
}

/** Sets a target.
 * @param x the x-coordinate of the target
 * @param y the y-coordinate of the target
 * @param ori the desired orientation at the target
 */
void
Navigator::goto_cartesian_ori(double x, double y, double ori)
{
  goto_cartesian(x, y, max_velocity);

  if(max_velocity > 0)
    {
      velocity_rotation = ori / (sqrt(pow(x, 2.) + pow(y, 2.)) / max_velocity);
    }
  else
    {
      velocity_rotation = ori / 3.;
    }
  //dest_ori and orientation is always positive
  //the velocity_rotation states the direction
  dest_ori = fabs(ori);
  orientation = 0;
}


/** Sets a target from polar coordinates.
 * @param phi the angle to the target looking forward
 * @param dist distance to the target
 * @param ori the desired orientation at the target
 */
void
Navigator::goto_polar_ori(float phi, float dist, float ori)
{
  double x = dist * cos(phi);
  double y = dist * sin(phi);

  goto_cartesian_ori(x, y, ori);
}


/** Sets a target and a velocity.
 *  The navigator will search a path and will drive along.
 * @param x the x-coordinate of the target
 * @param y the y-coordinate of the target
 * @param velocity the velocity of the robot
 */
void
Navigator::goto_cartesian(double x, double y, double velocity)
{
  step_x = 0;
  step_y = 0;
  t = 0;

  dest_ori = 0;
  dest_x = x;
  dest_y = y;

  this->current_velocity = velocity;

  destroy_path();
  pathfinder->set_target(x, y);

  surface_mutex->lock();
  path = pathfinder->get_path();
  surface_mutex->unlock();

  if(path[1]->x != 0)
    current_degree = atan2(path[1]->y , path[1]->x);
  else
    current_degree = 0;

  new_direction = true;

}

/** Destroys the current path.
 */
void
Navigator::destroy_path()
{
  for(unsigned int i = 0; i < path.size(); i++)
    {
      gts_object_destroy(GTS_OBJECT(path[i]));
    }
  path.clear();
}


/** The main loop in which the main calculations for the valocities are placed.
 * It is called from the navigator thread loop.
 */
void
Navigator::main_loop()
{
  double current_time = g_timer_elapsed( time_emitter, NULL );
  elapsed_time = current_time - last_time;
  last_time = current_time;

  step_x = elapsed_time * velocity_x;
  step_y = elapsed_time * velocity_y;

  double sum_x = 0;
  double sum_y = 0;

  /****************************
   *  Calculating the Gradient
   ***************************/
  //the following was the first implementation and needs to be rewritten
  //it handles only one bezier curve over all control points
  //and uses a very inacurate approximation of t
  double div_x;
  double div_y;

  double delta_s = sqrt(step_x * step_x + step_y * step_y);
  double f1 = 1/s(t);
  double f2 = 1/s(t + delta_s * f1);
  double f3 = 1/s(t + delta_s * f2);
  double f4 = 1/s(t + delta_s * f3);

  double t_Tn_Sn;

  if(count > 1)
    t_Tn_Sn = (delta_s/(count-1)) * (f1 + 2*f2 + 2*f3 + f4) / 6;
  else
    t_Tn_Sn = (delta_s) * (f1 + 2*f2 + 2*f3 + f4) / 6;

  t = t + t_Tn_Sn;

  //the end of the current path is reached
  //maybe t > 1 is too inexact
  if(t > 1 || target_tolerance >= gts_point_distance(path[0], path[path.size() -1]))
    {
      t= 0;
      step_x = 0;
      step_y = 0;
      new_direction = true;

      //stop at the end of the path
      if(path.size() == 2) //the robot is on the straight way to the end of the path
        {
          if(route.size() != 0)  //the robot drives along a route
            {
              if(route.size() == 1) //the robot is on the last part of the route and at the target
                {
                  current_velocity = 0;
                  gts_object_destroy(GTS_OBJECT(route[0]));
                  route.clear();
                }
              else //the robot is not at the target but somewhere on the route
                {
                  gts_object_destroy(GTS_OBJECT(route[0]));
                  route.erase(route.begin());
                  GtsPoint *p = route.front();
                  pathfinder->set_target(p->x, p->y);
                  gts_object_destroy(GTS_OBJECT(path[path.size() - 1]));
                  path[path.size() - 1] = gts_point_new(gts_point_class(), p->x, p->y, 0);
                }
            }
          else //the robot is at the target
            {
              current_velocity = 0;
            }
        }
    }

  //the bezier curve for the path
  for(int k = 0; k < count - 1; k++)
    {//the derivation of the bezier curve at t
      if(k + 1 < (int)path.size() && k < (int)path.size())
        {
          sum_x += (path[k + 1]->x - path[k]->x) * bernstein(k, count - 2, t);
          sum_y += (path[k + 1]->y - path[k]->y) * bernstein(k, count - 2, t);
        }
    }

  div_x = sum_x;
  div_y = sum_y;

  if(sqrt(pow(div_x, 2.) + pow(div_y, 2.)) != 0)
    {

#ifdef FUZZY_POTENTIAL_FIELD
      //this should be replaced by a not fuzzy potential field
      //it should take the angle of the velocity relative to the object into account

      // it may more efficent if the method would return the distance here too
      Obstacle* nearestObs = pathfinder->get_nearest_obstacle();

      //the fuzzy controller has to be reconfigured
      PotentialFieldController *controller = new PotentialFieldController(robot_width);
      double force;
      if(nearestObs != NULL)
        {
          double distance = sqrt(((nearestObs->x - pathfinder->get_robot_point()->x) * (nearestObs->x - pathfinder->get_robot_point()->x))
                                 + ((nearestObs->y - pathfinder->get_robot_point()->y) * (nearestObs->y - pathfinder->get_robot_point()->y))) - nearestObs->width/2. - robot_width/2.;
          force = controller->control(distance);
          div_x += (pathfinder->get_robot_point()->x - nearestObs->x) * force;
          div_y += (pathfinder->get_robot_point()->y - nearestObs->y) * force;
        }
      delete controller;
#endif //FUZZY_POTENTIAL_FIELD
#ifdef FUZZY_SMOOTHING
      //this controls the reaction of a course change
      //if the angle between the old and the new direction too large
      //the change is made smoother
      
      //the fuzzy controller has to be reconfigured
      SmoothingController cont;
      double grade = 0;

      if(div_x != 0)
        current_degree = atan2(div_y, div_x);
      else
        current_degree = 0;

      if(current_degree < 0)
        current_degree += M_PI * 2;

      double difference = last_degree - current_degree;

      if(difference > M_PI)
        difference = -(M_PI * 2 - difference);
      if(difference < -M_PI)
        difference = -(-M_PI *2 - difference);

      if(!new_direction)
        grade = cont.control(fabs(difference));
      else
        new_direction = false;

      div_x =  cos(current_degree + difference * grade );
      div_y =  sin(current_degree + difference * grade);
      last_degree = current_degree + difference * grade;
#endif //FUZZY_SMOOTHING

      //set the velocity
      velocity_x = (div_x * current_velocity) / sqrt((pow(div_x, 2.) + pow(div_y, 2.)));
      velocity_y = (div_y * current_velocity) / sqrt((pow(div_x, 2.) + pow(div_y, 2.)));
    }
  else
    {
      /*
       velocity_x = 0.00001;
       velocity_y = 0.00001;        
      */
      velocity_x = 0.0;
      velocity_y = 0.0;
    }

  //dest_ori is always positive
  if(orientation < dest_ori)
    {
      orientation += fabs(velocity_rotation) * elapsed_time;
    }
  else
    {
      velocity_rotation = 0;
      orientation = 0;
    }

  Vector bend_vector;
  double odometry_difference =  sqrt(pow(odometry_velocity_y, 2.) + pow(odometry_velocity_x, 2.)) * elapsed_time;

  //calculation of the odometry
  //rotation and tranlation
  if((odometry_velocity_rotation > 0.000000005 || odometry_velocity_rotation < -0.0000000005) && odometry_difference != 0.)
    {
      //recalculation of the arc
      double turned_angle = odometry_velocity_rotation * elapsed_time;

      double bend_radius = odometry_difference / fabs(turned_angle);

      if(turned_angle < 0)
        {
          bend_vector.y(bend_radius * sin(atan2(odometry_velocity_y, odometry_velocity_x) - deg2rad(90)));
          bend_vector.x(bend_radius * cos(atan2(odometry_velocity_y, odometry_velocity_x) - deg2rad(90)));
        }
      else
        {
          bend_vector.y(bend_radius * sin(atan2(odometry_velocity_y, odometry_velocity_x) + deg2rad(90)));
          bend_vector.x(bend_radius * cos(atan2(odometry_velocity_y, odometry_velocity_x) + deg2rad(90)));
        }
      bend_vector.z(0.);

      //recalculating the map for the new positions
      for(unsigned int i = 0; i < map.size(); i++)
        {//recalculating map
          Vector obstacle_rotation;
          obstacle_rotation.x(map[i].x);
          obstacle_rotation.y(map[i].y);
          obstacle_rotation.z(0.);
          obstacle_rotation -= bend_vector;
          obstacle_rotation.rotate_z(-turned_angle);
          obstacle_rotation += bend_vector;
          map[i].x = obstacle_rotation.x();
          map[i].y = obstacle_rotation.y();
        }
      //has to reset the target, since the path will be deleted
      Vector obstacle_rotation;
      obstacle_rotation.x(path[path.size() - 1]->x);
      obstacle_rotation.y(path[path.size() - 1]->y);
      obstacle_rotation -= bend_vector;
      obstacle_rotation.rotate_z(-turned_angle);
      obstacle_rotation += bend_vector;
      pathfinder->set_target(obstacle_rotation.x(), obstacle_rotation.y());
    }//rotation without translation
  else if((odometry_velocity_rotation > 0.000000005 || odometry_velocity_rotation < -0.0000000005) && odometry_difference == 0.)
    {
      for(unsigned int i = 0; i < map.size(); i++)
        {//recalculating map
          double ori = atan2(map[i].y, map[i].x);
          double radius = sqrt(pow(map[i].x,2.) + pow(map[i].y, 2.));
          map[i].x = radius * cos(ori - odometry_velocity_rotation * elapsed_time);
          map[i].y = radius * sin(ori - odometry_velocity_rotation * elapsed_time);
        }
      //has to reset the target, since the path will be deleted

      double ori = atan2(path[path.size() - 1]->y, path[path.size() - 1]->x);
      double radius = sqrt(pow(path[path.size() - 1]->x,2.) + pow(path[path.size() - 1]->y, 2.));
      pathfinder->set_target(
        radius * cos(ori + odometry_velocity_rotation * elapsed_time),
        radius * sin(ori + odometry_velocity_rotation * elapsed_time));
    }
  else if(odometry_difference != 0.)//translation without rotation
    {
      for(unsigned int i = 0; i < map.size(); i++)
        {//recalculating map
          map[i].x -= odometry_velocity_x * elapsed_time;
          map[i].y -= odometry_velocity_y * elapsed_time;
        }

      //has to reset the target, since the path will be deleted
      pathfinder->set_target(
        path[path.size() - 1]->x -  elapsed_time * odometry_velocity_x,
        path[path.size() - 1]->y -  elapsed_time * odometry_velocity_y);
    }

  pathfinder->set_obstacles(map);

  t = 0;

  path_mutex->lock();
  destroy_path();

  surface_mutex->lock();
  path = pathfinder->get_path();
  surface_mutex->unlock();

  path_mutex->unlock();

  count = (int)path.size();
}

