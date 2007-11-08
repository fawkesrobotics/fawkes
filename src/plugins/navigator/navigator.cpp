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

#include <plugins/navigator/navigator.h>

#include <plugins/navigator/fuzzy/potential_field_controller.h>
#include <plugins/navigator/pathfinder.h>

#include <plugins/navigator/fuzzy/smoothing_controller.h>

#include <plugins/navigator/libnavi/npoint.h>
#include <plugins/navigator/libnavi/nline.h>
#include <utils/math/binomial_coefficient.h>

#include <cmath>

#define FUZZY_POTENTIAL_FIELD
#define FUZZY_SMOOTHING

#define N_DELAY 100000

/** @class Navigator plugins/navigator/navigator.h
 *   The navigator of fawkes.
 *   
 *   @author Martin Liebenberg
 */

/** Constructor */
Navigator::Navigator()
{
  //this is the area within the objects are seen
  scanning_area_width = 5.; // m
  scanning_area_height = 5.; //m

  //the robot's width in m
  robot_width = 0.5;

  //the desired velocity of the robot
  velocity = 0; // m/s

  //the velocities in the x and y direction
  velocity_x = 0;
  velocity_y = 0;

  //the velocity of the rotation of the robot
  velocity_rotation = 0;
  orientation = 0;


  //the index of the formula of the bezier curve
  t = 0;


  //the pathfinder object to get the pathes
  pathfinder = new Pathfinder(robot_width, scanning_area_width, scanning_area_height);

  //path needs not to be NULL
  path = pathfinder->getPath();

  count = (int)path.size();


  newDirection = false;


  time_emitter = g_timer_new();

}

/** Destructor */
Navigator::~Navigator()
{
  delete pathfinder;
}

/** Pushes an edge out of a GTS fifo.
 *  It is needed for the gts_surface_foreach_edge function of GTS.
 * @param edge the pushed edge
 * @param fifo the GTS fifo with the edges
 */
void Navigator::getEdges(GtsEdge *edge, GtsFifo * fifo)
{
  gts_fifo_push(fifo, edge);
}

/** Pushes a vertex out of a GTS fifo.
 *  It is needed for the gts_surface_foreach_edge function of GTS.
 * @param vertex the pushed vertex
 * @param fifo the GTS fifo with the vertexes
 */
void Navigator::getVertexes(GtsVertex *vertex, GtsFifo * fifo)
{
  gts_fifo_push(fifo, GTS_OBJECT(vertex));
}

/** Gets the points of the surface of the triangulation.
 * @return a list of all points of the surface.
 */
std::list<NPoint *> *Navigator::get_surface_points()
{
  std::list<NPoint *> *p_list = new std::list<NPoint *>;
  GtsSurface *surface = pathfinder->getSurface();
  GtsFifo *vertexes = gts_fifo_new();

  gts_surface_foreach_vertex(surface, (GtsFunc) getVertexes, vertexes);

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
  gts_fifo_destroy(vertexes);
  return p_list;
}

/** Gets the obstacles within the scan area.
 * @return a list of all obstacles.
 */
std::list<Obstacle *> *Navigator::get_obstacles()
{
  std::list<Obstacle *> *o_list = new std::list<Obstacle *>;
  GtsSurface *surface = pathfinder->getSurface();
  GtsFifo *vertexes = gts_fifo_new();

  gts_surface_foreach_vertex(surface, (GtsFunc) getVertexes, vertexes);

  while(!gts_fifo_is_empty(vertexes))
    {
      GtsVertex * v = (GtsVertex *)gts_fifo_pop(vertexes);
      if(GTS_IS_OBSTACLE(v))
        {
          Obstacle *o = new Obstacle(GTS_OBSTACLE(v)->width, GTS_POINT (v)->x, GTS_POINT (v)->y, 0);
          o_list->push_back(o);
        }
    }
  gts_fifo_destroy(vertexes);
  return o_list;
}

/** Gets the points of the path.
 * @return a list of all points of the path.
 */
std::list<NPoint *> *Navigator::get_path_points()
{
  std::list<NPoint *> *p_list = new std::list<NPoint *>;
  for(unsigned int i = 0; i < path.size(); i++)
    {
      NPoint *p = new NPoint();
      p->x = path[i]->x;
      p->y = path[i]->y;
      p_list->push_back(p);
    }
  return p_list;
}


/** Gets the lines of the surface of the triangulation.
 * @return a list of all lines of the surface.
 */
std::list<NLine *> *Navigator::get_surface_lines()
{
  std::list<NLine *> *lines = new std::list<NLine *>;
  GtsSurface *surface = pathfinder->getSurface();
  GtsFifo *edges = gts_fifo_new ();
  gts_surface_foreach_edge(surface, (GtsFunc) getEdges, edges);

  while(!gts_fifo_is_empty(edges))
    {
      GtsEdge * e = (GtsEdge * )gts_fifo_pop(edges);
      NPoint *p1 = new NPoint();
      NPoint *p2 = new NPoint();

      GtsVertex * v1 = GTS_SEGMENT (e)->v1;
      if(GTS_IS_OBSTACLE(v1))
        {
          Obstacle *o = new Obstacle(GTS_OBSTACLE(v1)->width, GTS_POINT (v1)->x, GTS_POINT (v1)->y, 0);
          p1 = o;
        }
       else
        {
        	p1->x = GTS_POINT (v1)->x;
        	p1->y = GTS_POINT (v1)->y;
        }
        
      GtsVertex * v2 = GTS_SEGMENT (e)->v2;
      if(GTS_IS_OBSTACLE(v2))
        {
          Obstacle *o = new Obstacle(GTS_OBSTACLE(v2)->width, GTS_POINT (v2)->x, GTS_POINT (v2)->y, 0);
          p2 = o;
        }
       else
        {
        	p2->x = GTS_POINT (v2)->x;
        	p2->y = GTS_POINT (v2)->y;
        }
      //NPoint p1(GTS_POINT (v1)->x, GTS_POINT (v1)->y);
      //GtsVertex * v2 = GTS_SEGMENT (e)->v2;
     // NPoint p2(GTS_POINT (v2)->x, GTS_POINT (v2)->y);

      lines->push_back(new NLine(p1, p2));
  	delete p1;
  	delete p2;
    }

  gts_fifo_destroy(edges);
  return lines;
}

/** Sets the recognized obstacles to the navigator.
 * @param map a vector of obstacles
 */
void Navigator::setObstacles(std::vector< Obstacle > map)
{
  this->map = map;
  pathfinder->setObstacles(map);
}

/** Adds an obstacle to the pathfinder.
 * @param obstacle an obstacle
 */
void Navigator::add_obstacle(Obstacle obstacle)
{
  map.push_back(obstacle);
  pathfinder->addObstacle(obstacle);
}

/** Sets the velocity of the robot.
 * @param velocity the velocity value
 */
void Navigator::setVelocity(double velocity)
{
  this->velocity = velocity;
}

/** Sets the velocity of the rotation of the robot.
 * @param velocity_rotation the velocity of the rotation
 */
void Navigator::setVelocityRotation(double velocity_rotation)
{
  this->velocity_rotation = velocity_rotation;
}

/** Returns the velocity of the robot.
 * @return the velocity of the robot
 */
double Navigator::getVelocity()
{
  return velocity;
}

/** Returns the velocity of the robot in the x-direction.
 * @return the velocity in the x-direction
 */
double Navigator::getVelocityX()
{
  return velocity_x;
}

/** Returns the velocity of the robot in the y-direction.
 * @return the velocity in the y-direction
 */
double Navigator::getVelocityY()
{
  return velocity_y;
}


/** Returns the orientation of the robot.
 * @return the orientation of the robot
 */
double Navigator::getOrientation()
{
  return orientation;
}

/** Sets a list of targets for a route.
 * @param route a vector of target-points
 */
void Navigator::setRoute(std::vector<GtsPoint*> route)
{
  if(route.size() > 0)
    {
      this->route = route;
      pathfinder->setTargetPoint(gts_point_new(gts_point_class(), route.front()->x,
                                 route.front()->y, 0));
      gts_object_destroy(GTS_OBJECT(path[path.size() - 1]));
      path[path.size() - 1] = gts_point_new(gts_point_class(), route.front()->x,
                                            route.front()->y, 0);
    }
}

/** Returns the current route of the navigator.
 * @return a vector of target points
 */
std::vector<GtsPoint *> Navigator::getRoute()
{
  return route;
}

/** Sets the time which elapsed since the last calling of the main Loop.
 * Deprecated.
 * @param elapsedTime 
 */
void Navigator::setElapsedTime(double elapsedTime)
{
  elapsed_time = elapsedTime;
}

/** Deprecated.
 * @return 
 */
double Navigator::getElapsedTime()
{
  return elapsed_time;
}

/** Deprecated.
 * @return
 */
int Navigator::getCount()
{
  return count;
}

/** Returns the surface of the navigator.
 * @return the surface
 */
GtsSurface * Navigator::getSurface()
{
  return pathfinder->getSurface();
}

/** Returns the nearest obstacle by the robot.
 * @return the nearest obstacle
 */
GtsObstacle * Navigator::nearestObstacle()
{
  return pathfinder->getNearestGtsObstacle();
}

/** Returns the current Path of the navigator.
 * @return vector of points of the path
 */
std::vector< GtsPoint * > Navigator::getPath()
{
  return path;
}

/** Returns the target point of the navigator.
 * @return the target point
 */
NPoint* Navigator::getTargetPoint()
{
  return new NPoint(pathfinder->getTargetPoint()->x, pathfinder->getTargetPoint()->y);
}

/** Returns the robot position.
 * @return point with the position of the robot; default 0, 0
 */
GtsPoint * Navigator::getRobotPoint()
{
  return pathfinder->getRobotPoint();
}

/* Binomial coefficient for the bezier curve.
 * @param n upper value
 * @param k lower value
 * @return the binomial coefficient of n and k
 *
int Navigator::binomialCoefficient(int n, int k)
{
  int result;
  if(k == 0)
    return 1;
  if(2 * k > n)
    result = binomialCoefficient(n, n - k);
  else
    {
      result = n;
      for(int i = 2; i <= k; i++)
        {
          result = result * ((n + 1 - i) / i);
        }
    }
  return result;
}
 
*/

/** Bernstein polynomial for the bezier curve.
 * @param i
 * @param n
 * @param t
 * @return the value of the bernstein polynomial
 */
double Navigator::bernstein(unsigned int i, unsigned int n, double t)
{
  return BinomialCoefficient::binoc(n, i) * pow(t, (int)i) * pow(1. - t, (int)(n - i));
}


/** Auxilliary function for calculating the velocities.
 * @param t
 * @return s
 */
double Navigator::s(double t)
{
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
void Navigator::goTo_cartesian(double x, double y)
{
  goTo_cartesian(x, y, 0);
}

/** Sets a target and a velocity.
 *  The navigator will search a path and will drive along.
 * @param x the x-coordinate of the target
 * @param y the y-coordinate of the target
 * @param velocity the velocity of the robot
 */
void Navigator::goTo_cartesian(double x, double y, double velocity)
{
  step_x = 0;
  step_y = 0;
  t = 0;


  this->velocity = velocity;

  destroy_path();
  // std::cout << "path.size() " << path.size() << std::endl;
  for(unsigned int i = 0; i < path.size(); i++)
    {
      //   std::cout << "path remains" << path[i]->x << ", " << path[i]->y << std::endl;
    }
  pathfinder->setTarget_cartesian(x, y);
  path = pathfinder->getPath();


  if(path[1]->x != 0)
    current_degree = atan2(path[1]->y , path[1]->x);
  else
    current_degree = 0;

  newDirection = true;

}

/** Destroys the current path.
 */
void Navigator::destroy_path()
{

  for(unsigned int i = 0; i < path.size(); i++)
    {
      //  std::cerr <<path.size() << " destroy_path " << i << std::endl;
      // if(pathfinder->getRobotPoint() != path[i] ||
      //   gts_point_distance(path[0], path[path.size() -1]) != 0)
      //don't destroy the robot position
      //and the target will be destroyed by the pathfinder in the setTarget Methods
      // if(i != path.size() - 1 && i != 0)
      {
        //       std::cerr << "destroying path" << path[i] << std::endl;
        gts_object_destroy(GTS_OBJECT(path[i]));
      }
    }
  path.clear();
}


/** The main loop in which the main calculations for the valocities are placed.
 * It is called from the navigator thread loop.
 */
void Navigator::mainLoop()
{
  //while(true)
  {
    usleep(N_DELAY);
    double current_time = g_timer_elapsed( time_emitter, NULL );
    elapsed_time = current_time - last_time;
    last_time = current_time;

    step_x = elapsed_time * velocity_x;
    step_y = elapsed_time * velocity_y;

    /*
    std::cout << "------------------------------------------------route.size() " << route.size()<< std::endl;
       
    std::cout << "-------elapsed_time " << elapsed_time << std::endl;
    std::cout << "-------velocity_y " << velocity_y << std::endl;
    std::cout << "-------velocity_x " << velocity_x << std::endl;
    std::cout << "-------step_x " << step_x << std::endl;
    std::cout << "-------step_y " << step_y << std::endl;
    */


    double sum_x = 0;
    double sum_y = 0;

    // std::cout << "-------path.size() " << path.size() << std::endl;

    /****************************
     *  Calculating the Gradient
     ***************************/
    double div_x;
    double div_y;
    //mit step die Berechnung der Zunahme von t durchfuehren
    double delta_s = sqrt(pow(step_x, 2.) + pow(step_y, 2.));
    double f1 = 1/s(t);
    //   std::cout << "-------f1 " << f1 << std::endl;
    double f2 = 1/s(t + delta_s * f1);
    //  std::cout << "-------f2 " << f2 << std::endl;
    double f3 = 1/s(t + delta_s * f2);
    //   std::cout << "-------f3 " << f3 << std::endl;
    double f4 = 1/s(t + delta_s * f3);
    //   std::cout << "-------f4 " << f4 << std::endl;

    double t_Tn_Sn;

    if(count > 1)
      t_Tn_Sn = (delta_s/(count-1)) * (f1 + 2*f2 + 2*f3 + f4) / 6;
    else
      t_Tn_Sn = (delta_s) * (f1 + 2*f2 + 2*f3 + f4) / 6;

    t = t + t_Tn_Sn;
    /*
    std::cout << "-------t " << t << std::endl;   
    std::cout << "-------t_Tn_Sn " << t_Tn_Sn << std::endl;
    std::cout << "-------delta_s " << delta_s << std::endl;
    */
    /***************************
     * 
     * Wenn ein Pfadabschnitt abgefahren wurde, werden fuer den naechsten
     * Abschnitt wichtige Parameter neu gesetzt
     * 
     *****************************/
    if(t > 1)
      {
        t= 0;

        //      std::cout << "-------next Path-------------------------------------------------" << std::endl;
        step_x = 0;
        step_y = 0;
        newDirection = true;


        //stop at the end of the path
        if(path.size() == 2) //also es gibt nur ein Stück zwischen zwei Punkten
          {
            if(route.size() != 0)  //die route wird abgefahren
              {
                if(route.size() == 1) //wir befinden uns auf dem letzten Stück der route
                  {
                    velocity = 0;
                    std::cout << "-------STOP-------------------------------------------------" << std::endl;
                    gts_object_destroy(GTS_OBJECT(route[0]));
                    route.clear();
                  }
                else //wenn wir irgendwo auf der route sind, nur nicht am Ende
                  {
                    gts_object_destroy(GTS_OBJECT(route[0]));
                    route.erase(route.begin());
                    GtsPoint *p = route.front();
                    pathfinder->setTargetPoint(gts_point_new(gts_point_class(), p->x,
                                               p->y, 0));
                    gts_object_destroy(GTS_OBJECT(path[path.size() - 1]));
                    path[path.size() - 1] = gts_point_new(gts_point_class(), p->x, p->y, 0);
                    /*
                             std::cout << "route point " << p->x << ", " << p->y << std::endl;
                             GtsPoint *p1 = pathfinder->getTargetPoint();
                             std::cout << "target point " << p1->x << ", " << p1->y << std::endl;
                    */
                  }
              }
            else
              velocity = 0;
          }
      }
    /*
    std::cout << "traget-x" << pathfinder->getTargetPoint()->x << std::endl;
    std::cout << "traget-y" << pathfinder->getTargetPoint()->y << std::endl;
    */
    //bezier Kurve fuer Pfadabschnitt berechnen
    for(int k = 0; k < count - 1; k++)
      {//Berechnung der Ableitung  an der Stelle t
        if(k + 1 < (int)path.size() && k < (int)path.size())
          {
            sum_x += (path[k + 1]->x - path[k]->x) * bernstein(k, count - 2, t);
            sum_y += (path[k + 1]->y - path[k]->y) * bernstein(k, count - 2, t);
            //       std::cout << "-------sum_x " << sum_x << std::endl;
          }

      }
    //eigendlich muss die Summe noch mit (count-1) multipliziert werden
    //spielt aber keine Rolle, da ich ja nur die Richtung brauche
    div_x = sum_x;// * (count - 1);
    div_y = sum_y;// * (count - 1);
    /*
      std::cout << "-------path.size(): " << path.size() << std::endl;

      for(int k = 0; k < count; k++)
        {
          std::cout << "-------point["<<k<<"]_x: " << path[k]->x << std::endl;
          std::cout << "-------point["<<k<<"]_y: " << path[k]->y << std::endl;
        }
      std::cout << "-------div_x " << div_x << std::endl;
      std::cout << "-------div_y " << div_y << std::endl;
    */
    if(sqrt(pow(div_x, 2.) + pow(div_y, 2.)) != 0)
      {

#ifdef FUZZY_POTENTIAL_FIELD

        GtsObstacle * nearestObs = pathfinder->getNearestGtsObstacle();

        PotentialFieldController *controller = new PotentialFieldController(robot_width); //muss member werden
        double force;
        if(nearestObs != NULL)
          {
            force = controller->control(gts_point_distance(GTS_POINT (nearestObs), pathfinder->getRobotPoint())
                                        - robot_width/2. - nearestObs->width/2.);
       //     std::cout << "-------distance  "  << gts_point_distance(GTS_POINT (nearestObs), pathfinder->getRobotPoint()) - robot_width/2. - nearestObs->width/2. << std::endl;
      //      std::cout << "-------force " << force << std::endl;
            div_x += (pathfinder->getRobotPoint()->x - GTS_POINT (nearestObs)->x) * force;
            div_y += (pathfinder->getRobotPoint()->y - GTS_POINT (nearestObs)->y) * force;
            //nicht normiert
          }
        delete controller;
#endif //FUZZY_POTENTIAL_FIELD
#ifdef FUZZY_SMOOTHING

        SmoothingController cont;
        double grade = 0;

        if(div_x != 0)
          current_degree = atan2(div_y, div_x);
        else
          current_degree = 0;//müsste je nach dem Vorzeichen von div_y 90° oder -90° sein

        //      std::cout << "-------current_degree " << current_degree << std::endl;
        if(current_degree < 0)
          current_degree += M_PI * 2;

        //     std::cout << "-------last_degree" << last_degree << std::endl;
        //     std::cout << "-------current_degree " << current_degree << std::endl;
        double difference = last_degree - current_degree;



        //     std::cout << "-------difference " << difference << std::endl;
        if(difference > M_PI)
          difference = -(M_PI * 2 - difference);
        if(difference < -M_PI)
          difference = -(-M_PI *2 - difference);
        //      std::cout << "-------cont " << &(cont) << std::endl;

        //     std::cout << "-------difference " << difference << std::endl;

        if(!newDirection)
          grade = cont.control(fabs(difference));
        else
          newDirection = false;

        //    std::cout << "-------difference " << difference << std::endl;

        //      std::cout << "-------grade " << grade << std::endl;
        //grade = 1;
        div_x =  cos(current_degree + difference * grade );
        div_y =  sin(current_degree + difference * grade);
        last_degree = current_degree + difference * grade ;
        //      std::cout << "------- div_x " << div_x  << std::endl;
        //      std::cout << "------- div_y " << div_y << std::endl;
#endif //FUZZY_SMOOTHING

        //set the velocity
        velocity_x = (div_x * velocity) / sqrt((pow(div_x, 2.) + pow(div_y, 2.)));
        velocity_y = (div_y * velocity) / sqrt((pow(div_x, 2.) + pow(div_y, 2.)));
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


    /*
       std::cout << "-------velocity_x " << velocity_x << std::endl;
       std::cout << "-------velocity_y " << velocity_y << std::endl;
       std::cout << "-------velocity_rotation " << velocity_rotation << std::endl;
       std::cout << "-------velocity_rotation * elapsed_time " << velocity_rotation * elapsed_time << std::endl;
    */
    orientation += velocity_rotation * elapsed_time;
/*
    for(unsigned int i = 0; i < path.size(); i++)
      {
        //       std::cerr << "manipulate path " << path[i] << std::endl;
        path[i]->x -=  velocity_x * (elapsed_time);
        path[i]->y -=  velocity_y * (elapsed_time);
        if(velocity_rotation != 0)
          {
            double ori = atan2(path[i]->y, path[i]->x);
            double radius = sqrt(pow(path[i]->x, 2) + pow(path[i]->y, 2));
            path[i]->x = radius * cos(ori + velocity_rotation * elapsed_time);
            path[i]->y = radius * sin(ori + velocity_rotation * elapsed_time);
          }
      }*/
      
    //has to reset the target, since the path will be deleted
    pathfinder->setTargetPoint(gts_point_new(gts_point_class(),
                               path[path.size() - 1]->x -  velocity_x * (elapsed_time), 
                               path[path.size() - 1]->y -  velocity_y * (elapsed_time), 0));

    for(unsigned int i = 0; i < route.size(); i++)
      {
        route[i]->x -=  velocity_x * (elapsed_time);
        route[i]->y -=  velocity_y * (elapsed_time);
        if(velocity_rotation != 0)
          {
            double ori = atan2(route[i]->y, route[i]->x);
            double radius = sqrt(pow(route[i]->x,2) + pow(route[i]->y, 2));
            route[i]->x = radius * cos(ori + velocity_rotation * elapsed_time);
            route[i]->y = radius * sin(ori + velocity_rotation * elapsed_time);
          }
      }

    //recalculating the map for the new positions
    for(unsigned int i = 0; i < map.size(); i++)
      {//recalculating map
        map[i].x -= velocity_x * (elapsed_time);
        map[i].y -= velocity_y * (elapsed_time);
        if(velocity_rotation != 0)
          {
            double ori = atan2(map[i].y, map[i].x);
            double radius = sqrt(pow(map[i].x,2) + pow(map[i].y, 2));
            map[i].x = radius * cos(ori + velocity_rotation * elapsed_time);
            map[i].y = radius * sin(ori + velocity_rotation * elapsed_time);
          }

        //       std::cout << "------- map[" << i << "].x : " << map[i].x  << std::endl;
        //      std::cout << "------- map[" << i << "].y : " << map[i].y << std::endl;
      }

    pathfinder->setObstacles(map);

    t = 0;
    destroy_path();
    path = pathfinder->getPath();

    count = (int)path.size();
    /*
      if(path[1]->x != 0)
      current_degree = atan2(path[1]->y , path[1]->x);
      else
      current_degree = 0;
    */
  }
}

