
/***************************************************************************
 *  pathfinder.cpp - The Pathfinder of the navigator
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

#include <plugins/navigator/pathfinder.h>
#include <plugins/navigator/path_state.h>
#include <libs/utils/search/astar.h>

extern "C"
{
#include <gts.h>
}

#define STRAIGHT_AHEAD
//#define TIME

/** @class Pathfinder plugins/navigator/pathfinder.h
 *   The pathfinder of the navigator to find a path in a delaunay triangulation.
 *   
 *   @author Martin Liebenberg
 */

/** Constructor. */
Pathfinder::Pathfinder()
{
  scanning_area_width = 0.5;
  scanning_area_height = 0.5;

  robot_width = 0.5;
   
  initSurface();

  robot_point = gts_point_new(gts_point_class(), 0, 0, 0);

  target_point = gts_point_new(gts_point_class(), 0, 0, 0);
}

/** Constructor.
 * @param robot_width the width of the robot
 * @param scanning_area_width the width of the scanned area around the robot
 * @param scanning_area_height the height of the scanned area around the robot
 */
Pathfinder::Pathfinder(double robot_width, double scanning_area_width, double scanning_area_height)
{   
  this->scanning_area_width = scanning_area_width;
  this->scanning_area_height = scanning_area_height;

  this->robot_width = robot_width;
  
  initSurface();
  
  target_point = gts_point_new(gts_point_class(), 0, 0, 0);
  robot_point = gts_point_new(gts_point_class(), 0, 0, 0);
}
 
/** Destructor. */
Pathfinder::~Pathfinder()
{
  gts_object_destroy(GTS_OBJECT(target_point));
  gts_object_destroy(GTS_OBJECT(robot_point));
  gts_object_destroy(GTS_OBJECT(surface));
}
  
/** Initilizes the surface for the delaunay triangulation. */
void Pathfinder::initSurface()
{
  
  surface = gts_surface_new (gts_surface_class (),
                             gts_face_class (),
                             gts_edge_class (),
                             gts_vertex_class ());
   
  /*****************
   * Scanning Area
   *****************/
  GtsVertex* v0 = gts_vertex_new(gts_vertex_class(), -scanning_area_width / 2., -scanning_area_height / 2., 0); 
  GtsVertex* v1 = gts_vertex_new(gts_vertex_class(), -scanning_area_width / 2., +scanning_area_height / 2., 0);
  GtsVertex* v2 = gts_vertex_new(gts_vertex_class(), +scanning_area_width / 2., +scanning_area_height / 2., 0);
  GtsVertex* v3 = gts_vertex_new(gts_vertex_class(), +scanning_area_width / 2., -scanning_area_height / 2., 0);
  
  //the quadrangle around the robot
  GtsEdge*  e0 =  gts_edge_new(gts_edge_class(), v0, v1);
  GtsEdge*  e1 =  gts_edge_new(gts_edge_class(), v1, v2);
  GtsEdge*  e2 =  gts_edge_new(gts_edge_class(), v2, v3);
  GtsEdge*  e3 =  gts_edge_new(gts_edge_class(), v3, v0); 
  GtsEdge*  e4 =  gts_edge_new(gts_edge_class(), v0, v2); //the diagonal
  
  GtsFace* scanareaFace1 = gts_face_new (gts_face_class (),
                                         e1, e0, e4); 
  //the order of the edges is very important!!!!!!!!
                                                                                       
  GtsFace* scanareaFace2 = gts_face_new (gts_face_class (),
                                         e3, e2, e4);
                                               
  gts_surface_add_face (surface, scanareaFace1);
  gts_surface_add_face (surface, scanareaFace2);
}
  
/** Sets the target.
 * @param distance distance between the robot's middle and the target
 * @param direction_rad the direction to the target in radian 
 */
void Pathfinder::setTarget(double distance, double direction_rad)
{
  gts_object_destroy(GTS_OBJECT(target_point));
  target_point = gts_point_new(gts_point_class(), distance * cos(direction_rad),
                               distance * sin(direction_rad), 0);
}
  
  
/** Sets the target.
 * @param distance distance between the robot's middle and the target
 * @param direction_degree the direction to the target in degree 
 */
void Pathfinder::setTargetPolar(double distance, double direction_degree)
{
  gts_object_destroy(GTS_OBJECT(target_point));
  double rad = (direction_degree * 180) / 3.14159265;
  target_point = gts_point_new(gts_point_class(), distance * cos(rad),
                               distance * sin(rad), 0);
}
   
/** Sets the target.
 * @param x the x-coordinate of the target relative to the robot
 * @param y the y-coordinate of the target relative to the robot
 */  
void Pathfinder::setTarget_cartesian(double x, double y)
{
  gts_object_destroy(GTS_OBJECT(target_point));
  target_point = gts_point_new(gts_point_class(), x, y, 0);     
}
 
/** Calculates the delaunay triangulation for the given surface and the given obstacles.
 * @param surface the surface for the GTS delaunay function
 * @param obstacles the obstacles around the robot within the scanned area
 */
void Pathfinder::delaunay(GtsSurface * surface, std::vector< GtsObstacle *> obstacles)
{
#ifdef TIME
  GTimer * timer;
  
  
  timer = g_timer_new ();
  g_timer_start (timer);
#endif          
  
  
  
  // cout << "obstacles.size()" << obstacles.size() << endl;
  for (guint i = 0; i < obstacles.size(); i++)
    {
      GtsVertex * ver = (GtsVertex *)obstacles[i];//gts_obstacle_new(gts_obstacle_class(),
      //GTS_POINT(obstacles[i])->x, 
      //GTS_POINT(obstacles[i])->y,
      //0, obstacles[i]->width);
      gts_delaunay_add_vertex (surface, ver, NULL);
      // gts_object_destroy(GTS_OBJECT(obstacles[i]));
    }
 
#ifdef TIME  
  g_timer_stop (timer);

  gts_surface_print_stats (surface, stderr);
  fprintf (stderr, "# Triangulation time: %g s speed: %.0f vertex/s\n", 
           g_timer_elapsed (timer, NULL),
           gts_surface_vertex_number (surface)/g_timer_elapsed (timer, NULL));
#endif
} 
 
/** A kind of sign funcion.
 * @param x the funtions
 */
inline double p_sgn(double x)
{
  if(x < 0)
    return -1;
  else if(x > 0)
    return 1;
  else
    return -1;
}
  
  
/** Check whether a given point lies in a given recangle.
 * @param point_x x-coordinate of the point
 * @param point_y y-coordinate of the point
 * @param vertex1_x x-coordinate of one point of the rectangle
 * @param vertex1_y y-coordinate of one point of the rectangle
 * @param vertex2_x x-coordinate of one point of the rectangle
 * @param vertex2_y y-coordinate of one point of the rectangle
 * @param vertex3_x x-coordinate of one point of the rectangle
 * @param vertex3_y y-coordinate of one point of the rectangle
 * @param vertex4_x x-coordinate of one point of the rectangle
 * @param vertex4_y y-coordinate of one point of the rectangle
 */
bool isInRectangle(double point_x, double point_y,
                   double vertex1_x, double vertex1_y,
                   double vertex2_x, double vertex2_y,
                   double vertex3_x, double vertex3_y,
                   double vertex4_x, double vertex4_y)
{
  double determinant1 = vertex1_x*vertex2_y - vertex1_y*vertex2_x - vertex1_x*point_y + vertex1_y*point_x + vertex2_x*point_y - vertex2_y*point_x;
  double determinant2 = vertex2_x*vertex3_y - vertex2_y*vertex3_x - vertex2_x*point_y + vertex2_y*point_x + vertex3_x*point_y - vertex3_y*point_x;
  double determinant3 = vertex3_x*vertex4_y - vertex3_y*vertex4_x - vertex3_x*point_y + vertex3_y*point_x + vertex4_x*point_y - vertex4_y*point_x;
  double determinant4 = vertex4_x*vertex1_y - vertex4_y*vertex1_x - vertex4_x*point_y + vertex4_y*point_x + vertex1_x*point_y - vertex1_y*point_x;

  return (determinant1 < 0 && determinant2 < 0  && determinant3 < 0 && determinant4 < 0)
    || (determinant1 >= 0 && determinant2 >= 0  && determinant3 >= 0 && determinant4 >= 0);
}
  
/**Tests if the straight way is free.
 * It generates a rectangle whose two parallel lines are a line through
 * the starting point and one line lies a radius of the robot away from the target 
 * in the maximal distance to the start. The methode checks if the parallel to the 
 * target line lying cross section of an obstacle cuts the rectangle.
 * @return true, if the robot would not hit any obstacle on the straight way to the target,
 *                              false otherwise
 */
bool Pathfinder::test_straight_ahead()
{
  /*     ____________  start_base(width of the robot)
   *     |        S          |  start(robot)
   *     |                    |
   *     |                __|____  obstacle_base (width of the obstacle)
   *     |                    | O    obstacle
   *     |                    |
   *     |___________| target_base1 \
   *     |         T         |  target            > distance between base1 and base2 is the 
   *     |                    |                       |                                radius of the robot
   *     |___________|  target_base2/
   */
  bool test = false;   
   
  double pi = 4 * atan(1.);
   
  double start_x = robot_point->x;
  double start_y = robot_point->y;

  double target_x = target_point->x;
  double target_y = target_point->y;
   
   
  double start_base1_x;
  double start_base1_y;

  double start_base2_x;
  double start_base2_y;

  double target_base1_x;
  double target_base1_y;

  double target_base2_x;
  double target_base2_y;

  double target_back_x;
  double target_back_y;

  double obstacle_base1_x;
  double obstacle_base1_y;

  double obstacle_base2_x;
  double obstacle_base2_y;
  
  double obstacle_x;
  double obstacle_y;

  double obstacle_width;

  double m1_1;
  double m1_2;


  double tan;

  m1_1 = (target_x + start_x)/2;
  m1_2 = (target_y + start_y)/2;


  if(m1_1 == 0)
    tan = pi/2;
  else if(m1_2 == 0)
    tan = pi;
  else
    tan = atan(-m1_2 / m1_1);

  double tan2;

  if(m1_1 == 0)
    tan2 = pi;
  else if(m1_2 == 0)
    tan2 = pi/2;
  else
    tan2 = atan(m1_2 / m1_1);


  start_base1_x = p_sgn(tan) * sin(tan) * (robot_width / 2.) +  start_x;
  start_base1_y = p_sgn(tan) * cos(tan) * (robot_width / 2.) +  start_y;

  start_base2_x = p_sgn(tan) * -sin(tan) * (robot_width / 2.) +  start_x;
  start_base2_y = p_sgn(tan) * -cos(tan) * (robot_width / 2.) +  start_y;


  target_back_x =  p_sgn(tan2) * p_sgn(target_x - start_x) * sin(tan2) * (robot_width / 2.) +  target_x;

  if(target_x == start_x)
    target_back_y = p_sgn(target_y - start_y) * (robot_width / 2.) +  target_y;
  else
    target_back_y =  p_sgn(tan2) * p_sgn(target_x - start_x) * cos(tan2) * (robot_width / 2.) +  target_y;

  target_base1_x = p_sgn(tan) * sin(tan) * (robot_width / 2.) +  target_back_x;
  target_base1_y = p_sgn(tan) * cos(tan) * (robot_width / 2.) +  target_back_y;

  target_base2_x = p_sgn(tan) * -sin(tan) * (robot_width / 2.) +  target_back_x;
  target_base2_y = p_sgn(tan) * -cos(tan) * (robot_width / 2.) +  target_back_y;


  for (guint i = 0; i < map.size(); i++)
    {
      Obstacle o = map[i];
      GtsObstacle * obstacle = gts_obstacle_new(gts_obstacle_class(), o.x, o.y, 0, o.width);
   
      if(gts_point_locate(GTS_POINT(obstacle), surface,NULL) == NULL)
        {
          gts_object_destroy(GTS_OBJECT(obstacle)); 
          continue;
        }
      
      obstacle_x = GTS_POINT(obstacle)->x;
      obstacle_y = GTS_POINT(obstacle)->y;
      
       
      obstacle_width = obstacle->width;
       
      gts_object_destroy(GTS_OBJECT(obstacle)); 
        
      obstacle_base1_x = p_sgn(tan) * sin(tan) * (obstacle_width / 2.) +  obstacle_x;
      obstacle_base1_y = p_sgn(tan) * cos(tan) * (obstacle_width / 2.) +  obstacle_y;

      obstacle_base2_x = p_sgn(tan) * -sin(tan) * (obstacle_width / 2.) +  obstacle_x;
      obstacle_base2_y = p_sgn(tan) * -cos(tan) * (obstacle_width / 2.) +  obstacle_y;

   
      //it checks only if the left or the right or the middle lies within the recangle
      //this requires that the obstace is never more than twice as width like the robot
      test |= isInRectangle(obstacle_base1_x, obstacle_base1_y,
                            start_base1_x, start_base1_y,
                            target_base1_x, target_base1_y,
                            target_base2_x, target_base2_y,
                            start_base2_x, start_base2_y);
      test |= isInRectangle(obstacle_base2_x, obstacle_base2_y,
                            start_base1_x, start_base1_y,
                            target_base1_x, target_base1_y,
                            target_base2_x, target_base2_y,
                            start_base2_x, start_base2_y);
      test |= isInRectangle(obstacle_x, obstacle_y,
                            start_base1_x, start_base1_y,
                            target_base1_x, target_base1_y,
                            target_base2_x, target_base2_y,
                            start_base2_x, start_base2_y);
      if(test)
        break;
    }
    
  // std::cout << "--------------straight_ahead------: " << !test << std::endl;
   
  return !test;
}

/** Pushes a vertex out of a GTS fifo.
 *  It is needed for the gts_surface_foreach_edge function of GTS.
 * @param vertex the pushed vertex
 * @param fifo the GTS fifo with the vertexes
 */
void Pathfinder::getVertexes(GtsVertex *vertex, GtsFifo * fifo)
{
  gts_fifo_push(fifo, GTS_OBJECT(vertex));
}
  
/** Returns the nearest point by the robot.
 * @return the nearest point
 */
GtsPoint * Pathfinder::getNearestPoint()
{
  double length = 1000000;
  GtsPoint * point = NULL;
  GtsFifo * vertexes = gts_fifo_new();
  gts_surface_foreach_vertex(surface, (GtsFunc) getVertexes, vertexes);
  while(!gts_fifo_is_empty(vertexes))
    {
      GtsVertex * v = (GtsVertex *)gts_fifo_pop(vertexes);
      double distance = gts_point_distance(GTS_POINT(v), robot_point);
      if(distance < length)
        {
          length = distance;
          point = GTS_POINT(v);
        }
    }
  gts_fifo_destroy(vertexes);
  return point;
}

/** Returns the nearest obstacle of type GtsObstacle.
 * @return the nearest obstacle of type GtsObstacle
 */
GtsObstacle * Pathfinder::getNearestGtsObstacle()
{
  double length = 1000000;
  GtsObstacle * obstacle = NULL;
  GtsFifo * vertexes = gts_fifo_new();

  
  gts_surface_foreach_vertex(surface, (GtsFunc) getVertexes, vertexes);
  
  while(!gts_fifo_is_empty(vertexes))
    {
      GtsVertex * v = (GtsVertex *)gts_fifo_pop(vertexes);

      if(GTS_IS_OBSTACLE(v))
        {
          double distance = gts_point_distance(GTS_POINT(v), robot_point);

          if(distance - robot_width/2. - GTS_OBSTACLE(v)->width/2. < length)
            {
              length = distance - robot_width/2. - GTS_OBSTACLE(v)->width/2.;
              obstacle = GTS_OBSTACLE(v);
            }
        }
    }
  gts_fifo_destroy(vertexes);
   
  return obstacle;
}
 
/** Sets the recognized obstacles to the pathfinder.
 * @param obstacles a vector of obstacles
 */
void Pathfinder::setObstacles(std::vector< Obstacle > obstacles)
{
  map = obstacles;
}

/** Adds an obstacle to the pathfinder.
 * @param obstacle an obstacle
 */
void Pathfinder::addObstacle(Obstacle obstacle)
{
  map.push_back(obstacle);
}
 
/** Regains the Obstacles from the map if every obstacle
 *   is destoyed by destoying the surface.
 */ 
void Pathfinder::regainObstacles()
{
  /* Shut up!
  std::cout << "===============regain" << std::endl;
  for(unsigned int i = 0; i < obstacles.size(); i++)
    {
      //  gts_object_destroy(GTS_OBJECT(obstacles[i]));
      std::cout << "obst " << obstacles[i] << std::endl;
      std::cout << "class " << (obstacles[i] == NULL) << std::endl;
    }
  */
  this->obstacles.clear();
  
  //generate a vector of obstacles in the scanning_area
  //das braucht man nur für die Simulation, da man sonst nur 
  //Hindernisse innerhalb des scann-Bereichs hat 
  for(unsigned int i = 0; i < map.size(); i++)
    {
      Obstacle o = map[i];
      if(!out_of_area(o.x, o.y))//zu schwach, Breite mitberücksichtigen, da sonst, die Hindernisse
        //erst ab der Mitte im Feld auftauchen; ist aber egal, da wir ja eh nicht wissen
        //was hinter dem scan bereich liegt
        //besser gts_point_locate
        {
          GtsObstacle * obs = gts_obstacle_new(gts_obstacle_class(), o.x, o.y, 0, o.width);

          this->obstacles.push_back(obs);
        }
    }
}
 
/** Checks if a point given by its coordinates is within the scan area.
 * @param x the x-coordinate of the point
 * @param y the y-coordinate of the point
 * @return true if the point is within the scan area, false otherwise
 */
//poss. replace by gts_point_locate(GTS_POINT(p), surface,NULL)
bool Pathfinder::out_of_area(double x, double y)
{
  return (   x < -scanning_area_width  / 2 && x > scanning_area_width  / 2
             && y < -scanning_area_height / 2 && y > scanning_area_height / 2);
}
 
/** Calculates the delaunay triangulation.
 */
void Pathfinder::calculateDelaunay()
{ 
  for(unsigned int i = 0; i < obstacles.size(); i++)
    {
      gts_object_destroy(GTS_OBJECT(obstacles[i]));
    }
  gts_object_destroy(GTS_OBJECT(surface));
  
  initSurface();
  
  regainObstacles(); 
  
  delaunay(surface, obstacles);
}
 
/** Returns the surface of the delaunay triangulation.
 * @return the surface of the delaunay triangulation
 */
GtsSurface * Pathfinder::getSurface()
{
  return surface;
}
 
/** Returns the position of the robot.
 * @return the position of the robot as a GtsPoint
 */
GtsPoint * Pathfinder::getRobotPoint()
{
  return robot_point;
}
 
/** Returns the current target.
 * @return the target as GtsPoint
 */
GtsPoint * Pathfinder::getTargetPoint()
{
  return target_point;
}
 
/** Sets the target.
 * @param target_point the target as GtsPoint
 */
void Pathfinder::setTargetPoint(GtsPoint * target_point)
{
  gts_object_destroy(GTS_OBJECT(this->target_point));
  this->target_point = target_point;
}
 
 
/** Returns the path.
 * @return a vector of the path points as GtsPoints
 */
std::vector< GtsPoint * > Pathfinder::getPath()
{
  GTimer * timer;
  timer = g_timer_new ();
  g_timer_start (timer);
  
  //calculate the delaunay triangulation
  calculateDelaunay();
  
  std::vector< GtsPoint * > solution_path;
  
  //if there is no obstacle on the way, then the robot can drive ahead
  if(test_straight_ahead())
    {
      // std::cerr << "Target Point " << target_point << std::endl;
      // std::cerr << "Target Point " << target_point << std::endl;
      // std::cerr << "robot Point " << robot_point << std::endl;
      solution_path.push_back(gts_point_new(gts_point_class(), robot_point->x, robot_point->y, 0));
      solution_path.push_back(gts_point_new(gts_point_class(), target_point->x, target_point->y, 0));
        
      // std::cerr << "new Target Point " << solution_path[1] << std::endl;
      // std::cerr << "new robot Point " << solution_path[0] << std::endl;
      // cout << "solution_path " << solution_path.size() << endl;
      g_timer_stop (timer);
      // cout << "Time for getPath(): " << g_timer_elapsed (timer, NULL) << endl;
    }
  else 
    {
      // construct a new astar 
      // cout << "construct a new astar" << endl;
      AStar * astar = new AStar();
      // cout << "Initialized AStar" << endl;

      // GtsPoint * robot_point_copy = gts_point_new(gts_point_class(), robot_point->x, robot_point->y, 0);
      // construct the initial situation and tell the goal somehow ;-)
      PathState * initialState = new PathState(surface, NULL, 
                                               robot_point, 
                                               gts_point_locate (robot_point, surface, NULL),  target_point, robot_point, 0, 0);
    
      // cout << "Initialized Initial State" << endl;

      // solve the problem 
  
      std::vector< AStarState * > solution = astar->solve( initialState );

  
      // std::cerr << "Target Point " << target_point << std::endl;
      // std::cerr << "robot Point " << robot_point << std::endl;
      // look at the solution
      for (unsigned int i = 0; i < solution.size(); i++ )
        {
          PathState * state = (PathState *)(solution[i]);
          GtsPoint * p = state->getPoint();
          // std::cerr << "getPoint " << p << std::endl;
          solution_path.push_back(gts_point_new(gts_point_class(), p->x, p->y, 0));
          //std::cerr << "new getPoint " << solution_path[solution_path.size() -1] << std::endl;
        }
     
      // and delete the solution afterwards!!!!
      for (unsigned int j = 0; j < solution.size(); j++ )
        {
          PathState * state = (PathState *)(solution[j]);
          // std::cout << "prepar destroying " << state->key << std::endl;
          delete state;
        }
    
      g_timer_stop (timer);
      // cout << "Time for getPath(): " << g_timer_elapsed (timer, NULL) << endl;
      solution_path.push_back(gts_point_new(gts_point_class(), target_point->x, target_point->y, 0));
      // std::cerr << "new getPoint " << solution_path[solution_path.size() -1] << std::endl;
     
      // solution_path.push_back(target_point);
    
      // cout << "solution_path " << solution_path.size() << endl;
      delete astar;
    }
  g_timer_destroy(timer);
  return solution_path;
}
