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

#include <plugins/navigator/path_state.h>
#include <plugins/navigator/gts/gts_obstacle.h>
#include <utils/search/astar.h>

#include <limits>
#include <cmath>

long PathState::keyCount = 0;


/** @class PathState plugins/navigator/path_state.h
 *  This class implements a state of an astar search algorithm for the pathfinder.
 *  It searchs the best path through a set of obstacles.
 *  The pathfinder generates a delaunay triangulation, with the obstacles
 *  as nodes in it. Now, the algorithm works on the triangles, represented
 *  by faces, of the triangulation. The nodes of the search tree are the
 *  faces. Every face has three edges. So the successor nodes are the
 *  faces at the edges of the current face. The points for the path are 
 *  calculated by choosing a good point on an edge. The cost is calculated
 *  by computing the distance between the path points plus a value representing
 *  the distance between two objects.
 *
 * @author Martin Liebenberg
 */
/** @var PathState::keyCount
 *  The static count for the unique keys of the states.
 */
/** @var PathState::current_point
 *  The current path point of this state.
 */
/** @var PathState::current_face
 *  The current face with the current edge and the current point.
 */
/** @var PathState::current_edge
 *  The current edge where the current point is placed.
 */
/** @var PathState::target_point
 *  The point the robot shall reach.
 */
/** @var PathState::start_point
 *  The point the robot starts from.
 */
/** @var PathState::surface
 *  The surface of the triangulation, containing all faces(triangles) of the triangulation.
 */
/** @var PathState::robot_width
 *  The width of the robot.
 */
/** @var PathState::obstacle1_radius
 *  The radius of one of two obstacles constitute a passage.
 */
/** @var PathState::obstacle2_radius
 *  The radius of one of two obstacles constitute a passage.
 */


/** Standard Constructor. */
PathState::PathState() :
    AStarState()
{}


/** Constructor.
 *  @param surface the surface with the triangulation
 *  @param current_edge the edge whereon the current point is placed
 *  @param current_point the latest point of the path
 *  @param current_face the face wich has to be detected for the next point
 *  @param target_point the point the robot has to rech
 *  @param start_point the point where the robot starts
 *  @param pastCost the cost to reach the current_point
 *  @param father the predecessor of this state
 */
PathState::PathState(GtsSurface * surface, GtsEdge * current_edge,
                     GtsPoint * current_point, GtsFace * current_face, GtsPoint * target_point,
                     GtsPoint * start_point, double pastCost, PathState * father) :
    AStarState()
{
  this->current_point = current_point;
  this->target_point = target_point;
  this->surface = surface;
  this->current_face = current_face;
  this->current_edge = current_edge;
  this->start_point = start_point;
  this->pastCost = pastCost;
  this->totalEstimatedCost = pastCost + estimate();
  this->father = father;
  this->robot_width = 0.5;
  key = -1;
}

/** Destructor. */
PathState::~PathState()
{
  //don't erase the point of the robot position
  if(current_point != start_point && !GTS_OBJECT_DESTROYED(current_point))
    {
      gts_object_destroy(GTS_OBJECT(current_point));
    }
}

/** Returns the current edge of the delaunay triangulation.
 * @return a pointer of GtsEdge to the current edge
 */
GtsEdge * PathState::getEdge()
{
  return current_edge;
}

/** Returns the current point of the path on the current_edge.
 * @return a pointer of GtsPoint to the current point
 */
GtsPoint * PathState::getPoint()
{
  return current_point;
}

/** Generates a unique key for the current state.
 *  @return the unique key as a long value
 */
long PathState::calculateKey()
{
  if(keyCount > std::numeric_limits<long>::max())
    keyCount = 0;
  key = (long)(keyCount++);
  return key;
}

/** Estimates the cost up to the goal from this state.
 *  @return the distance between the target_point and the current point as a double value
 */
double PathState::estimate()
{
  return (double) gts_point_distance(current_point, target_point);
}

/** Checks wether the current state is a goal or not.
 *  @return true, if this state is a goal, else false
 */
bool PathState::isGoal()
{
  //if the target_point is outside the surface
  //check whether the direkt line between the target
  //and the robot intersects no boundary edge of the surface
  if (current_face == NULL)
    {
      GSList* boundary_list  =  gts_surface_boundary(surface);

      do //for each boundary edge
        {
          GtsVertex* v1 = gts_vertex_new(gts_vertex_class(), current_point->x, current_point->y, 0);
          GtsVertex* v2 = gts_vertex_new(gts_vertex_class(), target_point->x, target_point->y, 0);
          GtsSegment* s = gts_segment_new(gts_segment_class(), v1, v2);

          if(current_edge != boundary_list->data
              && GTS_IN == gts_segments_are_intersecting(s, GTS_SEGMENT(boundary_list->data)))
            {
              gts_object_destroy(GTS_OBJECT(s));
              return false;
            }
          gts_object_destroy(GTS_OBJECT(s));
        }
      while((boundary_list = boundary_list->next) != NULL);

      return true;
    }
  else if(gts_point_triangle_distance(target_point, &(current_face->triangle)) < 0.001)
    {
      // it is better to check the distance between the target_point and the current triangle than to check
      // whether the target_point is in the current triangle, since the target_point might be right between two
      // triangels and then the search runs eternally
      return true;
    }
  else
    {
      return false;
    }
}

/** Calculates the schortest difference between to angles.
 *  @param angle1 one of the two angles
 *  @param angle2 one of the two angles
 */
double PathState::shortest_difference_between_two_angles(double angle1, double angle2)
{
  if(angle1 < 0)
    angle1 += M_PI * 2;

  if(angle2 < 0)
    angle2 += M_PI * 2;


  double diff1 = angle2 - angle1;

  if(diff1 < 0)
    diff1 += M_PI * 2;


  double diff2 = angle1 - angle2;

  if(diff2 < 0)
    diff2 += M_PI * 2;

  return std::min(diff1, diff2);
}


/** Calculates the next point on a given edge defined by two points.
 *  @param p1 the first point defining the edge
 *  @param p2 the second point defining the edge
 *  @param newCost a reference to a double value getting the cost to drive to the next point
 *  @return the next point of the path
 */
GtsPoint * PathState::nextPoint(GtsPoint* p1, GtsPoint* p2, double &newCost)
{
  gdouble middle_x;
  gdouble middle_y;


  gdouble x1 = p1->x;
  gdouble y1 = p1->y;

  gdouble x2 = p2->x;
  gdouble y2 = p2->y;

  //compute the middle of the edge
  middle_x = (x1 + x2) / 2.;
  middle_y = (y1 + y2) / 2.;

  //calculate a direction vector

  //vector from p1 to p2
  double vector_x = x2 - x1;
  double vector_y = y2 - y1;

  double norm1 = sqrt(pow(vector_x, 2) + pow(vector_y, 2));

  vector_x /= norm1;
  vector_y /= norm1;


  GtsPoint * next_point;

  //the point in the middle of the passage through obstacle1 and obstacle2
  GtsPoint * next_point1 = gts_point_new(gts_point_class(),
                                         middle_x + (vector_x * (obstacle1_radius - obstacle2_radius)/2),
                                         middle_y + (vector_y * (obstacle1_radius - obstacle2_radius)/2), 0);

  // the following commented out was the idea to avoid detours over the midpoints of very long edges
  // it calculates points near the obstacles to get a much shorter path

  //  GtsPoint * next_point2;
  //  GtsPoint * next_point3;
  //
  //  double vector_length;
  //
  //  //if the edge is wide then calculate a new point further left or right of the middle
  //  if(gts_point_distance(p1, p2) > robot_width * 3 + obstacle1_radius + obstacle2_radius)
  //    {
  //      vector_length = robot_width; // * 0.8;
  //
  //      //a point near by p1
  //      next_point2 = gts_point_new(gts_point_class(),
  //                                  x1 + vector_x * (vector_length + obstacle1_radius),
  //                                  y1 + vector_y * (vector_length + obstacle1_radius), 0);
  //
  //      //a point near by p2
  //      next_point3 = gts_point_new(gts_point_class(),
  //                                  x2 - vector_x * (vector_length + obstacle2_radius),
  //                                  y2 - vector_y * (vector_length + obstacle2_radius), 0);
  //    }
  //  else if((gts_point_distance(p1, p2) - obstacle1_radius - obstacle2_radius) <= robot_width * 3)
  //    {
  //      next_point2 = next_point3 = next_point1;
  //    }
  //  else //if the edge is narrow then calculate a new point near by the middle
  //    {
  //      vector_length = (gts_point_distance(p1, p2) - obstacle1_radius - obstacle2_radius) / 3;
  //
  //      double vector_add = (obstacle1_radius - obstacle2_radius) / 2;
  //
  //
  //      //a point near by the middle, placed toward to p1
  //      next_point2 = gts_point_new(gts_point_class(),
  //                                  middle_x - vector_x * (vector_length + vector_add),
  //                                  middle_y - vector_y * (vector_length + vector_add), 0);
  //
  //      //a point near by the middle, placed toward to p2
  //      next_point3 = gts_point_new(gts_point_class(),
  //                                  middle_x + vector_x * (vector_length + vector_add),
  //                                  middle_y + vector_y * (vector_length + vector_add), 0);
  //    }

  //set the middle point to the next point
  next_point = next_point1;

  //At the beginning of the path
  //the point on the next edge and the target
  //has to be nearly unidirectional.
  //  if(current_edge == NULL)
  //    {
  //      double direction_target = atan2(target_point->y, target_point->x);
  //      double direction_np1 = atan2(next_point1->y, next_point1->x);
  //      double direction_np2 = atan2(next_point2->y, next_point2->x);
  //      double direction_np3 = atan2(next_point3->y, next_point3->x);
  //
  //      double min = shortest_difference_between_two_angles(direction_target, direction_np1);
  //      double angle = shortest_difference_between_two_angles(direction_target, direction_np2);
  //
  //      if(min > angle)
  //        {
  //          min = angle;
  //          next_point = next_point2;
  //        }
  //
  //      angle = shortest_difference_between_two_angles(direction_target, direction_np3);
  //
  //      if(min > angle)
  //        {
  //          min = angle;
  //          next_point = next_point3;
  //        }
  //
  //    }
  //  else
  //    {
  //      //decide whether the middle point or another is the best by means of the cost
  //      //the best will be the returned next point
  //
  //     double minCost = pastCost + (double) gts_point_distance(next_point1, current_point);

  // I gess it's not exactly brilliant
  newCost = pastCost + (double) gts_point_distance(next_point1, current_point) * ((robot_width - 0.05) / (gts_point_distance(p1, p2) - obstacle1_radius - obstacle2_radius));
  //minCost;
  //
  //      double cost;
  //
  //      cost = pastCost + (double) gts_point_distance(next_point2, current_point);
  //
  //      if(minCost > cost)
  //        {
  //          newCost = cost;
  //          next_point = next_point2;
  //          minCost = cost;
  //        }
  //
  //
  //      cost = pastCost + (double) gts_point_distance(next_point3, current_point);
  //
  //      if(minCost > cost)
  //        {
  //          newCost = cost;
  //          next_point = next_point3;
  //        }
  //    }

  //  if(next_point1 != next_point)
  //    gts_object_destroy(GTS_OBJECT(next_point1));
  //
  //  if(next_point2 != next_point)
  //    gts_object_destroy(GTS_OBJECT(next_point2));
  //
  //  if(next_point3 != next_point)
  //    gts_object_destroy(GTS_OBJECT(next_point3));

  return next_point;
}

/** Adds the next PathState to the vector children if the robot fits through p1 and p2.
 *  @param p1 the first of two points out of the triangulation
 *  @param p2 the second of two points out of the triangulation
 *  @param next_edge the edge where on is the next point of the path
 *  @param children a vector with all successors of this state
 */
void PathState::addChild(GtsPoint * p1, GtsPoint * p2, GtsEdge * next_edge, std::vector< AStarState * > &children)
{
  if(GTS_IS_OBSTACLE(p1))
    obstacle1_radius = GTS_OBSTACLE(p1)->width / 2.;
  else
    obstacle1_radius = 0;

  if(GTS_IS_OBSTACLE(p2))
    obstacle2_radius = GTS_OBSTACLE(p2)->width / 2.;
  else
    obstacle2_radius = 0;

  double newCost;

  GtsPoint * next_point = nextPoint(p1, p2, newCost);

  GSList* edges = NULL;

  edges = g_slist_append(edges, next_edge);

  GSList* faces_list = gts_faces_from_edges (edges, surface);

  //the current edge is contained in at most two faces
  //and one of this faces is the current_face
  //which has to be removed
  //so there remains only one face, the next_face
  //or the edge is at the border of the surface
  //then there is no next_face

  GtsFace* next_face;

  int i = 0;

  while((next_face = (GtsFace *) g_slist_nth_data(faces_list, i++)) != NULL)
    {
      if(next_face == current_face)
        {
          faces_list = g_slist_remove(faces_list, next_face);
        }
    }

  if(g_slist_length(faces_list) != 0)
    {
      next_face = (GtsFace *) faces_list->data;
      children.push_back( new PathState( surface, next_edge, next_point, next_face,
                                         target_point, start_point,  newCost, this));
    }
  //if the target_point is outside the surface...
  else if(gts_point_locate(target_point, surface, NULL) == NULL)
    {
      //...next_face must be NULL
      next_face = NULL;

      children.push_back( new PathState( surface, next_edge, next_point, next_face,
                                         target_point, start_point,  newCost, this));
    }
}

/** Generates the successors of this state.
 * @return a vector with the path states of the found path
 */
std::vector< AStarState * > PathState::generateChildren()
{
  std::vector< AStarState * > children;

  //must not be destroyed, because the pointers points to objects in the surface
  GtsEdge * edge1;
  GtsEdge * edge2;
  GtsEdge * edge3;

  GtsVertex * foo;
  
  //the robot is inside the surface
  if(current_face != NULL)
    {
      gts_triangle_vertices_edges     (&(current_face->triangle),
                                       current_edge,
                                       &foo,
                                       &foo,
                                       &foo,
                                       &edge1,
                                       &edge2,
                                       &edge3);

      //at the beginning there is no current_edge
      if(current_edge == NULL)
        {
          GtsPoint * p1_1 = &((edge1->segment).v1->p);
          GtsPoint * p1_2 = &((edge1->segment).v2->p);
          addChild(p1_1, p1_2, edge1, children);
        }

      GtsPoint * p2_1 = &((edge2->segment).v1->p);
      GtsPoint * p2_2 = &((edge2->segment).v2->p);

      addChild(p2_1, p2_2, edge2, children);

      GtsPoint * p3_1 = &((edge3->segment).v1->p);
      GtsPoint * p3_2 = &((edge3->segment).v2->p);

      addChild(p3_1, p3_2, edge3, children);
    }
  else //the robot is outside the surface
    {
      GSList* boundary_list  =  gts_surface_boundary(surface);
      GSList* list = boundary_list;

      do //for each boundary edge
        {
          GtsVertex* center_v = gts_segment_midvertex(GTS_SEGMENT(list->data), gts_vertex_class());

          bool no_child = false;
          GSList* polygon_list = boundary_list;
          do
            {
              GtsVertex* v1 = gts_vertex_new(gts_vertex_class(), start_point->x, start_point->y, 0);
              GtsVertex* v2 = gts_vertex_new(gts_vertex_class(), GTS_POINT(center_v)->x, GTS_POINT(center_v)->y, 0);
              GtsSegment* s = gts_segment_new(gts_segment_class(), v1, v2);

              if(list->data != polygon_list->data
                  && GTS_IN == gts_segments_are_intersecting(s, GTS_SEGMENT(polygon_list->data)))
                {
                  gts_object_destroy(GTS_OBJECT(s));
                  no_child = true;
                  break;
                }
              gts_object_destroy(GTS_OBJECT(s));
            }
          while((polygon_list = polygon_list->next) != NULL);
          gts_object_destroy(GTS_OBJECT(center_v));
          //if the direct line between the robot_point and center_v does not intersect the triangulation -> no child is added
          if(no_child)
            {
              continue;
            }

          GtsPoint * p1 = GTS_POINT(GTS_SEGMENT(list->data)->v1);
          GtsPoint * p2 = GTS_POINT(GTS_SEGMENT(list->data)->v2);
          addChild(p1, p2, GTS_EDGE(list->data), children);
        }
      while((list = list->next) != NULL);
    }

  return children;
}
