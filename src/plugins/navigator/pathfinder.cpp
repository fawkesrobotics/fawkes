
/***************************************************************************
 *  pathfinder.cpp - The Pathfinder of the navigator
 *
 *  Generated: Tue Jun 05 13:50:17 2007
 *  Copyright  2007  Martin Liebenberg
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include <plugins/navigator/pathfinder.h>
#include <plugins/navigator/path_state.h>
#include <libs/utils/search/astar.h>
#include <core/threading/mutex.h>

#include <set>
#include <map>

extern "C"
  {
#include <gts.h>

  }

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

  init_surface();

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
  //the minimum distance between the robot and an obstacle
  minimum_distance = 0.1;

  init_surface();

  target_point = gts_point_new(gts_point_class(), 0, 0, 0);
  robot_point = gts_point_new(gts_point_class(), 0, 0, 0);

  surface_mutex = new Mutex();
  start_face = NULL;
}

/** Destructor. */
Pathfinder::~Pathfinder()
{
  gts_object_destroy(GTS_OBJECT(target_point));
  gts_object_destroy(GTS_OBJECT(robot_point));
  gts_object_destroy(GTS_OBJECT(surface));

  delete surface_mutex;
}

/** Initilizes the surface for the delaunay triangulation. */
void Pathfinder::init_surface()
{
  surface = gts_surface_new (gts_surface_class (),
                             gts_face_class (),
                             gts_edge_class (),
                             gts_vertex_class ());

  v0 = gts_vertex_new(gts_vertex_class(), -scanning_area_width / 2., -scanning_area_height / 2., 0);
  v1 = gts_vertex_new(gts_vertex_class(), -scanning_area_width / 2., +scanning_area_height / 2., 0);
  v2 = gts_vertex_new(gts_vertex_class(), +scanning_area_width / 2., +scanning_area_height / 2., 0);
  v3 = gts_vertex_new(gts_vertex_class(), +scanning_area_width / 2., -scanning_area_height / 2., 0);

  //the quadrangle around the robot
  e0 =  gts_edge_new(gts_edge_class(), v0, v1);
  e1 =  gts_edge_new(gts_edge_class(), v1, v2);
  e2 =  gts_edge_new(gts_edge_class(), v2, v3);
  e3 =  gts_edge_new(gts_edge_class(), v3, v0);
  e4 =  gts_edge_new(gts_edge_class(), v0, v2); //the diagonal

  GtsFace* scanareaFace1 = gts_face_new (gts_face_class (),
                                         e1, e0, e4);
  //the order of the edges is important to avoid holes

  GtsFace* scanareaFace2 = gts_face_new (gts_face_class (),
                                         e3, e2, e4);

  gts_surface_add_face (surface, scanareaFace1);
  gts_surface_add_face (surface, scanareaFace2);
}

/** Finds a bisection of the angle between the two bondary edges of the current_vertex
 * and save a new vertex outside of the surface at this bisection to the map.
 * Needed by delaunay().
 * @param current_vertex the vertex of where the bisection is computed
 * @param map the map where the vertexes representing the bisections are stored
 * @param concave true if the surface is concave
 */
void Pathfinder::find_bisector(GtsVertex* current_vertex, std::map<GtsVertex*, GtsVertex*>* map)
{
  GtsEdge* e1 = NULL;
  GtsEdge* e2 = NULL;

  GSList* edge_list = current_vertex->segments;
  do
    {
      if(gts_edge_is_boundary(GTS_EDGE(edge_list->data), surface))
        {
          if(e1 == NULL)
            {
              e1 = GTS_EDGE(edge_list->data);
            }
          else
            {
              e2 = GTS_EDGE(edge_list->data);
            }
        }
    }
  while((edge_list = edge_list->next) != NULL);

  float angle1 = 0;
  float angle2 = 0;

  //calculate the angles of the vectors pointing to the current_vertex
  if(current_vertex == GTS_SEGMENT(e1)->v1)
    {
      angle1 = atan2(GTS_POINT(GTS_SEGMENT(e1)->v1)->y - GTS_POINT(GTS_SEGMENT(e1)->v2)->y,
                     GTS_POINT(GTS_SEGMENT(e1)->v1)->x - GTS_POINT(GTS_SEGMENT(e1)->v2)->x);
    }
  else
    {
      angle1 = atan2(GTS_POINT(GTS_SEGMENT(e1)->v2)->y - GTS_POINT(GTS_SEGMENT(e1)->v1)->y,
                     GTS_POINT(GTS_SEGMENT(e1)->v2)->x - GTS_POINT(GTS_SEGMENT(e1)->v1)->x);
    }

  if(current_vertex == GTS_SEGMENT(e2)->v1)
    {
      angle2 = atan2(GTS_POINT(GTS_SEGMENT(e2)->v1)->y - GTS_POINT(GTS_SEGMENT(e2)->v2)->y,
                     GTS_POINT(GTS_SEGMENT(e2)->v1)->x - GTS_POINT(GTS_SEGMENT(e2)->v2)->x);
    }
  else
    {
      angle2 = atan2(GTS_POINT(GTS_SEGMENT(e2)->v2)->y - GTS_POINT(GTS_SEGMENT(e2)->v1)->y,
                     GTS_POINT(GTS_SEGMENT(e2)->v2)->x - GTS_POINT(GTS_SEGMENT(e2)->v1)->x);
    }

  float bisector = fabs(angle1 - angle2);
  if(bisector > M_PI)
    {
      bisector = (M_PI * 2) - bisector;
    }
  bisector /= 2.;

  GtsPoint* p1 = GTS_POINT(GTS_SEGMENT(e1)->v1);
  GtsPoint* p2 = GTS_POINT(GTS_SEGMENT(e1)->v2);
  GtsVertex* opposite_v1 =  gts_triangle_vertex_opposite(GTS_TRIANGLE(e1->triangles->data), e1);

  //turn the vector and check whether it's on the right side, if not turn the other way round
  float length = robot_width + (2 * minimum_distance) + GTS_OBSTACLE(current_vertex)->width;
  float alpha = angle1  + bisector;
  float v_x = GTS_POINT(current_vertex)->x + length * cos(alpha);
  float v_y = GTS_POINT(current_vertex)->y + length * sin(alpha);

  if(v_y > ((p2->y - p1->y) / (p2->x - p1->x)) * (v_x - p1->x) + p1->y)
    {
      if(GTS_POINT(opposite_v1)->y > ((p2->y - p1->y) / (p2->x - p1->x)) * (GTS_POINT(opposite_v1)->x - p1->x) + p1->y)
        {
          alpha = angle2  + bisector;
          v_x = GTS_POINT(current_vertex)->x + length * cos(alpha);
          v_y = GTS_POINT(current_vertex)->y + length * sin(alpha);
        }
    }
  else
    {
      if(GTS_POINT(opposite_v1)->y <= ((p2->y - p1->y) / (p2->x - p1->x)) * (GTS_POINT(opposite_v1)->x - p1->x) + p1->y)
        {
          alpha = angle2  + bisector;
          v_x = GTS_POINT(current_vertex)->x + length * cos(alpha);
          v_y = GTS_POINT(current_vertex)->y + length * sin(alpha);
        }
    }
  map->insert(make_pair(current_vertex, gts_vertex_new(gts_vertex_class(), v_x, v_y, 0)));
}

/** Sets the target.
 * @param x the x-coordinate of the target relative to the robot
 * @param y the y-coordinate of the target relative to the robot
 */
void Pathfinder::set_target(double x, double y)
{
  gts_point_set(target_point, x, y, 0);
}

/** Calculates the delaunay triangulation for the given surface and the given obstacles.
 * @param surface the surface for the GTS delaunay function
 * @param obstacles the obstacles around the robot within the scanned area
 */
void Pathfinder::delaunay(GtsSurface * surface, std::vector< GtsObstacle *> obstacles)
{
  for (guint i = 0; i < obstacles.size(); i++)
    {
      GtsVertex * ver = (GtsVertex *)obstacles[i];
      gts_delaunay_add_vertex(surface, ver, NULL);
    }

  gts_allow_floating_vertices = TRUE;
  gts_object_destroy(GTS_OBJECT(v0));
  gts_object_destroy(GTS_OBJECT(v1));
  gts_object_destroy(GTS_OBJECT(v2));
  gts_object_destroy(GTS_OBJECT(v3));
  gts_allow_floating_vertices = FALSE;

  if(surface != NULL && gts_surface_face_number(surface) > 0)
    {
      start_face = gts_point_locate (robot_point, surface, NULL);
      GSList* boundary_list  =  gts_surface_boundary(surface);
      GSList* list = boundary_list;

      //the outer triangles may flicker because the gts_point_locate does not work
      //accurate if the surface is concave
      bool target_outside_triangulation = false;
      if(gts_point_locate(target_point, surface, NULL) == NULL)
        {
          //          printf("target outside\n");
          target_outside_triangulation = true;
        }
      //      else
      //        printf("target inside \n");
      bool robot_outside_triangulation = false;
      if(gts_point_locate(robot_point, surface, NULL) == NULL)
        {
          //          printf("robot outside\n");
          robot_outside_triangulation = true;
        }
      //      else
      //        printf("robot inside \n");

	  //calculate the bisectors
      std::map<GtsVertex*, GtsVertex*>* bisectors_map = new std::map<GtsVertex*, GtsVertex*>();
      do //for each boundary edge
        {
          GtsVertex* current_vertex = NULL;
          //angle bisection
          if(bisectors_map->find(GTS_SEGMENT(list->data)->v1) == bisectors_map->end())
            {
              current_vertex = GTS_SEGMENT(list->data)->v1;
              find_bisector(current_vertex, bisectors_map);
            }
          if(bisectors_map->find(GTS_SEGMENT(list->data)->v2) == bisectors_map->end())
            {
              current_vertex = GTS_SEGMENT(list->data)->v2;
              find_bisector(current_vertex, bisectors_map);
            }
        }
      while((list = list->next) != NULL);

      GtsPoint* p1 = NULL;
      GtsPoint* p2 = NULL;
      list = boundary_list;

      do //for each boundary edge
        {
          GtsVertex* center_v = gts_segment_midvertex(GTS_SEGMENT(list->data), gts_vertex_class());
          GtsVertex* opposite_v =  gts_triangle_vertex_opposite(GTS_TRIANGLE(GTS_EDGE(list->data)->triangles->data), GTS_EDGE(list->data));

          if(robot_outside_triangulation)
            {
              bool no_triangle = true;
              GSList* polygon_list = boundary_list;
              do
                {
                  GtsVertex* v1 = gts_vertex_new(gts_vertex_class(), robot_point->x, robot_point->y, robot_point->z);
                  GtsVertex* v2 = gts_vertex_new(gts_vertex_class(), GTS_POINT(center_v)->x, GTS_POINT(center_v)->y, GTS_POINT(center_v)->z);
                  GtsSegment* s = gts_segment_new(gts_segment_class(), v1, v2);

                  if(polygon_list->data != list->data
                      && GTS_IN == gts_segments_are_intersecting(s, GTS_SEGMENT(polygon_list->data)))
                    {
                      gts_object_destroy(GTS_OBJECT(s));
                      no_triangle = false;
                      break;
                    }
                  gts_object_destroy(GTS_OBJECT(s));
                }
              while((polygon_list = polygon_list->next) != NULL);
              //if the direct line between the robot_point and center_v does not intersect the triangulation -> no new triangle is added
              if(no_triangle)
                {
                  continue;
                }
            }

          if(target_outside_triangulation)
            {
              bool no_triangle = true;
              GSList* polygon_list = boundary_list;
              do
                {
                  GtsVertex* v1 = gts_vertex_new(gts_vertex_class(), target_point->x, target_point->y, target_point->z);
                  GtsVertex* v2 = gts_vertex_new(gts_vertex_class(), GTS_POINT(center_v)->x, GTS_POINT(center_v)->y, GTS_POINT(center_v)->z);
                  GtsSegment* s = gts_segment_new(gts_segment_class(), v1, v2);

                  if(polygon_list->data != list->data
                      && GTS_IN == gts_segments_are_intersecting(s, GTS_SEGMENT(polygon_list->data)))
                    {
                      gts_object_destroy(GTS_OBJECT(s));
                      no_triangle = false;
                      break;
                    }
                  gts_object_destroy(GTS_OBJECT(s));
                }
              while((polygon_list = polygon_list->next) != NULL);
              //if the direct line between the target_point and center_v does not intersect the triangulation -> no new triangle is added
              if(no_triangle)
                {
                  continue;
                }
            }

          //add a new triangle

          p1 = GTS_POINT(GTS_SEGMENT(list->data)->v1);
          p2 = GTS_POINT(GTS_SEGMENT(list->data)->v2);
          float v_x = 0;
          float v_y = 0;
          int sign = 0;
          float length = robot_width + (2 * minimum_distance) + max(GTS_OBSTACLE(p1)->width, GTS_OBSTACLE(p2)->width);
          float direction = atan2((p2->y - p1->y), (p2->x - p1->x));

          //turn the vector and check whether it's on the right side, if not turn the other way round
          //Is it possible that p2->x - p1->x == 0?
          if(GTS_POINT(opposite_v)->y < ((p2->y - p1->y) / (p2->x - p1->x)) * (GTS_POINT(opposite_v)->x - p1->x) + p1->y)
            {
              v_x = GTS_POINT(center_v)->x + length * cos(direction + M_PI / 2.);
              v_y = GTS_POINT(center_v)->y + length * sin(direction + M_PI / 2.);
              sign = 1;

              if(v_y < ((p2->y - p1->y) / (p2->x - p1->x)) * (v_x - p1->x) + p1->y)
                {
                  v_x = GTS_POINT(center_v)->x + length * cos(direction - M_PI / 2.);
                  v_y = GTS_POINT(center_v)->y + length * sin(direction - M_PI / 2.);
                  sign = -1;
                }
            }
          else
            {
              v_x = GTS_POINT(center_v)->x + length * cos(direction - M_PI / 2.);
              v_y = GTS_POINT(center_v)->y + length * sin(direction - M_PI / 2.);
              sign = -1;

              if(v_y >= ((p2->y - p1->y) / (p2->x - p1->x)) * (v_x - p1->x) + p1->y)
                {
                  v_x = GTS_POINT(center_v)->x + length * cos(direction + M_PI / 2.);
                  v_y = GTS_POINT(center_v)->y + length * sin(direction + M_PI / 2.);
                  sign = 1;
                }
            }

          //this was a try to check if the surface is at the current edge cuncave
          //because then the auxiliary edges left and right perpendicular to the edge are crossed
          //and the bisector is wrong
          //but there is a bug somewhere
          //          bool concave1 = false;
          //          bool concave2 = false;
          //          GtsEdge* e1 = NULL;
          //          GtsEdge* e2 = NULL;
          //
          //          GSList* edge_list = GTS_VERTEX(p1)->segments;
          //          do
          //            {
          //              if(gts_edge_is_boundary(GTS_EDGE(edge_list->data), surface)
          //                  && edge_list->data != list->data)
          //                {
          //                  e1 = GTS_EDGE(edge_list->data);
          //                  break;
          //                }
          //            }
          //          while((edge_list = edge_list->next) != NULL);
          //          edge_list = GTS_VERTEX(p2)->segments;
          //          do
          //            {
          //              if(gts_edge_is_boundary(GTS_EDGE(edge_list->data), surface)
          //                  && edge_list->data != list->data)
          //                {
          //                  e2 = GTS_EDGE(edge_list->data);
          //                  break;
          //                }
          //            }
          //          while((edge_list = edge_list->next) != NULL);
          //          GtsVertex* concave_v1 = NULL;
          //          GtsVertex* concave_v2 = NULL;
          //          if(GTS_SEGMENT(e1)->v1 != GTS_VERTEX(p1))
          //            {
          //              concave_v1 = GTS_SEGMENT(e1)->v1;
          //            }
          //          else
          //            {
          //              concave_v1 = GTS_SEGMENT(e1)->v2;
          //            }
          //          if(GTS_SEGMENT(e2)->v1 != GTS_VERTEX(p2))
          //            {
          //              concave_v2 = GTS_SEGMENT(e2)->v1;
          //            }
          //          else
          //            {
          //              concave_v2 = GTS_SEGMENT(e2)->v2;
          //            }
          //
          //          if(sign == 1 && GTS_POINT(concave_v1)->y >= ((p2->y - p1->y) / (p2->x - p1->x)) * (GTS_POINT(concave_v1)->x - p1->x) + p1->y)
          //            {
          //              concave1 = true;
          //            }
          //
          //          if(sign == 1 && GTS_POINT(concave_v2)->y >= ((p2->y - p1->y) / (p2->x - p1->x)) * (GTS_POINT(concave_v2)->x - p1->x) + p1->y)
          //            {
          //              concave2 = true;
          //            }


          float v1_x = p1->x + length * cos(direction + sign * M_PI / 2.);
          float v1_y = p1->y + length * sin(direction + sign * M_PI / 2.);
          float v2_x = p2->x + length * cos(direction + sign * M_PI / 2.);
          float v2_y = p2->y + length * sin(direction + sign * M_PI / 2.);

          //the auxiliary edges left and right perpendicular to the edge
          GtsVertex* auxiliary_vertex1 = gts_vertex_new(gts_vertex_class(), v1_x, v1_y, 0);
          GtsVertex* auxiliary_vertex2 = gts_vertex_new(gts_vertex_class(), v2_x, v2_y, 0);
          GtsEdge* auxiliary_edge1 = gts_edge_new(gts_edge_class(), GTS_VERTEX(p1), auxiliary_vertex1);
          GtsEdge* auxiliary_edge2 = gts_edge_new(gts_edge_class(), GTS_VERTEX(p2), auxiliary_vertex2);

          //the auxiliary triangle
          GtsVertex* auxiliary_triangle_vertex = gts_vertex_new(gts_vertex_class(), v_x, v_y, 0);
          GtsEdge* triangle_e1 = gts_edge_new(gts_edge_class(), GTS_VERTEX(p1), auxiliary_triangle_vertex);
          GtsEdge* triangle_e2 = gts_edge_new(gts_edge_class(), GTS_VERTEX(p2), auxiliary_triangle_vertex);

          //the outer connecting edges between the auxiliary triangle and the auxiliary vertexes
          GtsEdge* auxiliary_edge_1 = gts_edge_new(gts_edge_class(), auxiliary_vertex1, auxiliary_triangle_vertex);
          GtsEdge* auxiliary_edge_2 = gts_edge_new(gts_edge_class(), auxiliary_vertex2, auxiliary_triangle_vertex);
          GtsFace* face = NULL;

          face = gts_face_new(gts_face_class(), GTS_EDGE(list->data), triangle_e1, triangle_e2);
          if(gts_triangle_orientation(GTS_TRIANGLE(face)) < 0)
            {
              gts_triangle_revert(GTS_TRIANGLE(face));
            }
          gts_surface_add_face(surface, face);
          face = gts_face_new(gts_face_class(), auxiliary_edge_1, triangle_e1, auxiliary_edge1);
          if(gts_triangle_orientation(GTS_TRIANGLE(face)) < 0)
            {
              gts_triangle_revert(GTS_TRIANGLE(face));
            }
          gts_surface_add_face(surface, face);
          face = gts_face_new(gts_face_class(), auxiliary_edge_2, triangle_e2, auxiliary_edge2);
          if(gts_triangle_orientation(GTS_TRIANGLE(face)) < 0)
            {
              gts_triangle_revert(GTS_TRIANGLE(face));
            }
          gts_surface_add_face(surface, face);

          //if there are bisection vertexes for v1 or v2, then add new faces to the surface by using this vertexes
          std::map<GtsVertex*, GtsVertex*>::iterator iterator;
          if((iterator = bisectors_map->find(GTS_SEGMENT(list->data)->v1)) != bisectors_map->end())
            {
              GtsEdge* e1 = auxiliary_edge1;
              GtsEdge* e2 = NULL;
              GSList* segments_list = iterator->second->segments;
              if(segments_list != NULL)
                {
                  do
                    {
                      e2 = GTS_EDGE((GtsSegment*)segments_list->data);
                      segments_list = segments_list->next;
                    }
                  while(!gts_segment_connect(GTS_SEGMENT(e2), iterator->second, GTS_VERTEX(p1)));
                }
              else
                {
                  e2 = gts_edge_new(gts_edge_class(), GTS_VERTEX(p1), iterator->second);
                }
              GtsEdge* e3 = gts_edge_new(gts_edge_class(), auxiliary_vertex1, iterator->second);
              face = gts_face_new(gts_face_class(), e1, e2, e3);
              if(gts_triangle_orientation(GTS_TRIANGLE(face)) < 0)
                {//avoids holes within the surface
                  gts_triangle_revert(GTS_TRIANGLE(face));
                }
              gts_surface_add_face(surface, face);
            }

          if((iterator = bisectors_map->find(GTS_SEGMENT(list->data)->v2)) != bisectors_map->end())
            {
              GtsEdge* e1 = auxiliary_edge2;
              GtsEdge* e2 = NULL;
              GSList* segments_list = iterator->second->segments;
              if(segments_list != NULL)
                {
                  do
                    {
                      e2 = GTS_EDGE((GtsSegment*)segments_list->data);
                      segments_list = segments_list->next;
                    }
                  while(!gts_segment_connect(GTS_SEGMENT(e2), iterator->second, GTS_VERTEX(p2)));
                }
              else
                {
                  e2 = gts_edge_new(gts_edge_class(), GTS_VERTEX(p2), iterator->second);
                }
              GtsEdge* e3 = gts_edge_new(gts_edge_class(), auxiliary_vertex2, iterator->second);
              face = gts_face_new(gts_face_class(), e1, e2, e3);
              if(gts_triangle_orientation(GTS_TRIANGLE(face)) < 0)
                {//avoids holes within the surface
                  gts_triangle_revert(GTS_TRIANGLE(face));
                }
              gts_surface_add_face(surface, face);
            }
        }
      while((list = list->next) != NULL);
    } // if(surface != NULL && gts_surface_face_number(surface) > 0)
}

/** A kind of sign funcion.
 * @param x the funtions
 */
double Pathfinder::p_sgn(double x)
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
bool Pathfinder::is_in_rectangle(double point_x, double point_y,
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
  /*     ____________ ..start_base(width of the robot)
   *     |.......S.........|..start(robot)
   *     |.................|
   *     |..............__|____..obstacle_base (width of the obstacle)
   *     |.................|.O...obstacle
   *     |.................|
   *     |___________|..target_base1..\
   *     |........T........|...target...........>.....distance between base1 and base2 is the 
   *     |.................|......................|...........................radius of the robot
   *     |___________|..target_base2../
   */
  bool test = false;

  double pi = 4 * atan(1.);

  double start_x = robot_point->x;
  double start_y = robot_point->y;

  double target_x = target_point->x;
  double target_y = target_point->y;


  double start_base1_x = 0;
  double start_base1_y = 0;

  double start_base2_x = 0;
  double start_base2_y = 0;

  double target_base1_x = 0;
  double target_base1_y = 0;

  double target_base2_x = 0;
  double target_base2_y = 0;

  double target_back_x = 0;
  double target_back_y = 0;

  double obstacle_base1_x = 0;
  double obstacle_base1_y = 0;

  double obstacle_base2_x = 0;
  double obstacle_base2_y = 0;

  double obstacle_x = 0;
  double obstacle_y = 0;

  double obstacle_width = 0;

  double m1_1 = 0;
  double m1_2 = 0;


  double tan = 0;

  m1_1 = (target_x + start_x)/2;
  m1_2 = (target_y + start_y)/2;


  if(m1_1 == 0)
    tan = pi/2;
  else if(m1_2 == 0)
    tan = pi;
  else
    tan = atan(-m1_2 / m1_1);

  double tan2 = 0;

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

      obstacle_x = GTS_POINT(obstacle)->x;
      obstacle_y = GTS_POINT(obstacle)->y;

      obstacle_width = obstacle->width;

      gts_object_destroy(GTS_OBJECT(obstacle));

      obstacle_base1_x = p_sgn(tan) * sin(tan) * (obstacle_width / 2.) +  obstacle_x;
      obstacle_base1_y = p_sgn(tan) * cos(tan) * (obstacle_width / 2.) +  obstacle_y;

      obstacle_base2_x = p_sgn(tan) * -sin(tan) * (obstacle_width / 2.) +  obstacle_x;
      obstacle_base2_y = p_sgn(tan) * -cos(tan) * (obstacle_width / 2.) +  obstacle_y;

      //it checks only if the left or the right or the middle lies within the recangle
      //this requires that the obstace is never more than twice as width as the robot
      test |= is_in_rectangle(obstacle_base1_x, obstacle_base1_y,
                              start_base1_x, start_base1_y,
                              target_base1_x, target_base1_y,
                              target_base2_x, target_base2_y,
                              start_base2_x, start_base2_y);
      test |= is_in_rectangle(obstacle_base2_x, obstacle_base2_y,
                              start_base1_x, start_base1_y,
                              target_base1_x, target_base1_y,
                              target_base2_x, target_base2_y,
                              start_base2_x, start_base2_y);
      test |= is_in_rectangle(obstacle_x, obstacle_y,
                              start_base1_x, start_base1_y,
                              target_base1_x, target_base1_y,
                              target_base2_x, target_base2_y,
                              start_base2_x, start_base2_y);
      if(test)
        break;
    }
  return !test;
}

/** Pushes a vertex out of a GTS fifo.
 *  It is needed for the gts_surface_foreach_edge function of GTS.
 * @param vertex the pushed vertex
 * @param fifo the GTS fifo with the vertexes
 */
void Pathfinder::get_vertexes(GtsVertex *vertex, GtsFifo * fifo)
{
  gts_fifo_push(fifo, GTS_OBJECT(vertex));
}

/** Returns the nearest obstacle.
 * @return the nearest obstacle
 */
Obstacle* Pathfinder::get_nearest_obstacle()
{
  double length = 1000000;
  Obstacle * obstacle = NULL;

  for(unsigned int i = 0; i < map.size(); i++)
    {
      double distance = sqrt(((map[i].x - robot_point->x) * (map[i].x - robot_point->x))
                             + ((map[i].y - robot_point->y) * (map[i].y - robot_point->y))) - map[i].width/2. - robot_width/2.;
      if(distance < length)
        {
          length = distance;
          obstacle = &map[i];
        }
    }
  return obstacle;
}

/** Sets the recognized obstacles to the pathfinder.
 * @param obstacles a vector of obstacles
 */
void Pathfinder::set_obstacles(std::vector< Obstacle > obstacles)
{
  map = obstacles;
}

/** Adds an obstacle to the pathfinder.
 * @param obstacle an obstacle
 */
void Pathfinder::add_obstacle(Obstacle obstacle)
{
  map.push_back(obstacle);
}

/** Regains the Obstacles from the map if every obstacle
 *   is destoyed by destoying the surface.
 */
void Pathfinder::regain_obstacles()
{
  this->obstacles.clear();

  //generate a vector of obstacles in the scanning_area
  for(unsigned int i = 0; i < map.size(); i++)
    {
      Obstacle o = map[i];
      if(!out_of_area(o.x, o.y))
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
bool Pathfinder::out_of_area(double x, double y)
{
  return (   x < -scanning_area_width  / 2 && x > scanning_area_width  / 2
             && y < -scanning_area_height / 2 && y > scanning_area_height / 2);
}

/** Calculates the delaunay triangulation.
 */
void Pathfinder::calculate_delaunay()
{
  gts_object_destroy(GTS_OBJECT(surface));
  init_surface();
  regain_obstacles();
  delaunay(surface, obstacles);
}

/** Returns the surface of the delaunay triangulation.
 * @return the surface of the delaunay triangulation
 */
GtsSurface * Pathfinder::get_surface()
{
  return surface;
}

/** Returns the position of the robot.
 * @return the position of the robot as a GtsPoint
 */
GtsPoint * Pathfinder::get_robot_point()
{
  return robot_point;
}

/** Returns the current target.
 * @return the target as GtsPoint
 */
GtsPoint * Pathfinder::get_target_point()
{
  return target_point;
}


/** Returns the path.
 * @return a vector of the path points as GtsPoints
 */
std::vector< GtsPoint * > Pathfinder::get_path()
{
  std::vector< GtsPoint * > solution_path;

  //calculate the delaunay triangulation
  calculate_delaunay();

  //if there is no obstacle on the way, then the robot can drive ahead
  //or there is no triangulation 
  if((surface != NULL && gts_surface_face_number(surface) == 0) || test_straight_ahead())
    {
      solution_path.push_back(gts_point_new(gts_point_class(), robot_point->x, robot_point->y, 0));
      solution_path.push_back(gts_point_new(gts_point_class(), target_point->x, target_point->y, 0));
    }
  else
    {
      AStar * astar = new AStar();
      PathState * initialState = new PathState(surface, NULL,
                                 robot_point,
                                 start_face,  target_point, robot_point, 0, 0);
      std::vector< AStarState * > solution = astar->solve( initialState );

      for (unsigned int i = 0; i < solution.size(); i++ )
        {
          PathState * state = (PathState *)(solution[i]);
          GtsPoint * p = state->getPoint();
          solution_path.push_back(gts_point_new(gts_point_class(), p->x, p->y, 0));
        }

      for (unsigned int j = 0; j < solution.size(); j++ )
        {
          PathState * state = (PathState *)(solution[j]);
          delete state;
        }

      solution_path.push_back(gts_point_new(gts_point_class(), target_point->x, target_point->y, 0));
      delete astar;
    }
  return solution_path;
}
