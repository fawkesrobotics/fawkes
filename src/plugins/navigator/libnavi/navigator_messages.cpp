/***************************************************************************
 *  navigator_messages.cpp - Navigator Messages
 *
 *  Generated: Thu May 31 20:39:54 2007
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
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */


#include <plugins/navigator/libnavi/navigator_messages.h>
#include <plugins/navigator/libnavi/npoint.h>
#include <plugins/navigator/libnavi/nline.h>
#include <plugins/navigator/libnavi/obstacle.h>

#include <netcomm/utils/dynamic_buffer.h>
#include <core/exception.h>
#include <cstdlib>
#include <cstring>

/** @class NavigatorNodesListMessage plugins/navigator/libnavi/navigator_messages.h
 * This message is for containing the points of the surface of the triangulation.
 */

/** Constructor.
 * @param nodes a list with the nodes for this message
 */
NavigatorNodesListMessage::NavigatorNodesListMessage(std::list<NPoint *> *nodes)
{
  nodes_list = new DynamicBuffer(&(msg.nodes_list));

  std::list<NPoint *>::iterator i;
  npoint_t np;
  for ( i = nodes->begin(); i != nodes->end(); ++i )
    {
      np.x = (*i)->x;
      np.y = (*i)->y;
      nodes_list->append(&np, sizeof(np));
    }
}


/** Message content constructor.
 * This constructor is meant to be used with FawkesNetworkMessage::msgc().
 * @param component_id component ID
 * @param msg_id message ID
 * @param payload message payload
 * @param payload_size total payload size
 */
NavigatorNodesListMessage::NavigatorNodesListMessage(unsigned int component_id, unsigned int msg_id,
    void *payload, size_t payload_size)
{
  navigator_nodes_msg_t *tmsg = (navigator_nodes_msg_t *)payload;
  void *nodes_list_payload = (void *)((size_t)payload + sizeof(msg));
  nodes_list = new DynamicBuffer(&(tmsg->nodes_list), nodes_list_payload,
                                 payload_size - sizeof(msg));
}


/** Destructor. */
NavigatorNodesListMessage::~NavigatorNodesListMessage()
{
  delete nodes_list;
  if (_payload != NULL)
    {
      free(_payload);
      _payload = NULL;
      _payload_size = 0;
    }
}


void
NavigatorNodesListMessage::serialize()
{
  _payload_size = sizeof(msg) + nodes_list->buffer_size();
  _payload = malloc(_payload_size);
  copy_payload(0, &msg, sizeof(msg));
  copy_payload(sizeof(msg), nodes_list->buffer(), nodes_list->buffer_size());
}


/** Reset iterator.
 * For incoming messages only.
 */
void
NavigatorNodesListMessage::reset_iterator()
{
  nodes_list->reset_iterator();
}


/** Check if more list elements are available.
 * For incoming messages only.
 * @return true if there are more elements available, false otherwise.
 */
bool
NavigatorNodesListMessage::has_next()
{
  return nodes_list->has_next();
}


/** Get next plugin from list.
 * @return next plugin from list. This string has been allocated via strndup, so
 * you have to free it yourself!
 */
NavigatorNodesListMessage::npoint_t *
NavigatorNodesListMessage::next()
{
  size_t size;
  void *tmp = nodes_list->next(&size);
  return (npoint_t *)tmp;
}

/** @class NavigatorLinesListMessage <navigator/libnavi/navigator_messages.h>
 * This message is for containing the lines of the surface of the triangulation.
 */

/** Constructor.
 * @param lines a list with the lines for this message 
 */
NavigatorLinesListMessage::NavigatorLinesListMessage(std::list<NLine *> *lines)
{
  lines_list = new DynamicBuffer(&(msg.lines_list));

  std::list<NLine *>::iterator i;
  nline_t nl;
  for ( i = lines->begin(); i != lines->end(); ++i )
    {
      Obstacle *op1 = dynamic_cast<Obstacle *>((*i)->p1);
      if ( op1 )
        {
          // it IS an obstacle
          nl.width1 = op1->width;
        }
      else
        {
          // it's just a point
          nl.width1 = 0.;
        }
        
      Obstacle *op2 = dynamic_cast<Obstacle *>((*i)->p2);
      if ( op2 )
        {
          // it IS an obstacle
          nl.width2 = op2->width;
        }
      else
        {
          // it's just a point
          nl.width2 = 0.;
        }
      nl.x1 = (*i)->p1->x;
      nl.y1 = (*i)->p1->y;
      nl.x2 = (*i)->p2->x;
      nl.y2 = (*i)->p2->y;
      lines_list->append(&nl, sizeof(nl));
    }
}


/** Message content constructor.
 * This constructor is meant to be used with FawkesNetworkMessage::msgc().
 * @param component_id component ID
 * @param msg_id message ID
 * @param payload message payload
 * @param payload_size total payload size
 */
NavigatorLinesListMessage::NavigatorLinesListMessage(unsigned int component_id, unsigned int msg_id,
    void *payload, size_t payload_size)
{
  navigator_lines_msg_t *tmsg = (navigator_lines_msg_t *)payload;
  void *lines_list_payload = (void *)((size_t)payload + sizeof(msg));
  lines_list = new DynamicBuffer(&(tmsg->lines_list), lines_list_payload,
                                 payload_size - sizeof(msg));
}


/** Destructor. */
NavigatorLinesListMessage::~NavigatorLinesListMessage()
{
  delete lines_list;
  if (_payload != NULL)
    {
      free(_payload);
      _payload = NULL;
      _payload_size = 0;
    }
}


void
NavigatorLinesListMessage::serialize()
{
  _payload_size = sizeof(msg) + lines_list->buffer_size();
  _payload = malloc(_payload_size);
  copy_payload(0, &msg, sizeof(msg));
  copy_payload(sizeof(msg), lines_list->buffer(), lines_list->buffer_size());
}


/** Reset iterator.
 * For incoming messages only.
 */
void
NavigatorLinesListMessage::reset_iterator()
{
  lines_list->reset_iterator();
}


/** Check if more list elements are available.
 * For incoming messages only.
 * @return true if there are more elements available, false otherwise.
 */
bool
NavigatorLinesListMessage::has_next()
{
  return lines_list->has_next();
}


/** Get next plugin from list.
 * @return next plugin from list. This string has been allocated via strndup, so
 * you have to free it yourself!
 */
NavigatorLinesListMessage::nline_t *
NavigatorLinesListMessage::next()
{
  size_t size;
  void *tmp = lines_list->next(&size);
  return (nline_t *)tmp;
}

/** @class NavigatorPathListMessage plugins/navigator/libnavi/navigator_messages.h
 * This message is for containing the points of the path by the navigator.
 */

/** Constructor.
 * @param points a list with the path points for this message
 */
NavigatorPathListMessage::NavigatorPathListMessage(std::list<NPoint *> *points)
{
  path_list = new DynamicBuffer(&(msg.path_list));

  std::list<NPoint *>::iterator i;
  npoint_t np;
  for ( i = points->begin(); i != points->end(); ++i )
    {
      np.x = (*i)->x;
      np.y = (*i)->y;
      path_list->append(&np, sizeof(np));
    }
}


/** Message content constructor.
 * This constructor is meant to be used with FawkesNetworkMessage::msgc().
 * @param component_id component ID
 * @param msg_id message ID
 * @param payload message payload
 * @param payload_size total payload size
 */
NavigatorPathListMessage::NavigatorPathListMessage(unsigned int component_id, unsigned int msg_id,
    void *payload, size_t payload_size)
{
  navigator_path_msg_t *tmsg = (navigator_path_msg_t *)payload;
  void *path_list_payload = (void *)((size_t)payload + sizeof(msg));
  path_list = new DynamicBuffer(&(tmsg->path_list), path_list_payload,
                                payload_size - sizeof(msg));
}


/** Destructor. */
NavigatorPathListMessage::~NavigatorPathListMessage()
{
  delete path_list;
  if (_payload != NULL)
    {
      free(_payload);
      _payload = NULL;
      _payload_size = 0;
    }
}


void
NavigatorPathListMessage::serialize()
{
  _payload_size = sizeof(msg) + path_list->buffer_size();
  _payload = malloc(_payload_size);
  copy_payload(0, &msg, sizeof(msg));
  copy_payload(sizeof(msg), path_list->buffer(), path_list->buffer_size());
}


/** Reset iterator.
 * For incoming messages only.
 */
void
NavigatorPathListMessage::reset_iterator()
{
  path_list->reset_iterator();
}


/** Check if more list elements are available.
 * For incoming messages only.
 * @return true if there are more elements available, false otherwise.
 */
bool
NavigatorPathListMessage::has_next()
{
  return path_list->has_next();
}


/** Get next plugin from list.
 * @return next plugin from list. This string has been allocated via strndup, so
 * you have to free it yourself!
 */
NavigatorPathListMessage::npoint_t *
NavigatorPathListMessage::next()
{
  size_t size;
  void *tmp = path_list->next(&size);
  return (npoint_t *)tmp;
}

/** @class NavigatorObstacleListMessage plugins/navigator/libnavi/navigator_messages.h
 * This message is for containing the obstacles surrounding the robot.
 */

/** Constructor.
 * @param obstacles a list with the path points for this message
 */
NavigatorObstaclesListMessage::NavigatorObstaclesListMessage(std::list<NPoint *> *obstacles)
{
  obstacles_list = new DynamicBuffer(&(msg.obstacles_list));

  std::list<NPoint *>::const_iterator i;
  obstacle_t o;
  point_t p;
  Obstacle * op;
  for ( i = obstacles->begin(); i != obstacles->end(); ++i )
    {
      op = dynamic_cast<Obstacle *>(*i);
      if ( op )
        {
          // it IS an obstacle
          o.x = op->x;
          o.y = op->y;
          o.width = op->width;
          obstacles_list->append(&o, sizeof(o));
        }
      else
        {
          // it's just a point
          p.x = (*i)->x;
          p.y = (*i)->y;
          obstacles_list->append(&p, sizeof(p));
        }
    }
}


/** Message content constructor.
 * This constructor is meant to be used with FawkesNetworkMessage::msgc().
 * @param component_id component ID
 * @param msg_id message ID
 * @param payload message payload
 * @param payload_size total payload size
 */
NavigatorObstaclesListMessage::NavigatorObstaclesListMessage(unsigned int component_id, unsigned int msg_id,
    void *payload, size_t payload_size)
{
  navigator_obstacles_list_msg_t *tmsg = (navigator_obstacles_list_msg_t *)payload;
  void *obstacles_list_payload = (void *)((size_t)payload + sizeof(msg));
  obstacles_list = new DynamicBuffer(&(tmsg->obstacles_list), obstacles_list_payload,
                                     payload_size - sizeof(msg));
}


/** Destructor. */
NavigatorObstaclesListMessage::~NavigatorObstaclesListMessage()
{
  delete obstacles_list;
  if (_payload != NULL)
    {
      free(_payload);
      _payload = NULL;
      _payload_size = 0;
    }
}


void
NavigatorObstaclesListMessage::serialize()
{
  _payload_size = sizeof(msg) + obstacles_list->buffer_size();
  _payload = malloc(_payload_size);
  copy_payload(0, &msg, sizeof(msg));
  copy_payload(sizeof(msg), obstacles_list->buffer(), obstacles_list->buffer_size());
}


/** Reset iterator.
 * For incoming messages only.
 */
void
NavigatorObstaclesListMessage::reset_iterator()
{
  obstacles_list->reset_iterator();
}


/** Check if more list elements are available.
 * For incoming messages only.
 * @return true if there are more elements available, false otherwise.
 */
bool
NavigatorObstaclesListMessage::has_next()
{
  return obstacles_list->has_next();
}


/** Get next plugin from list.
 * @return next plugin from list. This string has been allocated via strndup, so
 * you have to free it yourself!
 */
//NavigatorObstaclesListMessage::obstacle_t *
NPoint *
NavigatorObstaclesListMessage::next()
{
  size_t size;
  void *tmp = obstacles_list->next(&size);

  if ( size == sizeof(obstacle_t) )
    {
      return new Obstacle(((obstacle_t *)tmp)->width, ((obstacle_t *)tmp)->x, ((obstacle_t *)tmp)->y, 0);
    }
  else if ( size == sizeof(point_t) )
    {
      return new NPoint(((point_t *)tmp)->x, ((point_t *)tmp)->y);
    }
  else
    {
      throw Exception("unsupported value");
    }
  // return (obstacle_t *)tmp;
}
