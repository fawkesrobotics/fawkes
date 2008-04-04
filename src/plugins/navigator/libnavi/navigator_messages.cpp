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

/** @class NavigatorObstaclesListMessage plugins/navigator/libnavi/navigator_messages.h
 * This message is for containing the obstacles maintained by the navigator.
 */

/** Constructor.
 * @param obstacles a list with the obstacles for this message
 */
NavigatorObstaclesListMessage::NavigatorObstaclesListMessage(std::list<Obstacle *> *obstacles)
{
  obstacle_list = new DynamicBuffer(&(msg.obstacle_list));

  std::list<Obstacle *>::iterator i;
  obstacle_t o;
  for ( i = obstacles->begin(); i != obstacles->end(); ++i )
    {
      o.x = (*i)->x;
      o.y = (*i)->y;
      o.width =  (*i)->width;
      obstacle_list->append(&o, sizeof(o));
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
  navigator_obstacles_msg_t *tmsg = (navigator_obstacles_msg_t *)payload;
  void *obstacle_list_payload = (void *)((size_t)payload + sizeof(msg));
  obstacle_list = new DynamicBuffer(&(tmsg->obstacle_list), obstacle_list_payload,
                                payload_size - sizeof(msg));
}


/** Destructor. */
NavigatorObstaclesListMessage::~NavigatorObstaclesListMessage()
{
  delete obstacle_list;
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
  _payload_size = sizeof(msg) + obstacle_list->buffer_size();
  _payload = malloc(_payload_size);
  copy_payload(0, &msg, sizeof(msg));
  copy_payload(sizeof(msg), obstacle_list->buffer(), obstacle_list->buffer_size());
}


/** Reset iterator.
 * For incoming messages only.
 */
void
NavigatorObstaclesListMessage::reset_iterator()
{
  obstacle_list->reset_iterator();
}


/** Check if more list elements are available.
 * For incoming messages only.
 * @return true if there are more elements available, false otherwise.
 */
bool
NavigatorObstaclesListMessage::has_next()
{
  return obstacle_list->has_next();
}


/** Get next plugin from list.
 * @return next plugin from list. This string has been allocated via strndup, so
 * you have to free it yourself!
 */
NavigatorObstaclesListMessage::obstacle_t *
NavigatorObstaclesListMessage::next()
{
  size_t size;
  void *tmp = obstacle_list->next(&size);
  return (obstacle_t *)tmp;
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
