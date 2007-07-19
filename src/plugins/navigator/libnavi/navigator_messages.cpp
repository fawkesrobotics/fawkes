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

#include <netcomm/utils/dynamic_buffer.h>
#include <cstdlib>
#include <cstring>

/** @class NavigatorNodesListMessage plugins/navigator/libnavi/navigator_messages.h
 * XXX
 */

/** Constructor. 
 * @param nodes a list with the nodes for this message
 */
NavigatorNodesListMessage::NavigatorNodesListMessage(std::list<NPoint *> *nodes)
{
  nodes_list = new DynamicBuffer(&(msg.nodes_list));

  std::list<NPoint *>::iterator i;
  npoint_t np;
  for ( i = nodes->begin(); i != nodes->end(); ++i ) {
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
  if (_payload != NULL) {
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
 * XXX
 */

/** Constructor. 
 * @param lines a list with the lines for this message 
 */
NavigatorLinesListMessage::NavigatorLinesListMessage(std::list<NLine *> *lines)
{
  lines_list = new DynamicBuffer(&(msg.lines_list));

  std::list<NLine *>::iterator i;
  nline_t nl;
  for ( i = lines->begin(); i != lines->end(); ++i ) {
    nl.x1 = (*i)->p1.x;
    nl.y1 = (*i)->p1.y;
    nl.x2 = (*i)->p2.x;
    nl.y2 = (*i)->p2.y;
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
  if (_payload != NULL) {
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

