/***************************************************************************
 *  navigator_messages.h - Navigator Messages
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

#ifndef __NAVIGATOR_LIBNAVI_MESSAGES_H_
#define __NAVIGATOR_LIBNAVI_MESSAGES_H_

#include <netcomm/utils/dynamic_buffer.h>
#include <netcomm/fawkes/message_content.h>
#include <stdint.h>
#include <list>

class NPoint;
class NLine;

#define NAVIGATOR_MSGTYPE_JOYSTICK                      1
#define NAVIGATOR_MSGTYPE_GUI                                   2
#define NAVIGATOR_MSGTYPE_SUBSCRIBE                             3
#define NAVIGATOR_MSGTYPE_UNSUBSCRIBE                           4
#define NAVIGATOR_MSGTYPE_NODES                                 5
#define NAVIGATOR_MSGTYPE_LINES                                 6
#define NAVIGATOR_MSGTYPE_TARGET                                7
#define NAVIGATOR_MSGTYPE_CONTROL_SUBERR                        8
#define NAVIGATOR_MSGTYPE_VELOCITY                                      9
#define NAVIGATOR_MSGTYPE_DRIVE                                         10
#define NAVIGATOR_MSGTYPE_KICK                                          11

/** The message type of the kick messages.
 */
typedef struct { 
  bool left;    /**< Left kick command.*/
  bool center;  /**< Center kick command.*/ 
  bool right;   /**< Right kick command.*/
} navigator_kick_message_t;

/** The message type of the joystick tool messages.
 */
typedef struct { 
  float forward;        /**< Forward command.*/
  float sideward;       /**< Sideward command.*/ 
  float rotation;       /**< Rotation command.*/
  float speed;       /**< Speed command.*/
} navigator_joystick_message_t;

/** The message type of driving commands.
 */
typedef struct { 
  float forward;        /**< Forward command.*/
  float sideward;       /**< Sideward command.*/ 
  float rotation;       /**< Rotation command.*/
  float velocity;       /**< Velocity command.*/
} navigator_drive_message_t;

/** The message type of the navigator gui tool messages.
 */
typedef struct {
  unsigned int numberPoints;  /**< Number of points of the delaunay. */
  double pointsX[100]; /**< The x-coordinates of the points of the delaunay. */
  double pointsY[100];  /**< The y-coordinates of the points of the delaunay. */
  unsigned int numberLines;  /**< Number of lines of the delaunay. */
  unsigned int lines[100];  /**< index point1 of line1; index point2 of line1; index point1 of line2... */
  float target; /**< The target to drive to. */
} navigator_gui_message_t;


/** The message type to set a target to the navigator.
 */
typedef struct {
  float x;  /**< X-coordinate of the target. */
  float y;  /**< Y-coordinate of the target. */
} navigator_target_message_t;



/** The message type to set the velocity to the navigator.
 */
typedef struct {
  float value;  /**< The velocity value. */
} navigator_velocity_message_t;

/** Navigator list message.
 * Message type ID is NAVIGATOR_MSGTYPE_NODES.
 */
typedef struct {
  dynamic_list_t nodes_list;    /**< dynamically growing list of nodes */
} navigator_nodes_msg_t;

/** Navigator list message.
 * Message type ID is NAVIGATOR_MSGTYPE_LINES.
 */
typedef struct {
  dynamic_list_t lines_list;    /**< dynamically growing list of nodes */
} navigator_lines_msg_t;

/** The message type for subscribe messages to subscribe messages
 *   from the navigator network thread.
 */
typedef struct {
  uint32_t sub_type_points_and_lines:1; /**< This bit states the subscribe type points and lines. */
  uint32_t sub_type_joystick:1; /**< This bit states the subscribe type joystick. */
  uint32_t sub_type_control:1; /**< This bit states the subscribe type control. */
  uint32_t reserved:29;  /**< Reserved bits for future work. */
} navigator_subscribe_message_t;

/** The message type for subscribe messages to subscribe messages
 *   from the navigator network thread.
 */
typedef struct {
  uint32_t unsub_type_points_and_lines:1; /**< This bit states the unsubscribe type points and lines. */
  uint32_t unsub_type_joystick:1; /**< This bit states the unsubscribe type joystick. */
  uint32_t unsub_type_control:1; /**< This bit states the subscribe type control. */
  uint32_t reserved:29; /**< Reserved bits for future work. */
} navigator_unsubscribe_message_t;


class NavigatorNodesListMessage : public FawkesNetworkMessageContent
{
 public:
  NavigatorNodesListMessage(std::list<NPoint *> *nodes);
  NavigatorNodesListMessage(unsigned int component_id, unsigned int msg_id,
                            void *payload, size_t payload_size);
  virtual ~NavigatorNodesListMessage();

  virtual void serialize();

  typedef struct {
    float x; /**< x-coordinate */
    float y; /**< y-coordinate */
  } npoint_t;

  void        reset_iterator();
  bool        has_next();
  npoint_t *  next();
  
 private:
  DynamicBuffer     *nodes_list;
  navigator_nodes_msg_t  msg;
};

class NavigatorLinesListMessage : public FawkesNetworkMessageContent
{
 public:
  NavigatorLinesListMessage(std::list<NLine *> *lines);
  NavigatorLinesListMessage(unsigned int component_id, unsigned int msg_id,
                            void *payload, size_t payload_size);
  virtual ~NavigatorLinesListMessage();

  virtual void serialize();

  typedef struct {
    float x1; /**< x-coordinate of point 1 */
    float y1; /**< y-coordinate of point 1 */
    float x2; /**< x-coordinate of point 2 */
    float y2; /**< y-coordinate of point 2 */
  } nline_t;

  void        reset_iterator();
  bool        has_next();
  nline_t *  next();
  
 private:
  DynamicBuffer     *lines_list;
  navigator_lines_msg_t  msg;
};


#endif /*NAVIGATOR_MESSAGES_H_*/
