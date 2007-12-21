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

#ifndef __NAVIGATOR_LIBNAVI_NAVIGATOR_MESSAGES_H_
#define __NAVIGATOR_LIBNAVI_NAVIGATOR_MESSAGES_H_

#include <netcomm/utils/dynamic_buffer.h>
#include <netcomm/fawkes/message_content.h>
#include <stdint.h>
#include <list>

class NPoint;
class NLine;
class Obstacle;

#define NAVIGATOR_MSGTYPE_JOYSTICK                      1
#define NAVIGATOR_MSGTYPE_GUI                                   2
#define NAVIGATOR_MSGTYPE_SUBSCRIBE                             3
#define NAVIGATOR_MSGTYPE_UNSUBSCRIBE                           4
#define NAVIGATOR_MSGTYPE_SURFACE                                 5
#define NAVIGATOR_MSGTYPE_TARGET                                6
#define NAVIGATOR_MSGTYPE_CONTROL_SUBERR                        7
#define NAVIGATOR_MSGTYPE_VELOCITY                                      8
#define NAVIGATOR_MSGTYPE_TRANS_ROT                                 9
#define NAVIGATOR_MSGTYPE_RPM                                 10
#define NAVIGATOR_MSGTYPE_ORBIT                              11
#define NAVIGATOR_MSGTYPE_KICK                                          12
#define NAVIGATOR_MSGTYPE_ODOMETRY                      13
#define NAVIGATOR_MSGTYPE_RESET_ODOMETRY                      14
#define NAVIGATOR_MSGTYPE_BALL                      15
#define NAVIGATOR_MSGTYPE_PATH                     16
#define NAVIGATOR_MSGTYPE_OBSTACLE                     17

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
  uint32_t type_trans_rot:1; /**< This bit states the trans rot motor command. */
  uint32_t type_trans:1; /**< This bit states the trans motor command. */
  uint32_t type_rot:1; /**< This bit states the rot motor command. */
  uint32_t reserved:29;  /**< Reserved bits for future work. */
  float forward;        /**< Forward command.*/
  float sideward;       /**< Sideward command.*/ 
  float rotation;       /**< Rotation command.*/
} navigator_trans_rot_message_t;

/** The message type of driving commands.
 */
typedef struct { 
  float left;        /**<RPM of the left wheel.*/
  float rear;       /**< RPM of the rear wheel.*/ 
  float right;       /**< RPM of the right wheel.*/
} navigator_rpm_message_t;

/** The message type of driving commands.
 */
typedef struct { 
  float orbit_center_x;        /**<The center of the orbit.*/
  float orbit_center_y;       /**< The center of the orbit.*/ 
  float angular_velocity;       /**< The angular velocity of the robot in orbit.*/
} navigator_orbit_message_t;

/** The message type of the navigator gui tool messages.
 * Message type ID is NAVIGATOR_MSGTYPE_GUI.
 */
typedef struct {
  unsigned int numberPoints;  /**< Number of points of the delaunay. */
  double pointsX[100]; /**< The x-coordinates of the points of the delaunay. */
  double pointsY[100];  /**< The y-coordinates of the points of the delaunay. */
  unsigned int numberLines;  /**< Number of lines of the delaunay. */
  unsigned int lines[100];  /**< index point1 of line1; index point2 of line1; index point1 of line2... */
  uint32_t control_type_rpm:1; /**< controls the motors directly with RPMs */
  uint32_t control_type_trans_rot:1; /**< controls the motors directly with trans_rot command */
  uint32_t control_type_trans:1; /**< controls the motors directly with trans command */
  uint32_t control_type_rot:1; /**< controls the motors directly with rot command */
  uint32_t control_type_orbit:1; /**< controls the motors directly with orbit command */
  uint32_t control_type_navigator:1; /**< controls the robot with the navigator */
  uint32_t control_type_reset_odometry:1; /**< resets the recalculation of the odometry */
  uint32_t reserved:25; /**< Reserved bits for future work. */
  float target_x; /**< The x-coordiante of the target to drive to. */
  float target_y; /**< The y-coordiante of the target to drive to. */
  float target_ori; /**< The orientation of the robot at the target. */
  float velocity; /**< The velocity of the robot. */
  float target; /**< The target to drive to. */
} navigator_gui_message_t;

/** The message type to send the odometry to the odometry subscriber.
 * Message type ID is NAVIGATOR_MSGTYPE_ODOMETRY.
 */
typedef struct {
  float path_length; /**< The path length the robot traveled till the last reset. */
  float position_x; /**< The x-coordinate of the supposed position of the robot. */
  float position_y; /**< The y-coordinate of the supposed position of the robot. */
  float orientation; /**< The supposed orientation of the robot. */
  float rpm_left; /**< The RPM of the left front wheel of the robot. */
  float rpm_rear; /**< The RPM of the rear wheel of the robot. */
  float rpm_right; /**< The RPM of the right front wheel of the robot. */
  float velocity_x; /**< The velocity towards x. */
  float velocity_y; /**< The velocity towards y. */
  float velocity_rotation; /**< the velocity of the rotation. */
} navigator_odometry_message_t;

/** The message type to set a target to the navigator.
 * Message type ID is NAVIGATOR_MSGTYPE_TARGET.
 */
typedef struct {
  float x;  /**< X-coordinate of the target. */
  float y;  /**< Y-coordinate of the target. */
} navigator_target_message_t;

/** The message type to determine the ball position.
 * Message type ID is NAVIGATOR_MSGTYPE_BALL.
 */
typedef struct {
  float x;  /**< X-coordinate of the ball. */
  float y;  /**< Y-coordinate of the ball. */
} navigator_ball_message_t;

/** The message type to determine an obstacle.
 * Message type ID is NAVIGATOR_MSGTYPE_OBSTACLE.
 */
typedef struct {
  float x;  /**< X-coordinate of the obstacle. */
  float y;  /**< Y-coordinate of the obstacle. */
  float width;  /**< width of the obstacle. */
} navigator_obstacle_msg_t;


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
 * Message type ID is NAVIGATOR_MSGTYPE_SURFACE.
 */
typedef struct {
  dynamic_list_t lines_list;    /**< dynamically growing list of lines */
} navigator_lines_msg_t;

/** Navigator list message.
 * Message type ID is NAVIGATOR_MSGTYPE_PATH.
 */
typedef struct {
  dynamic_list_t path_list;    /**< dynamically growing list of path nodes */
} navigator_path_msg_t;

/** The message type to determine the obstacles.
 * Message type ID is NAVIGATOR_MSGTYPE_OBSTACLES_LIST.
 */
typedef struct {
  dynamic_list_t obstacles_list;   /**< dynamically growing list of obstacles */
} navigator_obstacles_list_msg_t;

/** The message type for subscribe messages to subscribe messages
 *   from the navigator network thread.
 */
typedef struct {
  uint32_t sub_type_points_and_lines:1; /**< This bit states the subscribe type points and lines. */
  uint32_t sub_type_joystick:1; /**< This bit states the subscribe type joystick. */
  uint32_t sub_type_motor_control:1; /**< This bit states the subscribe type motor control. */
  uint32_t sub_type_navigator_control:1; /**< This bit states the subscribe type navigator control. */
  uint32_t sub_type_odometry:1; /**< This bit states the subscribe type odometry. */
  uint32_t reserved:27;  /**< Reserved bits for future work. */
} navigator_subscribe_message_t;

/** The message type for subscribe messages to subscribe messages
 *   from the navigator network thread.
 */
typedef struct {
  uint32_t unsub_type_points_and_lines:1; /**< This bit states the unsubscribe type points and lines. */
  uint32_t unsub_type_joystick:1; /**< This bit states the unsubscribe type joystick. */
  uint32_t unsub_type_motor_control:1; /**< This bit states the subscribe type motor control. */
  uint32_t unsub_type_navigator_control:1; /**< This bit states the subscribe type navigator control. */
  uint32_t unsub_type_odometry:1; /**< This bit states the subscribe type odometry. */
  uint32_t reserved:27; /**< Reserved bits for future work. */
} navigator_unsubscribe_message_t;


class NavigatorSurfaceMessage : public FawkesNetworkMessageContent
{
 public:
  NavigatorSurfaceMessage(std::list<NLine *> *lines);
  NavigatorSurfaceMessage(unsigned int component_id, unsigned int msg_id,
                            void *payload, size_t payload_size);
  virtual ~NavigatorSurfaceMessage();

  virtual void serialize();

  typedef struct {
    float x1; /**< x-coordinate of point 1 */
    float y1; /**< y-coordinate of point 1 */
    float width1; /**< width of the obstacle1, 0 if point 1 is not an obstacle */
    float x2; /**< x-coordinate of point 2 */
    float y2; /**< y-coordinate of point 2 */
    float width2; /**< width of the obstacle2, 0 if point 2 is not an obstacle */
  } nline_t;

  void        reset_iterator();
  bool        has_next();
  nline_t *  next();
  
 private:
  DynamicBuffer     *lines_list;
  navigator_lines_msg_t  msg;
};

class NavigatorPathListMessage : public FawkesNetworkMessageContent
{
 public:
  NavigatorPathListMessage(std::list<NPoint *> *points);
  NavigatorPathListMessage(unsigned int component_id, unsigned int msg_id,
                            void *payload, size_t payload_size);
  virtual ~NavigatorPathListMessage();

  virtual void serialize();

  typedef struct {
    float x; /**< x-coordinate */
    float y; /**< y-coordinate */
  } npoint_t;

  void        reset_iterator();
  bool        has_next();
  npoint_t *  next();
  
 private:
  DynamicBuffer     *path_list;
  navigator_path_msg_t  msg;
};


#endif /*NAVIGATOR_MESSAGES_H_*/
