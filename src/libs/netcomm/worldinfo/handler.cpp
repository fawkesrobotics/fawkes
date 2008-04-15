
/***************************************************************************
 *  handler.cpp - World Info Handler
 *
 *  Created: Sun Jan 21 15:57:22 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#include <netcomm/worldinfo/handler.h>

/** @class WorldInfoHandler netcomm/worldinfo/handler.h
 * World info handler.
 * This interface defines methods called by WorldInfoTransceiver for incoming
 * information.
 *
 * @ingroup NetComm
 * @author Tim Niemueller
 *
 *
 * @fn void WorldInfoHandler::pose_rcvd(const char *from_host, float x, float y, float theta, float *covariance)
 * Pose information received.
 * @param from_host transmitting host of this information, if available symbolic name
 * @param x x position
 * @param y y position
 * @param theta rotation of the robot
 * @param covariance covariance matrix, line-wise float array
 *
 * @fn void WorldInfoHandler::velocity_rcvd(const char *from_host, float vel_x, float vel_y, float vel_theta, float *covariance)
 * Robot velocity information received.
 * @param from_host transmitting host of this information, if available symbolic name
 * @param vel_x velocity in x direction
 * @param vel_y velocity in y direction
 * @param vel_theta rotational velocity, positive velocity means clockwise
 * rotation, negative velocity means counter-clockwise.
 * @param covariance covariance matrix with 9 entries, ordered as three concatenated
 * rows (first row, three floats, second row, three floats, third row, three
 * floats).
 * @see WorldInfoTransceiver::set_velocity()
 *
 * @fn void WorldInfoHandler::ball_pos_rcvd(const char *from_host, bool visible, int visibility_history, float dist, float bearing, float slope, float *covariance)
 * Ball position information received.
 * @param from_host transmitting host of this information, if available symbolic name
 * @param visible true if ball is visible, false otherwise. If the ball is not visible
 * the given position is the last known position and may be invalid. Use visibility history
 * to decide whether you expect useful data.
 * @param visibility_history Ball visibility history.
 * @param dist distance to ball in meters
 * @param bearing bearing angle to ball
 * @param slope slope angle to ball
 * @param covariance covariance matrix with 9 entries, ordered as three concatenated
 * rows (first row, three floats, second row, three floats, third row, three
 * floats).
 * @see WorldInfoTransceiver::set_ball_pos()
 * @see WorldInfoTransceiver::set_ball_visible()
 *
 * @fn void WorldInfoHandler::ball_velocity_rcvd(const char *from_host, float vel_x, float vel_y, float vel_z, float *covariance)
 * Ball velocity information received.
 * @param from_host transmitting host of this information, if available symbolic name
 * @param vel_x velocity in x direction
 * @param vel_y velocity in y direction
 * @param vel_z velocity in z direction
 * @param covariance covariance matrix with 9 entries, ordered as three concatenated
 * rows (first row, three floats, second row, three floats, third row, three
 * floats).
 * @see WorldInfoTransceiver::set_ball_velocity()
 *
 * @fn void WorldInfoHandler::opponent_pose_rcvd(const char *from_host, unsigned int uid, float distance, float bearing, , float *covariance)
 * Opponent information received.
 * @param from_host transmitting host of this information, if available symbolic name
 * @param uid unique ID of the opponent
 * @param distance to opponent
 * @param bearing bearing to opponent (angle is zero if opponent is in front of robot,
 * positive if right of robot, negative if left of robot).
 * @param covariance covariance matrix with 4 entries, ordered as two concatenated
 * rows (first row, two floats, second row, two floats)
 * @see WorldInfoTransceiver::add_opponent()
 *
 * @fn void WorldInfoHandler::opponent_disapp_rcvd(const char *from_host, unsigned int uid)
 * Opponent disappeared.
 * @param from_host transmitting host of this information, if available symbolic name
 * @param uid unique ID of the opponent
 *
 * @fn void WorldInfoHandler::gamestate_rcvd(const char *from_host, worldinfo_gamestate_t game_state, worldinfo_gamestate_team_t state_team, unsigned int score_cyan, unsigned int score_magenta, worldinfo_gamestate_team_t our_team, worldinfo_gamestate_goalcolor_t our_goal_color, worldinfo_gamestate_half_t half)
 * Gamestate information received.
 * @param from_host transmitting host of this information, if available symbolic name
 * @param game_state current gamestate
 * @param state_team team related to the game state
 * @param score_cyan current score of team cyan
 * @param score_magenta current score of team magenta
 * @param our_team our team color
 * @param our_goal_color our goal color
 * @param half current half of the game, first or second
 * @see WorldInfoTransceiver::set_gamestate()
 *
 */

/** Virtual empty destructor. */
WorldInfoHandler::~WorldInfoHandler()
{
}


