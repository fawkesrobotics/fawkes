
/***************************************************************************
 *  handler.cpp - World Info Handler
 *
 *  Created: Sun Jan 21 15:57:22 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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

#include <netcomm/worldinfo/handler.h>

/** @class WorldInfoHandler netcomm/worldinfo/handler.h
 * World info handler.
 * This interface defines methods called by WorldInfoTransceiver for incoming
 * information.
 *
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
 * @fn void WorldInfoHandler::ball_pos_rcvd(const char *from_host, float dist, float pitch, float yaw, float *covariance)
 * Ball position information received.
 * @param from_host transmitting host of this information, if available symbolic name
 * @param dist distance to ball in meters
 * @param pitch pitch angle to ball
 * @param yaw yaw angle to ball
 * @param covariance covariance matrix with 9 entries, ordered as three concatenated
 * rows (first row, three floats, second row, three floats, third row, three
 * floats).
 * @see WorldInfoTransceiver::set_ball_pos()
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
 * @fn void WorldInfoHandler::opponent_pose_rcvd(const char *from_host, float distance, float angle, , float *covariance)
 * Opponent information received.
 * @param from_host transmitting host of this information, if available symbolic name
 * @param distance to opponent
 * @param angle angle to opponent (angle is zero if opponent is in front of robot,
 * positive if right of robot, negative if left of robot).
 * @param covariance covariance matrix with 4 entries, ordered as two concatenated
 * rows (first row, two floats, second row, two floats)
 * @see WorldInfoTransceiver::add_opponent()
 *
 */

/** Virtual empty destructor. */
WorldInfoHandler::~WorldInfoHandler()
{
}


