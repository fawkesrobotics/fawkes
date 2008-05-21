
/***************************************************************************
 *  fieldinfo_sender.h - Field info sender, this is used to send field
 *                       information (like robot pose, ball position etc.)
 *                       via a message queue
 *
 *  Generated: Fri Apr 21 17:09:27 2006
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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

#include <fvutils/ipc/fieldinfo_sender.h>
#include <utils/system/console_colors.h>
#include <utils/ipc/msg.h>

#include <cstring>

using namespace fawkes;

/** @class FieldInfoSender <fvutils/ipc/fieldinfo_sender.h>
 * Send field information via IPCMessageQueue.
 * This class can be used to send vision-relevant field information via an
 * IPC message queue.
 * The message queue is not opened. Rather we look if the desired message
 * queue exists. If this is the case we send the information. 
 *
 * @author Tim Niemueller
 */

/** Constructor.
 * @param id Message queue ID
 * @see IPCMessageQueue
 */
FieldInfoSender::FieldInfoSender(int id)
{
  mq = new IPCMessageQueue(id);
}


/** Destructor. */
FieldInfoSender::~FieldInfoSender()
{
  delete mq;
}


/** Check if there are listeners.
 * @return true if there are listeners, false otherwise
 */
bool
FieldInfoSender::hasListeners()
{
  return mq->isValid();
}


/** Send field info.
 * @param fieldinfo the field info to send in a fieldinfo_t struct.
 */
void
FieldInfoSender::send(fieldinfo_t *fieldinfo)
{
  if (! mq->isValid()) {
    return;
  }

  fieldinfo_msg_t fim;
  fim.mtype = FIELDINFO_MTYPE_FIELDINFO;
  memcpy(&(fim.fieldinfo), fieldinfo, sizeof(fieldinfo_t));

  mq->send((IPCMessageQueue::MessageStruct *)&fim, sizeof(fim));
}


/** Send field information.
 * Constructs a fieldinfo_t struct with the given information and sends it out.
 * @param source info source
 * @param time_sec timestamp seconds
 * @param time_usec timestamp microseconds
 * @param own_goal_color own goal color
 * @param opp_goal_color opponent goal color
 * @param own_head_color own head color
 * @param pose_x pose x
 * @param pose_y pose y
 * @param pose_ori pose ori
 * @param pan pan
 * @param tilt tilt
 * @param tracking_mode tracking mode
 * @param ball_rel_x ball relative x position
 * @param ball_rel_y ball relative y position
 * @param ball_glob_x ball global x position
 * @param ball_glob_y ball global y position
 * @param ball_rel_vel_x ball relative velocity in x direction
 * @param ball_rel_vel_y ball relative velocity in y direction
 * @param ball_glob_vel_x ball global velocity in x direction
 * @param ball_glob_vel_y ball global velocity in y direction
 * @param rel_vel_available true if relative velocity available
 * @param glob_vel_available true if global velocity available
 * @param ball_visible true if ball visible
 */
void
FieldInfoSender::send( source_t     source,
		       long int     time_sec,
		       long int     time_usec,
		       color_t      own_goal_color,
		       color_t      opp_goal_color,
		       color_t      own_head_color,
		       float        pose_x,
		       float        pose_y,
		       float        pose_ori,
		       float        pan,
		       float        tilt,
		       int          tracking_mode,
		       float        ball_rel_x,
		       float        ball_rel_y,
		       float        ball_glob_x,
		       float        ball_glob_y,
		       float        ball_rel_vel_x,
		       float        ball_rel_vel_y,
		       float        ball_glob_vel_x,
		       float        ball_glob_vel_y,
		       bool         rel_vel_available,
		       bool         glob_vel_available,
		       bool         ball_visible
		       )
{
  fieldinfo_t fieldinfo;

  fieldinfo.source =               source;
  fieldinfo.time_sec =             time_sec;
  fieldinfo.time_usec =            time_usec;
  fieldinfo.own_goal_color =       own_goal_color;
  fieldinfo.opp_goal_color =       opp_goal_color;
  fieldinfo.own_head_color =       own_head_color;
  fieldinfo.pose_x =               pose_x;
  fieldinfo.pose_y =               pose_y;
  fieldinfo.pose_ori =             pose_ori;
  fieldinfo.pan =                  pan;
  fieldinfo.tilt =                 tilt;
  fieldinfo.tracking_mode =        tracking_mode;
  fieldinfo.ball_rel_x =           ball_rel_x;
  fieldinfo.ball_rel_y =           ball_rel_y;
  fieldinfo.ball_glob_x =          ball_glob_x;
  fieldinfo.ball_glob_y =          ball_glob_y;
  fieldinfo.ball_rel_vel_x =       ball_rel_vel_x;
  fieldinfo.ball_rel_vel_y =       ball_rel_vel_y;
  fieldinfo.ball_glob_vel_x =      ball_glob_vel_x;
  fieldinfo.ball_glob_vel_y =      ball_glob_vel_y;
  fieldinfo.rel_vel_available =    rel_vel_available;
  fieldinfo.glob_vel_available =   glob_vel_available;
  fieldinfo.ball_visible =         ball_visible;

  send( &fieldinfo );
}
