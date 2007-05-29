
/***************************************************************************
 *  fieldinfo_sender.h - Field info sender, this is used to send field
 *                       information (like robot pose, ball position etc.)
 *                       via a message queue
 *
 *  Generated: Fri Apr 21 17:08:10 2006
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_FVUTILS_IPC_FIELDINFO_SENDER_H_
#define __FIREVISION_FVUTILS_IPC_FIELDINFO_SENDER_H_

#include <fvutils/base/types.h>

/** Fountain Control Message */
#define FIELDINFO_MTYPE_FIELDINFO    201

/** Field info message type. */
typedef struct {
  long         mtype;		/**< message type */
  fieldinfo_t  fieldinfo;	/**< field info */
} fieldinfo_msg_t;

class IPCMessageQueue;

class FieldInfoSender {

 public:
  FieldInfoSender(int id);
  ~FieldInfoSender();

  bool hasListeners();
  void send(fieldinfo_t *fieldinfo);
  void send( source_t     source,
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
	     );

 private:
  IPCMessageQueue  *mq;

};


#endif
