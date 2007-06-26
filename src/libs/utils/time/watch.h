
/***************************************************************************
 *  watch.h - A stopwatch
 *
 *  Generated: Sun June 03 00:44:22 2007
 *  Copyright  2007  Daniel Beck 
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

#ifndef __UTILS_TIME_WATCH_H_
#define __UTILS_TIME_WATCH_H_

#include <utils/time/time.h>

class Watch
{
 public:
  Watch(Clock *clock);
  virtual ~Watch();

  void start(Time* t = 0);
  void stop(Time* t = 0);
  void pause(Time* t = 0);
  void reset();

  Time watch_time();
  Time clock_time();

 private:
  Time watch_start;
  Time watch_stop;

  Time pause_start;
  Time pause_stop;
  Time pause_time;

  Clock* clock;

  bool is_running;
  bool is_paused;
};

#endif /*  __UTILS_TIME_WATCH_H_ */
