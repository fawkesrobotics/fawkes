
/***************************************************************************
 *  clock.h - A central clock
 *
 *  Generated: Sun June 03 00:16:29 2007
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

#ifndef __UTILS_SYSTEM_CLOCK_H_
#define __UTILS_SYSTEM_CLOCK_H_

#include <utils/system/time.h>

class TimeSource;

class Clock
{
 public:

  /** Select the time source. */
  typedef enum {
    DEFAULT,		/**< select the default time source */
    REALTIME,		/**< select the system time source */
    EXTERNAL   		/**< select the external time source */
  } TimesourceSelector;

  virtual ~Clock();
  
  static Clock* init();
  static void finalize();

  void register_ext_timesource(TimeSource* ts, bool make_default = false);
  void make_ext_default_timesource();
  bool is_ext_default_timesource() const;

  bool get_time(struct timeval* tv, TimesourceSelector sel = DEFAULT) const;

  Time ext_to_realtime(const Time& t);

  bool has_ext_timesource() const;
 private:
  Clock();

  static bool destruct_ok;

  TimeSource* ext_timesource;
  bool ext_default;

  static Clock* instance;
};

#endif /* __UTILS_SYSTEM_CLOCK_H_ */
