
/***************************************************************************
 *  clock.h - A central clock
 *
 *  Generated: Sun Jun 03 00:16:29 2007
 *  Copyright  2007  Daniel Beck 
 *             2007  Tim Niemueller [www.niemueller.de]
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

#ifndef __UTILS_TIME_CLOCK_H_
#define __UTILS_TIME_CLOCK_H_

#include <utils/time/time.h>

namespace fawkes {

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
  
  static Clock * instance();
  static void    finalize();

  void register_ext_timesource(TimeSource* ts, bool make_default = false);
  void set_ext_default_timesource(bool ext_is_default);
  bool is_ext_default_timesource() const;
  bool has_ext_timesource() const;
  Time ext_to_realtime(const Time& t);
  Time native_to_time(const Time &t);
  void remove_ext_timesource(TimeSource *ts = 0);

  void get_time(struct timeval *tv) const;
  void get_time(struct timeval *tv, TimesourceSelector sel) const;

  void get_time(Time &time) const;
  void get_time(Time &time, TimesourceSelector sel) const;

  void get_time(Time *time) const;
  void get_time(Time *time, TimesourceSelector sel) const;

  void get_systime(struct timeval *tv) const;
  void get_systime(Time &time) const;
  void get_systime(Time *time) const;

  Time  now() const;
  float elapsed(Time *t) const;
  float sys_elapsed(Time *t) const;

 private:
  Clock();

  TimeSource *ext_timesource;
  bool ext_default;

  static Clock* _instance;
};

} // end namespace fawkes

#endif /* __UTILS_TIME_CLOCK_H_ */
