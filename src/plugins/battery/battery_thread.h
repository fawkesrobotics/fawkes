
/***************************************************************************
 *  battery_plugin.h - Fawkes Battery Thread
 *
 *  Generated: Tue Jan 29 11:56:28 2008
 *  Copyright  2008  Daniel Beck
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

#ifndef __PLUGINS_BATTERY_BATTERY_THREAD_H
#define __PLUGINS_BATTERY_BATTERY_THREAD_H

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/configurable.h>
#include <aspect/clock.h>

#include <termios.h>

class BatteryInterface;
class TimeWait;

class BatteryThread : public Thread,
  public LoggingAspect,
  public BlackBoardAspect,
  public ConfigurableAspect,
  public ClockAspect
{
 public:
  BatteryThread();
  virtual ~BatteryThread();

  virtual void finalize();
  virtual void init();
  virtual void loop();

 private:
  bool send_command(unsigned char cmd);
  unsigned int read_numeric(unsigned char cmd);
  /*  char* read_string(unsigned char cmd); */

  BatteryInterface* m_battery_interface;
  TimeWait* m_time_wait;

  unsigned int m_interval;
  char* m_port;
  int m_fd;
  struct termios m_old_options;
};

#endif /* __PLUGINS_BATTERY_BATTERY_THREAD_H */
