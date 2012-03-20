
/***************************************************************************
 *  thread_roomba_500.h - Roomba 500 thread
 *
 *  Created: Sun Jan 02 12:47:35 2011
 *  Copyright  2006-2010  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#ifndef __PLUGINS_ROOMBA_THREAD_ROOMBA_500_H_
#define __PLUGINS_ROOMBA_THREAD_ROOMBA_500_H_

#include "roomba_500.h"

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/clock.h>
#include <aspect/blackboard.h>
#include <aspect/thread_producer.h>
#include <core/utils/refptr.h>

namespace fawkes {
  class LedInterface;
  class SwitchInterface;
  class MotorInterface;
  class BatteryInterface;
  class Roomba500Interface;
}

class Roomba500Thread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::ClockAspect,
  public fawkes::BlackBoardAspect
{
 public:
  Roomba500Thread();

  virtual void init();
  virtual void finalize();
  virtual void loop();

  void write_blackboard();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  void close_interfaces();
  float led_process(fawkes::LedInterface *iface);
  void set_mode(Roomba500::Mode mode);

 private:
  fawkes::LedInterface       *__led_if_debris;
  fawkes::LedInterface       *__led_if_spot;
  fawkes::LedInterface       *__led_if_dock;
  fawkes::LedInterface       *__led_if_check_robot;
  fawkes::LedInterface       *__led_if_clean_color;
  fawkes::LedInterface       *__led_if_clean_intensity;
  fawkes::SwitchInterface    *__switch_if_vacuuming;
  fawkes::SwitchInterface    *__switch_if_but_clean;
  fawkes::SwitchInterface    *__switch_if_but_spot;
  fawkes::SwitchInterface    *__switch_if_but_dock;
  fawkes::SwitchInterface    *__switch_if_but_minute;
  fawkes::SwitchInterface    *__switch_if_but_hour;
  fawkes::SwitchInterface    *__switch_if_but_day;
  fawkes::SwitchInterface    *__switch_if_but_schedule;
  fawkes::SwitchInterface    *__switch_if_but_clock;
  //fawkes::MotorInterface     *__motor_if;
  fawkes::BatteryInterface   *__battery_if;
  fawkes::Roomba500Interface *__roomba500_if;

  fawkes::RefPtr<Roomba500>   __roomba;

  std::string  __cfg_conntype;
  std::string  __cfg_mode;
  std::string  __cfg_device;
  std::string  __cfg_bttype;
  bool         __cfg_btsave;
  bool         __cfg_btfast;
  bool         __cfg_query_mode;
  bool         __cfg_play_fanfare;

  unsigned int __greeting_loop_count;

  int          __battery_percent;

  class WorkerThread;
  WorkerThread *__wt;
};

#endif
