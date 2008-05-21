
/***************************************************************************
 *  main_thread.h - Fawkes main thread
 *
 *  Created: Thu Nov  2 16:46:37 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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

#ifndef __FAWKES_MAIN_THREAD_H_
#define __FAWKES_MAIN_THREAD_H_

#include <core/threading/thread.h>

namespace fawkes {
  class ArgumentParser;
  class BlackBoard;
  class Configuration;
  class MultiLogger;
  class NetworkLogger;
  class HostInfo;
  class Clock;
  class TimeWait;
#ifdef USE_TIMETRACKER
  class TimeTracker;
#endif
}
class FawkesThreadManager;
class FawkesPluginManager;
class FawkesNetworkManager;
class FawkesThreadIniFin;
class FawkesConfigManager;

class FawkesMainThread : public fawkes::Thread
{
 public:
  FawkesMainThread(fawkes::ArgumentParser *argp);
  virtual ~FawkesMainThread();

  virtual void once();
  virtual void loop();

 private:
  void destruct();

  fawkes::ArgumentParser     *argp;
  fawkes::Configuration      *config;
  fawkes::BlackBoard         *blackboard;
  fawkes::HostInfo           *hostinfo;
  fawkes::MultiLogger        *multi_logger;
  fawkes::NetworkLogger      *network_logger;
  fawkes::Clock              *clock;
  fawkes::TimeWait           *__time_wait;

  FawkesThreadManager        *thread_manager;
  FawkesThreadIniFin         *thread_inifin;
  FawkesPluginManager        *plugin_manager;
  FawkesNetworkManager       *network_manager;
  FawkesConfigManager        *config_manager;

  char *config_mutable_file;
  const char *config_default_file;

#ifdef USE_TIMETRACKER
  fawkes::TimeTracker  *__tt;
  unsigned int  __tt_loopcount;
  unsigned int  __ttc_pre_loop;
  unsigned int  __ttc_sensor;
  unsigned int  __ttc_worldstate;
  unsigned int  __ttc_think;
  unsigned int  __ttc_skill;
  unsigned int  __ttc_act;
  unsigned int  __ttc_post_loop;
  unsigned int  __ttc_netproc;
  unsigned int  __ttc_full_loop;
  unsigned int  __ttc_real_loop;
#endif
};

#endif
