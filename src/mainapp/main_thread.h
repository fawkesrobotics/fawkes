
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

class ArgumentParser;
class BlackBoard;
class FawkesThreadManager;
class FawkesPluginManager;
class FawkesNetworkManager;
class FawkesThreadIniFin;
class FawkesConfigManager;
class Configuration;
class MultiLogger;
class NetworkLogger;
class HostInfo;
class Clock;
class TimeWait;
#ifdef USE_TIMETRACKER
class TimeTracker;
#endif

class FawkesMainThread : public Thread
{
 public:
  FawkesMainThread(ArgumentParser *argp);
  virtual ~FawkesMainThread();

  virtual void once();
  virtual void loop();

 private:
  void destruct();

  ArgumentParser             *argp;
  Configuration              *config;
  BlackBoard                 *blackboard;
  HostInfo                   *hostinfo;
  FawkesThreadManager        *thread_manager;
  FawkesThreadIniFin         *thread_inifin;
  FawkesPluginManager        *plugin_manager;
  FawkesNetworkManager       *network_manager;
  FawkesConfigManager        *config_manager;
  MultiLogger                *multi_logger;
  NetworkLogger              *network_logger;
  Clock                      *clock;
  TimeWait                   *__time_wait;

  char *config_mutable_file;
  const char *config_default_file;

#ifdef USE_TIMETRACKER
  TimeTracker  *__tt;
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
