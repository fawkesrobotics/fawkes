
/***************************************************************************
 *  main_thread.h - Fawkes main thread
 *
 *  Created: Thu Nov  2 16:46:37 2006
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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
#include <aspect/mainloop/employer.h>

namespace fawkes {
  class ArgumentParser;
  class LocalBlackBoard;
  class Configuration;
  class ConfigNetworkHandler;
  class MultiLogger;
  class NetworkLogger;
  class Clock;
  class TimeWait;
  class AspectIniFin;
  class PluginManager;
}
class FawkesThreadManager;
class FawkesNetworkManager;

class FawkesMainThread
: public fawkes::Thread,
  public fawkes::MainLoopEmployer,
  public fawkes::MainLoop
{
 public:
  FawkesMainThread(fawkes::ArgumentParser *argp);
  virtual ~FawkesMainThread();

  virtual void once();
  virtual void loop();

  virtual void mloop();
  virtual void set_mainloop(fawkes::MainLoop *mainloop);

 private:
  void destruct();

  fawkes::ArgumentParser       *__argp;
  fawkes::Configuration        *__config;
  fawkes::ConfigNetworkHandler *__config_nethandler;
  fawkes::LocalBlackBoard      *__blackboard;
  fawkes::MultiLogger          *__multi_logger;
  fawkes::NetworkLogger        *__network_logger;
  fawkes::Clock                *__clock;
  fawkes::TimeWait             *__time_wait;
  fawkes::AspectIniFin         *__aspect_inifin;
  fawkes::MainLoop             *__mainloop;

  FawkesThreadManager        *thread_manager;
  fawkes::PluginManager      *plugin_manager;
  FawkesNetworkManager       *network_manager;

};

#endif
