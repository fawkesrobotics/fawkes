
/***************************************************************************
 *  openprs_agent_thread.h - OpenPRS agent thread
 *
 *  Created: Fri Aug 22 13:56:36 2014
 *  Copyright  2014  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_OPENPRS_AGENT_OPENPRS_AGENT_THREAD_H_
#define __PLUGINS_OPENPRS_AGENT_OPENPRS_AGENT_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/clock.h>
#include <aspect/blocked_timing.h>
#include <aspect/configurable.h>
#include <plugins/openprs/aspect/openprs.h>

#include <string>

class OpenPRSAgentThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ClockAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlockedTimingAspect,
  public fawkes::OpenPRSAspect
{
 public:
  OpenPRSAgentThread(OpenPRSAspect::Mode oprs_mode, bool gdb_delay);
  virtual ~OpenPRSAgentThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private: // methods
  void handle_message(std::string sender, std::string message);

 private: // members
  std::string cfg_agent_;
  bool        agent_alive_;
};

#endif
