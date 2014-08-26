
/***************************************************************************
 *  openprs_thread.h - OpenPRS aspect provider thread
 *
 *  Created: Thu Aug 14 15:52:35 2014
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_OPENPRS_OPENPRS_THREAD_H_
#define __PLUGINS_OPENPRS_OPENPRS_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/aspect_provider.h>
#include <aspect/logging.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
// include <plugins/openprs/aspect/openprs_inifin.h>

#include <string>

namespace fawkes {
  class AspectIniFin;
  class SubProcess;
}

class OpenPRSThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ClockAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::BlockedTimingAspect
//public fawkes::AspectProviderAspect
{
 public:
  OpenPRSThread();
  virtual ~OpenPRSThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private: // methods

 private: // members
  bool         cfg_mp_run_;
  std::string  cfg_mp_bin_;
  std::string  cfg_mp_port_;
  bool         cfg_server_run_;
  std::string  cfg_server_bin_;
  std::string  cfg_server_port_;

  //fawkes::OpenPRSAspectIniFin openprs_aspect_inifin_;

  fawkes::SubProcess *proc_mp_;
  fawkes::SubProcess *proc_srv_;


};

#endif
