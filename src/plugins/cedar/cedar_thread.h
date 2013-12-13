
/***************************************************************************
 *  cedar_thread.h - CLIPS-based Error Detection, Analysis, and Recovery
 *
 *  Created: Fri Aug 16 18:00:32 2013 +0200
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_CEDAR_CEDAR_THREAD_H_
#define __PLUGINS_CEDAR_CEDAR_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <plugins/clips/aspect/clips.h>
#include <utils/time/time.h>

#include <clipsmm.h>

#include <map>
#include <string>

class CedarPluginDirectorThread;

class CedarThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::ClockAspect,
  public fawkes::CLIPSAspect
{
 public:
  CedarThread(CedarPluginDirectorThread *pdt);
  virtual ~CedarThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  void clips_get_plugin_info();

 private:
  CedarPluginDirectorThread *pdt_;
};

#endif
