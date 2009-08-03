
/***************************************************************************
 *  engine_thread.h - Thread driving the XABSL Engine
 *
 *  Created: Thu Aug 07 14:49:11 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_XABSL_ENGINE_THREAD_H_
#define __PLUGINS_XABSL_ENGINE_THREAD_H_

#include "iface_field_wrapper.h"

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/clock.h>
#include <aspect/blackboard.h>

#include <map>
#include <string>

namespace xabsl {
  class Engine;
}

namespace fawkes {
  class Time;
  class ObjectPositionInterface;
  class SkillerInterface;
}

class XabslLoggingErrorHandler;
class XabslFileInputSource;
class XabslSkillWrapper;

class XabslEngineThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::ClockAspect,
  public fawkes::BlackBoardAspect
{
 public:
  XabslEngineThread();

  virtual void init();
  virtual void finalize();
  virtual void once();
  virtual void loop();

  unsigned long int current_time();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  xabsl::Engine             *__xe;
  XabslLoggingErrorHandler  *__xleh;

  fawkes::Time              *__now;

  fawkes::SkillerInterface        *__skiller_if;
  fawkes::ObjectPositionInterface *__wm_ball_if;
  fawkes::ObjectPositionInterface *__wm_ball_w_if;

  XabslInterfaceFieldWrapper<double, float> *__ball_rx;
  XabslInterfaceFieldWrapper<double, float> *__ball_ry;

  std::map<std::string, XabslSkillWrapper *> __wrappers;
  std::map<std::string, XabslSkillWrapper *>::iterator __wit;
};


#endif
