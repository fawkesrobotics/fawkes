/***************************************************************************
 *  messag_handler_thread.h - Colli Message Handler Thread
 *
 *  Created: Thu Oct 17 16:58:00 2013
 *  Copyright  2013  AllemaniACs
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

#ifndef __PLUGINS_COLLI_MESSAGE_HANDLER_THREAD_H_
#define __PLUGINS_COLLI_MESSAGE_HANDLER_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/configurable.h>

namespace fawkes
{
  class NavigatorInterface;
}

class ColliMessageHandlerThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::ConfigurableAspect
{
 public:
  ColliMessageHandlerThread();
  virtual ~ColliMessageHandlerThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 private:

  fawkes::NavigatorInterface* if_navi_;

  float cfg_security_distance_;
  float cfg_max_velocity_;
  float cfg_escaping_enabled_;
};

#endif

