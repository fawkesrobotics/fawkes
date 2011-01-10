
/***************************************************************************
 *  rrdweb_thread.h - RRD Webview thread
 *
 *  Created: Tue Dec 21 01.04:02 2010
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

#ifndef __PLUGINS_RRDWEB_RRDWEB_THREAD_H_
#define __PLUGINS_RRDWEB_RRDWEB_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/clock.h>
#include <aspect/webview.h>
#include <plugins/rrd/aspect/rrd.h>

class RRDWebRequestProcessor;

class RRDWebThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::ClockAspect,
  public fawkes::RRDAspect,
  public fawkes::WebviewAspect
{
 public:
  RRDWebThread();
  virtual ~RRDWebThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  RRDWebRequestProcessor *__processor;
};

#endif
