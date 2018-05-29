
/***************************************************************************
 *  webview-ptzcam-thread.h - CLIPS introspection via webview
 *
 *  Created: Sat Jun 15 19:54:25 2013
 *  Copyright  2006-2014  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_WEBVIEW_PTZCAM_WEBVIEW_PTZCAM_THREAD_H_
#define __PLUGINS_WEBVIEW_PTZCAM_WEBVIEW_PTZCAM_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/clock.h>
#include <aspect/blackboard.h>
#include <aspect/webview.h>
#include <aspect/configurable.h>

namespace fawkes {
  class TimeWait;
  class PanTiltInterface;
  class SwitchInterface;
}

class WebviewPtzCamRequestProcessor;

class WebviewPtzCamThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ClockAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::WebviewAspect
{
 public:
  WebviewPtzCamThread();
  virtual ~WebviewPtzCamThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  WebviewPtzCamRequestProcessor *web_proc_;

  fawkes::TimeWait *time_wait_;
  fawkes::PanTiltInterface *ptu_if_;
  fawkes::SwitchInterface  *power_if_;
  fawkes::SwitchInterface  *camen_if_;

  bool  timeout_;

  float cfg_inactivity_timeout_;
  float cfg_park_pan_tolerance_;
  float cfg_park_pan_pos_;
  float cfg_park_tilt_tolerance_;
  float cfg_park_tilt_pos_;

};

#endif
