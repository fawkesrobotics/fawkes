
/***************************************************************************
 *  clips-webview-thread.h - CLIPS introspection via webview
 *
 *  Created: Sat Jun 15 19:54:25 2013
 *  Copyright  2006-2013  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_CLIPS_WEBVIEW_CLIPS_WEBVIEW_THREAD_H_
#define __PLUGINS_CLIPS_WEBVIEW_CLIPS_WEBVIEW_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/webview.h>
#include <aspect/configurable.h>
#include <plugins/clips/aspect/clips_manager.h>

#include <vector>
#include <string>

class ClipsWebRequestProcessor;

class ClipsWebviewThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::WebviewAspect,
  public fawkes::CLIPSManagerAspect
{
 public:
  ClipsWebviewThread();
  virtual ~ClipsWebviewThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  std::vector<std::string> cfg_proto_dirs_;

  ClipsWebRequestProcessor *web_proc_;
};

#endif
