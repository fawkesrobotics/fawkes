
/***************************************************************************
 *  webview_thread.h - Thread that handles web interface requests
 *
 *  Created: Mon Oct 13 17:49:52 2008 (I5 Developer's Day)
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#ifndef __PLUGINS_WEBVIEW_WEBVIEW_THREAD_H_
#define __PLUGINS_WEBVIEW_WEBVIEW_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>
#include <aspect/network.h>
#include <aspect/logger.h>
#include <aspect/plugin_director.h>

#include <utils/logging/cache.h>

namespace fawkes {
  class NetworkService;
}

class WebRequestDispatcher;
class WebStaticRequestProcessor;
class WebBlackBoardRequestProcessor;
class WebStartPageRequestProcessor;
class WebPluginsRequestProcessor;

class WebviewThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::NetworkAspect,
  public fawkes::LoggerAspect,
  public fawkes::PluginDirectorAspect
{
 public:
  WebviewThread();
  ~WebviewThread();

  virtual void init();
  virtual void finalize();
  virtual void loop();

  static const char *STATIC_URL_PREFIX;
  static const char *BLACKBOARD_URL_PREFIX;
  static const char *PLUGINS_URL_PREFIX;

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  struct MHD_Daemon    *__daemon;
  WebRequestDispatcher *__dispatcher;

  WebStaticRequestProcessor      *__static_processor;
  WebStartPageRequestProcessor   *__startpage_processor;
  WebBlackBoardRequestProcessor  *__blackboard_processor;
  WebPluginsRequestProcessor     *__plugins_processor;

  unsigned int __cfg_port;

  fawkes::CacheLogger     __cache_logger;
  fawkes::NetworkService *__webview_service;
};


#endif
