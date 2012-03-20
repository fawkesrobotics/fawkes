
/***************************************************************************
 *  xmlrpc_thread.h - Thread that handles xml-rpc requests
 *
 *  Created: Sun Aug 30 12:44:30 2009
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_XMLRPC_XMLRPC_THREAD_H_
#define __PLUGINS_XMLRPC_XMLRPC_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>
#include <aspect/network.h>
#include <aspect/logger.h>
#include <aspect/plugin_director.h>
#include <aspect/webview.h>

#include <logging/cache.h>

namespace fawkes {
  class NetworkService;
  class WebRequestDispatcher;
  class WebUrlManager;
  class WebServer;
}

class XmlRpcRequestProcessor;
class XmlRpcPluginMethods;
class XmlRpcLogMethods;

class XmlRpcThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::NetworkAspect,
  public fawkes::LoggerAspect,
  public fawkes::PluginDirectorAspect,
  public fawkes::WebviewAspect
{
 public:
  XmlRpcThread();
  ~XmlRpcThread();

  virtual void init();
  virtual void finalize();
  virtual void loop();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  fawkes::WebServer            *__webserver;
  fawkes::WebRequestDispatcher *__dispatcher;
  fawkes::WebUrlManager        *__url_manager;

  XmlRpcRequestProcessor       *__processor;
  XmlRpcPluginMethods          *__plugin_methods;
  XmlRpcLogMethods             *__log_methods;

  bool                          __custom_server;
  unsigned int                  __cfg_port;

  fawkes::CacheLogger           __cache_logger;
  fawkes::NetworkService       *__xmlrpc_service;
};

#endif
