
/***************************************************************************
 *  webview_thread.h - Thread that handles web interface requests
 *
 *  Created: Mon Oct 13 17:49:52 2008 (I5 Developer's Day)
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

#ifndef __PLUGINS_WEBVIEW_WEBVIEW_THREAD_H_
#define __PLUGINS_WEBVIEW_WEBVIEW_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>
#include <aspect/network.h>
#include <aspect/logger.h>
#include <aspect/plugin_director.h>
#include <aspect/webview.h>
#ifdef HAVE_JPEG
#  include <aspect/thread_producer.h>
#endif
#ifdef HAVE_TF
#  include <aspect/tf.h>
#endif

#include <logging/cache.h>

namespace fawkes {
  class NetworkService;
  class WebServer;
  class WebRequestDispatcher;
}

class WebviewStaticRequestProcessor;
class WebviewBlackBoardRequestProcessor;
class WebviewStartPageRequestProcessor;
class WebviewPluginsRequestProcessor;
class WebviewRESTRequestProcessor;
class WebviewServiceBrowseHandler;
class WebviewFooterGenerator;
class WebviewHeaderGenerator;
class WebviewUserVerifier;
#ifdef HAVE_TF
class WebviewTfRequestProcessor;
#endif
#ifdef HAVE_JPEG
class WebviewImageRequestProcessor;
#endif

class WebviewThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::NetworkAspect,
  public fawkes::LoggerAspect,
  public fawkes::PluginDirectorAspect,
#ifdef HAVE_JPEG
  public fawkes::ThreadProducerAspect,
#endif
#ifdef HAVE_TF
  public fawkes::TransformAspect,
#endif
  public fawkes::WebviewAspect
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
  static const char *TF_URL_PREFIX;
  static const char *IMAGE_URL_PREFIX;

 private:
  void ssl_create(const char *ssl_key_file, const char *ssl_cert_file);


 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  fawkes::WebServer                  *__webserver;
  fawkes::WebRequestDispatcher       *__dispatcher;

  WebviewStaticRequestProcessor      *__static_processor;
  WebviewStartPageRequestProcessor   *__startpage_processor;
  WebviewBlackBoardRequestProcessor  *__blackboard_processor;
  WebviewPluginsRequestProcessor     *__plugins_processor;
  WebviewRESTRequestProcessor *__rest_processor;
#ifdef HAVE_TF
  WebviewTfRequestProcessor          *__tf_processor;
#endif
#ifdef HAVE_JPEG
  WebviewImageRequestProcessor       *__image_processor;
#endif
  WebviewServiceBrowseHandler        *__service_browse_handler;
  WebviewHeaderGenerator             *__header_gen;
  WebviewFooterGenerator             *__footer_gen;
  WebviewUserVerifier                *__user_verifier;

  unsigned int __cfg_port;
  bool         __cfg_use_ipv4;
  bool         __cfg_use_ipv6;
  bool         __cfg_use_ssl;
  bool         __cfg_ssl_create;
  std::string  __cfg_ssl_key;
  std::string  __cfg_ssl_cert;
  std::string  __cfg_ssl_cipher_suite;
  bool         __cfg_use_basic_auth;
  std::string  __cfg_basic_auth_realm;
  std::string  __cfg_access_log;

  fawkes::CacheLogger     __cache_logger;
  fawkes::NetworkService *__webview_service;
};


#endif
