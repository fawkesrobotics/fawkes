
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

class WebviewThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::NetworkAspect,
  public fawkes::LoggerAspect,
  public fawkes::PluginDirectorAspect,
#ifdef HAVE_TF
  public fawkes::TransformAspect,
#endif
  public fawkes::WebviewAspect
{
 public:
  WebviewThread(bool enable_tp);
  ~WebviewThread();

  virtual void init();
  virtual void finalize();
  virtual void loop();

 private:
  void tls_create(const char *tls_key_file, const char *tls_cert_file);


 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  fawkes::WebServer                  *webserver_;
  fawkes::WebRequestDispatcher       *dispatcher_;

  WebviewStaticRequestProcessor      *static_processor_;
  WebviewStartPageRequestProcessor   *startpage_processor_;
  WebviewBlackBoardRequestProcessor  *blackboard_processor_;
  WebviewPluginsRequestProcessor     *plugins_processor_;
  WebviewRESTRequestProcessor        *rest_processor_;
#ifdef HAVE_TF
  WebviewTfRequestProcessor          *tf_processor_;
#endif
  WebviewServiceBrowseHandler        *service_browse_handler_;
  WebviewHeaderGenerator             *header_gen_;
  WebviewFooterGenerator             *footer_gen_;
  WebviewUserVerifier                *user_verifier_;

  unsigned int cfg_port_;
  bool         cfg_use_ipv4_;
  bool         cfg_use_ipv6_;
  bool         cfg_use_tls_;
  bool         cfg_tls_create_;
  std::string  cfg_tls_key_;
  std::string  cfg_tls_cert_;
  std::string  cfg_tls_cipher_suite_;
  bool         cfg_use_basic_auth_;
  std::string  cfg_basic_auth_realm_;
  std::string  cfg_access_log_;
  bool         cfg_use_thread_pool_;
  unsigned int cfg_num_threads_;

  fawkes::CacheLogger     cache_logger_;
  fawkes::NetworkService *webview_service_;
};


#endif
