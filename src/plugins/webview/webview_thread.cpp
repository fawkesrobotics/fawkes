
/***************************************************************************
 *  webview_thread.cpp - Thread that handles web interface requests
 *
 *  Created: Mon Oct 13 17:51:31 2008 (I5 Developer's Day)
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

#include "webview_thread.h"
#include "static_processor.h"
#include "blackboard_processor.h"
#include "startpage_processor.h"
#include "plugins_processor.h"
#include "rest_processor.h"
#ifdef HAVE_TF
#  include "tf_processor.h"
#endif
#ifdef HAVE_JPEG
#  include "image_processor.h"
#endif
#include "service_browse_handler.h"
#include "header_generator.h"
#include "footer_generator.h"
#include "user_verifier.h"

#include <core/version.h>
#include <core/exceptions/system.h>
#include <utils/system/file.h>
#include <utils/system/hostinfo.h>
#include <webview/request_dispatcher.h>
#include <webview/page_reply.h>
#include <webview/server.h>
#include <webview/url_manager.h>
#include <webview/nav_manager.h>
#include <utils/misc/string_conversions.h>

#include <sys/wait.h>

using namespace fawkes;


/** Prefix for the WebStaticRequestProcessor. */
const char *WebviewThread::STATIC_URL_PREFIX = "/static";
/** Prefix for the WebBlackBoardRequestProcessor. */
const char *WebviewThread::BLACKBOARD_URL_PREFIX = "/blackboard";
/** Prefix for the WebPluginsRequestProcessor. */
const char *WebviewThread::PLUGINS_URL_PREFIX = "/plugins";
/** Prefix for the WebTfRequestProcessor. */
const char *WebviewThread::TF_URL_PREFIX = "/tf";
/** Prefix for the WebMJPEGRequestProcessor. */
const char *WebviewThread::IMAGE_URL_PREFIX = "/images";

/** @class WebviewThread "webview_thread.h"
 * Webview Thread.
 * This thread runs the HTTP server and handles requests via the
 * WebRequestDispatcher.
 * @author Tim Niemueller
 */


/** Constructor.
 * @param enable_tp true to enable thread pool setting the thread to
 * wait-for-wakeup mode, falso to run request processing in this
 * thread.
 */
WebviewThread::WebviewThread(bool enable_tp)
	: Thread("WebviewThread", enable_tp ? Thread::OPMODE_WAITFORWAKEUP : Thread::OPMODE_CONTINUOUS),
    LoggerAspect(&cache_logger_)
{
	cfg_use_thread_pool_ = enable_tp;

	if (!enable_tp) set_prepfin_conc_loop(true);
}


WebviewThread::~WebviewThread()
{
}

void
WebviewThread::init()
{
  cfg_port_ = config->get_uint("/webview/port");

  WebReply::set_caching(config->get_bool("/webview/client_side_caching"));

  webview_service_ = NULL;
  service_browse_handler_ = NULL;
  header_gen_ = NULL;
  footer_gen_ = NULL;
  dispatcher_ = NULL;

  cfg_use_tls_ = false;
  try {
    cfg_use_tls_ = config->get_bool("/webview/tls/enable");
  } catch (Exception &e) {}

  cfg_use_ipv4_ = config->get_bool("/network/ipv4/enable");
  cfg_use_ipv6_ = config->get_bool("/network/ipv6/enable");

  if (cfg_use_tls_) {
    cfg_tls_create_ = false;
    try {
      cfg_tls_create_ = config->get_bool("/webview/tls/create");
    } catch (Exception &e) {}

    cfg_tls_key_  = config->get_string("/webview/tls/key-file");
    cfg_tls_cert_ = config->get_string("/webview/tls/cert-file");

    try {
      cfg_tls_cipher_suite_ = config->get_string("/webview/tls/cipher-suite");
      logger->log_debug(name(), "Using cipher suite %s", cfg_tls_cipher_suite_.c_str());
    } catch (Exception &e) {}

    if (cfg_tls_key_[0] != '/')
      cfg_tls_key_ = std::string(CONFDIR"/") + cfg_tls_key_;

    if (cfg_tls_cert_[0] != '/')
      cfg_tls_cert_ = std::string(CONFDIR"/") + cfg_tls_cert_;

    logger->log_debug(name(), "Key file: %s  Cert file: %s",
                      cfg_tls_key_.c_str(), cfg_tls_cert_.c_str());

    if (! File::exists(cfg_tls_key_.c_str())) {
	    if (File::exists(cfg_tls_cert_.c_str())) {
		    throw Exception("Key file %s does not exist, but certificate file %s "
		                    "does", cfg_tls_key_.c_str(), cfg_tls_cert_.c_str());
	    } else if (cfg_tls_create_) {
		    tls_create(cfg_tls_key_.c_str(), cfg_tls_cert_.c_str());
	    } else {
		    throw Exception("Key file %s does not exist", cfg_tls_key_.c_str());
	    }
    } else if (! File::exists(cfg_tls_cert_.c_str())) {
      throw Exception("Certificate file %s does not exist, but key file %s "
		      "does", cfg_tls_key_.c_str(), cfg_tls_cert_.c_str());
    }
  }

  if (cfg_use_thread_pool_) {
	  cfg_num_threads_ = config->get_uint("/webview/thread-pool/num-threads");
  }

  cfg_use_basic_auth_ = false;
  try {
    cfg_use_basic_auth_ = config->get_bool("/webview/use_basic_auth");
  } catch (Exception &e) {}
  cfg_basic_auth_realm_ = "Fawkes Webview";
  try {
    cfg_basic_auth_realm_ = config->get_bool("/webview/basic_auth_realm");
  } catch (Exception &e) {}

  cfg_access_log_ = "";
  try {
    cfg_access_log_ = config->get_string("/webview/access_log");
  } catch (Exception &e) {}


  cache_logger_.clear();

  webview_service_ = new NetworkService(nnresolver, "Fawkes Webview on %h",
					 "_http._tcp", cfg_port_);
  webview_service_->add_txt("fawkesver=%u.%u.%u",
			     FAWKES_VERSION_MAJOR, FAWKES_VERSION_MINOR,
			     FAWKES_VERSION_MICRO);
  service_browse_handler_ = new WebviewServiceBrowseHandler(logger, webview_service_);

  header_gen_ = new WebviewHeaderGenerator(webview_nav_manager);
  footer_gen_ = new WebviewFooterGenerator(service_browse_handler_);

  dispatcher_ = new WebRequestDispatcher(webview_url_manager,
					  header_gen_, footer_gen_);


  try {
	  webserver_  = new WebServer(cfg_port_, dispatcher_, logger);

	  webserver_->setup_ipv(cfg_use_ipv4_, cfg_use_ipv6_);

    if (cfg_use_tls_) {
	    webserver_->setup_tls(cfg_tls_key_.c_str(), cfg_tls_cert_.c_str(),
	                           cfg_tls_cipher_suite_.empty() ? NULL : cfg_tls_cipher_suite_.c_str());
    }

    if (cfg_use_thread_pool_) {
	    webserver_->setup_thread_pool(cfg_num_threads_);
    }

    if (cfg_use_basic_auth_) {
      user_verifier_ = new WebviewUserVerifier(config, logger);
      webserver_->setup_basic_auth(cfg_basic_auth_realm_.c_str(),
                                    user_verifier_);
    }
    webserver_->setup_request_manager(webview_request_manager);

    if (cfg_access_log_ != "") {
      logger->log_debug(name(), "Setting up access log %s", cfg_access_log_.c_str());
      webserver_->setup_access_log(cfg_access_log_.c_str());
    }
  } catch (Exception &e) {
    delete webview_service_;
    delete service_browse_handler_;
    delete header_gen_;
    delete footer_gen_;
    delete dispatcher_;
    throw;
  }
  startpage_processor_  = new WebviewStartPageRequestProcessor(&cache_logger_);
  // get all directories for the static processor
  std::vector<std::string> static_dirs = config->get_strings("/webview/static-dirs");
  static_dirs = StringConversions::resolve_paths(static_dirs);
  std::vector<const char *> static_dirs_cstr = std::vector<const char *>(static_dirs.size());
  for(unsigned int i = 0; i < static_dirs.size(); i++)
  {
    static_dirs_cstr[i] = static_dirs[i].c_str();
  }
  static_processor_     = new WebviewStaticRequestProcessor(STATIC_URL_PREFIX, static_dirs_cstr, logger);
  blackboard_processor_ = new WebviewBlackBoardRequestProcessor(BLACKBOARD_URL_PREFIX, blackboard);
  plugins_processor_    = new WebviewPluginsRequestProcessor(PLUGINS_URL_PREFIX, plugin_manager);
  rest_processor_       = new WebviewRESTRequestProcessor("/api", webview_rest_api_manager, logger);
#ifdef HAVE_TF
  tf_processor_         = new WebviewTfRequestProcessor(TF_URL_PREFIX, tf_listener);
#endif
#ifdef HAVE_JPEG
  image_processor_     = new WebviewImageRequestProcessor(IMAGE_URL_PREFIX, config,
							   logger, thread_collector);
#endif

  webview_url_manager->register_baseurl("/", startpage_processor_);
  webview_url_manager->register_baseurl(STATIC_URL_PREFIX, static_processor_);
  webview_url_manager->register_baseurl(BLACKBOARD_URL_PREFIX, blackboard_processor_);
  webview_url_manager->register_baseurl(PLUGINS_URL_PREFIX, plugins_processor_);
  webview_url_manager->register_baseurl("/api", rest_processor_);
#ifdef HAVE_TF
  webview_url_manager->register_baseurl(TF_URL_PREFIX, tf_processor_);
#endif
#ifdef HAVE_JPEG
  webview_url_manager->register_baseurl(IMAGE_URL_PREFIX, image_processor_);
#endif

  webview_nav_manager->add_nav_entry(BLACKBOARD_URL_PREFIX, "BlackBoard");
#ifdef HAVE_TF
  webview_nav_manager->add_nav_entry(TF_URL_PREFIX, "TF");
#endif
  webview_nav_manager->add_nav_entry(PLUGINS_URL_PREFIX, "Plugins");
#ifdef HAVE_JPEG
  webview_nav_manager->add_nav_entry(IMAGE_URL_PREFIX, "Images");
#endif

  std::string afs;
  if (cfg_use_ipv4_ && cfg_use_ipv6_) {
	  afs = "IPv4,IPv6";
  } else if (cfg_use_ipv4_) {
	  afs = "IPv4";
  } else if (cfg_use_ipv6_) {
	  afs = "IPv6";
  }
  webserver_->start();
  logger->log_info("WebviewThread", "Listening for HTTP%s connections on port %u (%s)",
                   cfg_use_tls_ ? "S" : "", cfg_port_, afs.c_str());

  service_publisher->publish_service(webview_service_);
  service_browser->watch_service("_http._tcp", service_browse_handler_);
}


void
WebviewThread::finalize()
{
  try {
    service_publisher->unpublish_service(webview_service_);
  } catch (Exception &e) {} // ignored, can happen if avahi-daemon not running
  try {
    service_browser->unwatch_service("_http._tcp", service_browse_handler_);
  } catch (Exception &e) {} // ignored, can happen if avahi-daemon not running

  webview_url_manager->unregister_baseurl("/");
  webview_url_manager->unregister_baseurl(STATIC_URL_PREFIX);
  webview_url_manager->unregister_baseurl(BLACKBOARD_URL_PREFIX);
  webview_url_manager->unregister_baseurl(PLUGINS_URL_PREFIX);
  webview_url_manager->unregister_baseurl(IMAGE_URL_PREFIX);

#ifdef HAVE_TF
  webview_url_manager->unregister_baseurl(TF_URL_PREFIX);
#endif

  webview_nav_manager->remove_nav_entry(BLACKBOARD_URL_PREFIX);
  webview_nav_manager->remove_nav_entry(PLUGINS_URL_PREFIX);
#ifdef HAVE_TF
  webview_nav_manager->remove_nav_entry(TF_URL_PREFIX);
#endif
#ifdef HAVE_JPEG
  webview_nav_manager->remove_nav_entry(IMAGE_URL_PREFIX);
#endif

  delete webserver_;

  delete webview_service_;
  delete service_browse_handler_;

  delete dispatcher_;
  delete static_processor_;
  delete blackboard_processor_;
  delete startpage_processor_;
  delete plugins_processor_;
  delete rest_processor_;
#ifdef HAVE_TF
  delete tf_processor_;
#endif
#ifdef HAVE_JPEG
  delete image_processor_;
#endif
  delete footer_gen_;
  delete header_gen_;
  dispatcher_ = NULL;
}


void
WebviewThread::loop()
{
	if (! cfg_use_thread_pool_) webserver_->process();
}


void
WebviewThread::tls_create(const char *tls_key_file, const char *tls_cert_file)
{
  logger->log_info(name(), "Creating TLS key and certificate. "
		   "This may take a while...");
  HostInfo h;

  char *cmd;
  if (asprintf(&cmd, "openssl req -new -x509 -batch -nodes -days 365 "
	       "-subj \"/C=XX/L=World/O=Fawkes/CN=%s.local\" "
	       "-out \"%s\" -keyout \"%s\" >/dev/null 2>&1",
	       h.short_name(), tls_cert_file, tls_key_file) == -1)
  {
    throw OutOfMemoryException("Webview/TLS: Could not generate OpenSSL string");
  }

  int status = system(cmd);
  free(cmd);

  if (WEXITSTATUS(status) != 0) {
    throw Exception("Failed to auto-generate key/certificate pair");
  }
}
