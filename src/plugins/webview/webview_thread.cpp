
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


/** Constructor. */
WebviewThread::WebviewThread()
  : Thread("WebviewThread", Thread::OPMODE_CONTINUOUS),
    LoggerAspect(&__cache_logger)
{
  set_prepfin_conc_loop(true);
}


WebviewThread::~WebviewThread()
{
}

void
WebviewThread::init()
{
  __cfg_port = config->get_uint("/webview/port");

  WebReply::set_caching(config->get_bool("/webview/client_side_caching"));

  __webview_service = NULL;
  __service_browse_handler = NULL;
  __header_gen = NULL;
  __footer_gen = NULL;
  __dispatcher = NULL;

  __cfg_use_ssl = false;
  try {
    __cfg_use_ssl = config->get_bool("/webview/use_ssl");
  } catch (Exception &e) {}

  __cfg_use_ipv4 = config->get_bool("/network/ipv4/enable");
  __cfg_use_ipv6 = config->get_bool("/network/ipv6/enable");

  if (__cfg_use_ssl) {
    __cfg_ssl_create = false;
    try {
      __cfg_ssl_create = config->get_bool("/webview/ssl_create");
    } catch (Exception &e) {}

    __cfg_ssl_key  = config->get_string("/webview/ssl_key");
    __cfg_ssl_cert = config->get_string("/webview/ssl_cert");

    try {
      __cfg_ssl_cipher_suite = config->get_string("/webview/ssl_cipher_suite");
      logger->log_debug(name(), "Using cipher suite %s", __cfg_ssl_cipher_suite.c_str());
    } catch (Exception &e) {}

    if (__cfg_ssl_key[0] != '/')
      __cfg_ssl_key = std::string(CONFDIR"/") + __cfg_ssl_key;

    if (__cfg_ssl_cert[0] != '/')
      __cfg_ssl_cert = std::string(CONFDIR"/") + __cfg_ssl_cert;

    logger->log_debug(name(), "Key: %s  Cert: %s", __cfg_ssl_key.c_str(),
		      __cfg_ssl_cert.c_str());

    if (! File::exists(__cfg_ssl_key.c_str())) {
      if (File::exists(__cfg_ssl_cert.c_str())) {
	throw Exception("Key file %s does not exist, but certificate file %s "
			"does", __cfg_ssl_key.c_str(), __cfg_ssl_cert.c_str());
      } else if (__cfg_ssl_create) {
	ssl_create(__cfg_ssl_key.c_str(), __cfg_ssl_cert.c_str());
      } else {
 	throw Exception("Key file %s does not exist", __cfg_ssl_key.c_str());
      }
    } else if (! File::exists(__cfg_ssl_cert.c_str())) {
      throw Exception("Certificate file %s does not exist, but key file %s "
		      "does", __cfg_ssl_key.c_str(), __cfg_ssl_cert.c_str());
    }
  }

  __cfg_use_basic_auth = false;
  try {
    __cfg_use_basic_auth = config->get_bool("/webview/use_basic_auth");
  } catch (Exception &e) {}
  __cfg_basic_auth_realm = "Fawkes Webview";
  try {
    __cfg_basic_auth_realm = config->get_bool("/webview/basic_auth_realm");
  } catch (Exception &e) {}

  __cfg_access_log = "";
  try {
    __cfg_access_log = config->get_string("/webview/access_log");
  } catch (Exception &e) {}


  __cache_logger.clear();

  __webview_service = new NetworkService(nnresolver, "Fawkes Webview on %h",
					 "_http._tcp", __cfg_port);
  __webview_service->add_txt("fawkesver=%u.%u.%u",
			     FAWKES_VERSION_MAJOR, FAWKES_VERSION_MINOR,
			     FAWKES_VERSION_MICRO);
  __service_browse_handler = new WebviewServiceBrowseHandler(logger, __webview_service);

  __header_gen = new WebviewHeaderGenerator(webview_nav_manager);
  __footer_gen = new WebviewFooterGenerator(__service_browse_handler);

  __dispatcher = new WebRequestDispatcher(webview_url_manager,
					  __header_gen, __footer_gen);


  try {
    if (__cfg_use_ssl) {
      __webserver  = new WebServer(__cfg_port, __dispatcher,
                                   __cfg_ssl_key.c_str(), __cfg_ssl_cert.c_str(),
                                   __cfg_ssl_cipher_suite.empty() ? NULL : __cfg_ssl_cipher_suite.c_str(),
                                   logger, __cfg_use_ipv4, __cfg_use_ipv6);
    } else {
      __webserver  = new WebServer(__cfg_port, __dispatcher, logger, __cfg_use_ipv4, __cfg_use_ipv6);
    }

    if (__cfg_use_basic_auth) {
      __user_verifier = new WebviewUserVerifier(config, logger);
      __webserver->setup_basic_auth(__cfg_basic_auth_realm.c_str(),
				    __user_verifier);
    }
    __webserver->setup_request_manager(webview_request_manager);

    if (__cfg_access_log != "") {
      logger->log_debug(name(), "Setting up access log %s", __cfg_access_log.c_str());
      __webserver->setup_access_log(__cfg_access_log.c_str());
    }
  } catch (Exception &e) {
    delete __webview_service;
    delete __service_browse_handler;
    delete __header_gen;
    delete __footer_gen;
    delete __dispatcher;
    throw;
  }


  __startpage_processor  = new WebviewStartPageRequestProcessor(&__cache_logger);
  // get all directories for the static processor
  std::vector<std::string> static_dirs = config->get_strings("/webview/static-dirs");
  static_dirs = StringConversions::resolve_paths(static_dirs);
  std::vector<const char *> static_dirs_cstr = std::vector<const char *>(static_dirs.size());
  for(unsigned int i = 0; i < static_dirs.size(); i++)
  {
    static_dirs_cstr[i] = static_dirs[i].c_str();
  }
  __static_processor     = new WebviewStaticRequestProcessor(STATIC_URL_PREFIX, static_dirs_cstr, logger);
  __blackboard_processor = new WebviewBlackBoardRequestProcessor(BLACKBOARD_URL_PREFIX, blackboard);
  __plugins_processor    = new WebviewPluginsRequestProcessor(PLUGINS_URL_PREFIX, plugin_manager);
  __rest_processor       = new WebviewRESTRequestProcessor("/api", webview_rest_api_manager, logger);
#ifdef HAVE_TF
  __tf_processor         = new WebviewTfRequestProcessor(TF_URL_PREFIX, tf_listener);
#endif
#ifdef HAVE_JPEG
  __image_processor     = new WebviewImageRequestProcessor(IMAGE_URL_PREFIX, config,
							   logger, thread_collector);
#endif
  
  webview_url_manager->register_baseurl("/", __startpage_processor);
  webview_url_manager->register_baseurl(STATIC_URL_PREFIX, __static_processor);
  webview_url_manager->register_baseurl(BLACKBOARD_URL_PREFIX, __blackboard_processor);
  webview_url_manager->register_baseurl(PLUGINS_URL_PREFIX, __plugins_processor);
  webview_url_manager->register_baseurl("/api", __rest_processor);
#ifdef HAVE_TF
  webview_url_manager->register_baseurl(TF_URL_PREFIX, __tf_processor);
#endif
#ifdef HAVE_JPEG
  webview_url_manager->register_baseurl(IMAGE_URL_PREFIX, __image_processor);
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
  if (__cfg_use_ipv4 && __cfg_use_ipv6) {
	  afs = "IPv4,IPv6";
  } else if (__cfg_use_ipv4) {
	  afs = "IPv4";
  } else if (__cfg_use_ipv6) {
	  afs = "IPv6";
  }
  logger->log_info("WebviewThread", "Listening for HTTP%s connections on port %u (%s)",
                   __cfg_use_ssl ? "S" : "", __cfg_port, afs.c_str());

  service_publisher->publish_service(__webview_service);
  service_browser->watch_service("_http._tcp", __service_browse_handler);
}


void
WebviewThread::finalize()
{
  try {
    service_publisher->unpublish_service(__webview_service);
  } catch (Exception &e) {} // ignored, can happen if avahi-daemon not running
  try {
    service_browser->unwatch_service("_http._tcp", __service_browse_handler);
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

  delete __webserver;

  delete __webview_service;
  delete __service_browse_handler;

  delete __dispatcher;
  delete __static_processor;
  delete __blackboard_processor;
  delete __startpage_processor;
  delete __plugins_processor;
#ifdef HAVE_TF
  delete __tf_processor;
#endif
#ifdef HAVE_JPEG
  delete __image_processor;
#endif
  delete __footer_gen;
  delete __header_gen;
  __dispatcher = NULL;
}


void
WebviewThread::loop()
{
  __webserver->process();
}


void
WebviewThread::ssl_create(const char *ssl_key_file, const char *ssl_cert_file)
{
  logger->log_info(name(), "Creating SSL key and certificate. "
		   "This may take a while...");
  HostInfo h;

  char *cmd;
  if (asprintf(&cmd, "openssl req -new -x509 -batch -nodes -days 365 "
	       "-subj \"/C=XX/L=World/O=Fawkes/CN=%s.local\" "
	       "-out \"%s\" -keyout \"%s\" >/dev/null 2>&1",
	       h.short_name(), ssl_cert_file, ssl_key_file) == -1)
  {
    throw OutOfMemoryException("Webview/SSL: Could not generate OpenSSL string");
  }

  int status = system(cmd);
  free(cmd);

  if (WEXITSTATUS(status) != 0) {
    throw Exception("Failed to auto-generate key/certificate pair");
  }
}
