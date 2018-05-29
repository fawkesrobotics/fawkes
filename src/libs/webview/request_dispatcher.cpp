
/***************************************************************************
 *  request_dispatcher.cpp - Web request dispatcher
 *
 *  Created: Mon Oct 13 22:48:04 2008
 *  Copyright  2006-2018  Tim Niemueller [www.niemueller.de]
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

#include <webview/request_dispatcher.h>
#include <webview/url_manager.h>
#include <webview/page_reply.h>
#include <webview/error_reply.h>
#include <webview/user_verifier.h>
#include <webview/access_log.h>

#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include <core/exception.h>
#include <utils/misc/string_urlescape.h>
#include <utils/time/time.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <cstdarg>
#include <microhttpd.h>
#include <cstring>
#include <cstdlib>

#include <microhttpd.h>

#define UNAUTHORIZED_REPLY						\
  "<html>\n"								\
  " <head><title>Access denied</title></head>\n"			\
  " <body>\n"								\
  "  <h1>Access denied</h1>\n"					\
  "  <p>Authentication is required to access Fawkes Webview</p>\n"	\
  " </body>\n"								\
  "</html>"

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class WebRequestDispatcher "request_dispatcher.h"
 * Web request dispatcher.
 * Takes web request received via a webserver run by libmicrohttpd and dispatches
 * pages to registered URL handlers or gives a 404 error if no
 * handler was registered for the given url.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param url_manager URL manager to use for URL to processor mapping
 * @param headergen page header generator
 * @param footergen page footer generator
 */
WebRequestDispatcher::WebRequestDispatcher(WebUrlManager *url_manager,
                                           WebPageHeaderGenerator *headergen,
                                           WebPageFooterGenerator *footergen)
{
  realm_                 = NULL;
  access_log_            = NULL;
  url_manager_           = url_manager;
  page_header_generator_ = headergen;
  page_footer_generator_ = footergen;
  active_requests_       = 0;
  active_requests_mutex_ = new Mutex();
  last_request_completion_time_ = new Time();

  cors_allow_all_        = false;
  cors_max_age_          = 0;
}


/** Destructor. */
WebRequestDispatcher::~WebRequestDispatcher()
{
  if (realm_)  free(realm_);
  delete active_requests_mutex_;
  delete last_request_completion_time_;
  delete access_log_;
}


/** Setup basic authentication.
 * @param realm authentication realm to display to the user.
 * If NULL basic authentication will be disabled.
 * @param verifier verifier to use for checking credentials.
 * If NULL basic authentication will be disabled.
 */
void
WebRequestDispatcher::setup_basic_auth(const char *realm,
				       WebUserVerifier *verifier)
{
#if MHD_VERSION >= 0x00090400
  if (realm_)  free(realm_);
  realm_ = NULL;
  user_verifier_ = NULL;
  if (realm && verifier) {
    realm_ = strdup(realm);
    user_verifier_ = verifier;
  }
#else
  throw Exception("libmicrohttpd >= 0.9.4 is required for basic authentication, "
		  "which was not available at compile time.");
#endif
}


/** Setup access log.
 * @param filename access log file name
 */
void
WebRequestDispatcher::setup_access_log(const char *filename)
{
  delete access_log_;
  access_log_ = NULL;
  access_log_ = new WebviewAccessLog(filename);
}


/** Setup cross-origin resource sharing
 * @param allow_all allow access to all hosts
 * @param origins allow access from these specific origins
 * @param max_age maximum cache time to send to the client, zero to disable
 */
void
WebRequestDispatcher::setup_cors(bool allow_all, std::vector<std::string>&& origins,
                                 unsigned int max_age)
{
	cors_allow_all_ = allow_all;
	cors_origins_   = std::move(origins);
	cors_max_age_   = max_age;
}

/** Callback for new requests.
 * @param cls closure, must be WebRequestDispatcher
 * @param uri requested URI
 * @return returns output of WebRequestDispatcher::log_uri()
 */
void *
WebRequestDispatcher::uri_log_cb(void *cls, const char * uri)
{
  WebRequestDispatcher *rd = static_cast<WebRequestDispatcher *>(cls);
  return rd->log_uri(uri);
}

/** Process request callback for libmicrohttpd.
 * @param callback_data instance of WebRequestDispatcher to call
 * @param connection libmicrohttpd connection instance
 * @param url URL, may contain escape sequences
 * @param method HTTP method
 * @param version HTTP version
 * @param upload_data uploaded data
 * @param upload_data_size size of upload_data parameter
 * @param session_data session data pointer
 * @return appropriate return code for libmicrohttpd
 */
int
WebRequestDispatcher::process_request_cb(void *callback_data,
					 struct MHD_Connection * connection,
					 const char *url,
					 const char *method,
					 const char *version,
					 const char *upload_data,
					 size_t *upload_data_size,
					 void **session_data)
{
  WebRequestDispatcher *rd = static_cast<WebRequestDispatcher *>(callback_data);
  return rd->process_request(connection, url, method, version,
			     upload_data, upload_data_size, session_data);
}


/** Process request completion.
 * @param cls closure which is a pointer to the request dispatcher
 * @param connection connection on which the request completed
 * @param con_cls connection specific data, for us the request
 * @param toe termination code
 */
void
WebRequestDispatcher::request_completed_cb(void *cls,
					   struct MHD_Connection *connection, void **con_cls,
					   enum MHD_RequestTerminationCode toe)
{
  WebRequestDispatcher *rd = static_cast<WebRequestDispatcher *>(cls);
  WebRequest *request = static_cast<WebRequest *>(*con_cls);
  rd->request_completed(request, toe);
  delete request;
}


/** Callback based chunk-wise data.
 * Supplies data chunk based.
 * @param reply instance of DynamicWebReply
 * @param pos position in stream
 * @param buf buffer to put data in
 * @param max maximum number of bytes that can be put in buf
 * @return suitable libmicrohttpd return code
 */
static ssize_t
dynamic_reply_data_cb(void *reply, uint64_t pos, char *buf, size_t max)
{
  DynamicWebReply *dreply = static_cast<DynamicWebReply *>(reply);
  ssize_t bytes = dreply->next_chunk(pos, buf, max);
  WebRequest *request = dreply->get_request();
  if (bytes > 0 && request)  request->increment_reply_size(bytes);
  return bytes;
}


/** Callback to free dynamic web reply.
 * @param reply Instance of DynamicWebReply to free.
 */
static void
dynamic_reply_free_cb(void *reply)
{
  DynamicWebReply *dreply = static_cast<DynamicWebReply *>(reply);
  delete dreply;
}


/** Prepare response from static reply.
 * @param sreply static reply
 * @return response struct ready to be enqueued
 */
struct MHD_Response *
WebRequestDispatcher::prepare_static_response(StaticWebReply *sreply)
{
  struct MHD_Response *response;
  WebPageReply *wpreply = dynamic_cast<WebPageReply *>(sreply);
  if (wpreply) {
    wpreply->pack(active_baseurl_,
		  page_header_generator_, page_footer_generator_);
  } else {
    sreply->pack_caching();
    sreply->pack();
  }
  if (sreply->body_length() > 0) {
    response = MHD_create_response_from_buffer(sreply->body_length(),
                                               (void*) sreply->body().c_str(),
                                               MHD_RESPMEM_MUST_COPY);
  } else {
    response = MHD_create_response_from_buffer(0, (void*) "",
                                               MHD_RESPMEM_PERSISTENT);
  }

  WebRequest *request = sreply->get_request();
  if (request) {
    request->set_reply_code(sreply->code());
    request->increment_reply_size(sreply->body_length());
  }

  const WebReply::HeaderMap &headers = sreply->headers();
  WebReply::HeaderMap::const_iterator i;
  for (i = headers.begin(); i != headers.end(); ++i) {
    MHD_add_response_header(response, i->first.c_str(), i->second.c_str());
  }

  return response;
}

/** Prepare response from static reply.
 * @param request request this reply is associated to
 * @param sreply static reply
 * @return response struct ready to be enqueued
 */
int
WebRequestDispatcher::queue_dynamic_reply(struct MHD_Connection * connection,
                                          WebRequest *request,
                                          DynamicWebReply *dreply)
{
  dreply->set_request(request);
  dreply->pack_caching();
  request->set_reply_code(dreply->code());

  struct MHD_Response *response;
  response = MHD_create_response_from_callback(dreply->size(),
					       dreply->chunk_size(),
					       dynamic_reply_data_cb,
					       dreply,
					       dynamic_reply_free_cb);

  const WebReply::HeaderMap &headers = dreply->headers();
  WebReply::HeaderMap::const_iterator i;
  for (i = headers.begin(); i != headers.end(); ++i) {
    MHD_add_response_header(response, i->first.c_str(), i->second.c_str());
  }

  int ret = MHD_queue_response (connection, dreply->code(), response);
  MHD_destroy_response (response);

  return ret;
}

/** Queue a static web reply.
 * @param connection libmicrohttpd connection to queue response to
 * @param request request this reply is associated to
 * @param sreply static web reply to queue
 * @return suitable libmicrohttpd return code
 */
int
WebRequestDispatcher::queue_static_reply(struct MHD_Connection * connection,
					 WebRequest *request,
					 StaticWebReply *sreply)
{
  sreply->set_request(request);

  struct MHD_Response *response = prepare_static_response(sreply);

  int rv = MHD_queue_response(connection, sreply->code(), response);
  MHD_destroy_response(response);
  return rv;
}


/** Queue a static web reply after basic authentication failure.
 * @param connection libmicrohttpd connection to queue response to
 * @return suitable libmicrohttpd return code
 */
int
WebRequestDispatcher::queue_basic_auth_fail(struct MHD_Connection * connection,
					    WebRequest *request)
{
  StaticWebReply sreply(WebReply::HTTP_UNAUTHORIZED, UNAUTHORIZED_REPLY);
#if MHD_VERSION >= 0x00090400
  sreply.set_request(request);
  sreply.pack_caching();
  sreply.pack();
  struct MHD_Response *response = prepare_static_response(&sreply);

  int rv = MHD_queue_basic_auth_fail_response(connection, realm_, response);
  MHD_destroy_response(response);
#else
  sreply.add_header(MHD_HTTP_HEADER_WWW_AUTHENTICATE,
		    (std::string("Basic realm=") + realm_).c_str());
  
  int rv = queue_static_reply(connection, request, &sreply);
#endif
  return rv;
}


/// @cond INTERNALS
/** Iterator over key-value pairs where the value
 * maybe made available in increments and/or may
 * not be zero-terminated.  Used for processing
 * POST data.
 *
 * @param cls user-specified closure
 * @param kind type of the value
 * @param key 0-terminated key for the value
 * @param filename name of the uploaded file, NULL if not known
 * @param content_type mime-type of the data, NULL if not known
 * @param transfer_encoding encoding of the data, NULL if not known
 * @param data pointer to size bytes of data at the
 *              specified offset
 * @param off offset of data in the overall value
 * @param size number of bytes in data available
 * @return MHD_YES to continue iterating,
 *         MHD_NO to abort the iteration
 */
static int
post_iterator(void *cls, enum MHD_ValueKind kind, const char *key,
	     const char *filename, const char *content_type,
	     const char *transfer_encoding, const char *data, uint64_t off,
	     size_t size)
{
  WebRequest *request = static_cast<WebRequest *>(cls);

  // Cannot handle files, yet
  if (filename)  return MHD_NO;

  request->set_post_value(key, data+off, size);

  return MHD_YES;
}
/// @endcond

/** URI logging callback.
 * @param uri requested URI
 */
void *
WebRequestDispatcher::log_uri(const char *uri)
{
  return new WebRequest(uri);
}

/** Process request callback for libmicrohttpd.
 * @param connection libmicrohttpd connection instance
 * @param url URL, may contain escape sequences
 * @param method HTTP method
 * @param version HTTP version
 * @param upload_data uploaded data
 * @param upload_data_size size of upload_data parameter
 * @param session_data session data pointer
 * @return appropriate return code for libmicrohttpd
 */
int
WebRequestDispatcher::process_request(struct MHD_Connection * connection,
				      const char *url,
				      const char *method,
				      const char *version,
				      const char *upload_data,
				      size_t *upload_data_size,
				      void **session_data)
{
  WebRequest *request = static_cast<WebRequest *>(*session_data);

  if ( ! request->is_setup() ) {
    // The first time only the headers are valid,
    // do not respond in the first round...
    request->setup(url, method, version, connection);

    active_requests_mutex_->lock();
    active_requests_ += 1;
    active_requests_mutex_->unlock();

    if (0 == strcmp(method, MHD_HTTP_METHOD_POST)) {
	    request->pp_ =
		    MHD_create_post_processor(connection, 1024, &post_iterator, request);
    }

    return MHD_YES;
  }

#if MHD_VERSION >= 0x00090400
  if (realm_) {
    char *user, *pass = NULL;
    user = MHD_basic_auth_get_username_password(connection, &pass);
    if ( (user == NULL) || (pass == NULL) ||
	 ! user_verifier_->verify_user(user, pass))
    {
      return queue_basic_auth_fail(connection, request);
    }
    request->user_ = user;
  }
#endif

  if (0 == strcmp(method, MHD_HTTP_METHOD_OPTIONS)) {
	  StaticWebReply *reply = new StaticWebReply(WebReply::HTTP_OK);
	  reply->set_caching(true); // handled via Max-Age header anyway
	  const std::map<std::string, std::string> &headers{request->headers()};
	  const auto &request_method = headers.find("Access-Control-Request-Method");
	  const auto &request_headers = headers.find("Access-Control-Request-Headers");
	  if (cors_allow_all_) {
		  reply->add_header("Access-Control-Allow-Origin", "*");
		  if (cors_max_age_ > 0) {
			  reply->add_header("Access-Control-Max-Age", std::to_string(cors_max_age_));
		  }
		  if (request_method != headers.end()) {
			  reply->add_header("Access-Control-Allow-Methods", request_method->second);
		  }
		  if (request_headers != headers.end()) {
			  reply->add_header("Access-Control-Allow-Headers", request_headers->second);
		  }
	  } else if (! cors_origins_.empty()) {
		  const auto &origin = headers.find("Origin");
		  if (origin != headers.end()) {
			  if (std::find(cors_origins_.begin(), cors_origins_.end(), origin->second) != cors_origins_.end()) {
				  reply->add_header("Access-Control-Allow-Origin", origin->second);
				  if (cors_max_age_ > 0) {
					  reply->add_header("Access-Control-Max-Age", std::to_string(cors_max_age_));
				  }
				  if (request_method != headers.end()) {
					  reply->add_header("Access-Control-Allow-Methods", request_method->second);
				  }
				  if (request_headers != headers.end()) {
					  reply->add_header("Access-Control-Allow-Headers", request_headers->second);
				  }
			  } else {
				  reply->set_code(WebReply::HTTP_FORBIDDEN);
			  }
		  } else {
			  reply->set_code(WebReply::HTTP_FORBIDDEN);
		  }
	  }
	  return queue_static_reply(connection, request, reply);
	  delete reply;
  }

  if (0 == strcmp(method, MHD_HTTP_METHOD_POST)) {
	  if (MHD_post_process(request->pp_, upload_data, *upload_data_size) == MHD_NO) {
		  request->addto_body(upload_data, *upload_data_size);
	  }
	  if (0 != *upload_data_size) {
		  *upload_data_size = 0;
		  return MHD_YES;
	  }
	  MHD_destroy_post_processor(request->pp_);
	  request->pp_ = NULL;
  } else if (0 != *upload_data_size) {
	  request->addto_body(upload_data, *upload_data_size);
	  *upload_data_size = 0;
	  return MHD_YES;
  } else {
	  request->finish_body();
  }

  try {
	  WebReply *reply = url_manager_->process_request(request);
	  int ret;

	  if (reply) {
		  if (cors_allow_all_) {
			  reply->add_header("Access-Control-Allow-Origin", "*");
		  }

	    StaticWebReply  *sreply = dynamic_cast<StaticWebReply *>(reply);
	    DynamicWebReply *dreply = dynamic_cast<DynamicWebReply *>(reply);
      if (sreply) {
	      ret = queue_static_reply(connection, request, sreply);
	      delete reply;
      } else if (dreply) {
	      ret = queue_dynamic_reply(connection, request, dreply);
      } else {
	      WebErrorPageReply ereply(WebReply::HTTP_INTERNAL_SERVER_ERROR,
	                               "Unknown reply type");
	      ret = queue_static_reply(connection, request, &ereply);
	      delete reply;
      }
	  } else {
		  WebErrorPageReply ereply(WebReply::HTTP_NOT_FOUND);
	    ret = queue_static_reply(connection, request, &ereply);
	  }
	  return ret;
  } catch (Exception &e) {
	  WebErrorPageReply ereply(WebReply::HTTP_INTERNAL_SERVER_ERROR,
	                           "%s", e.what_no_backtrace());
	  return queue_static_reply(connection, request, &ereply);
  } catch (std::exception &e) {
	  WebErrorPageReply ereply(WebReply::HTTP_INTERNAL_SERVER_ERROR,
	                           "%s", e.what());
	  return queue_static_reply(connection, request, &ereply);
  }
}


void
WebRequestDispatcher::request_completed(WebRequest *request, MHD_RequestTerminationCode term_code)
{
  active_requests_mutex_->lock();
  if (active_requests_ >  0)  active_requests_ -= 1;
  last_request_completion_time_->stamp();
  active_requests_mutex_->unlock();
  if (access_log_)  access_log_->log(request);
}

/** Get number of active requests.
 * @return number of ongoing requests.
 */
unsigned int
WebRequestDispatcher::active_requests() const
{
  MutexLocker lock(active_requests_mutex_);
  return active_requests_;
}

/** Get time when last request was completed.
 * @return Time when last request was completed
 */
Time
WebRequestDispatcher::last_request_completion_time() const
{
  MutexLocker lock(active_requests_mutex_);
  return *last_request_completion_time_;
}

} // end namespace fawkes
