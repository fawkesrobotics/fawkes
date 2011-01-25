
/***************************************************************************
 *  request_dispatcher.cpp - Web request dispatcher
 *
 *  Created: Mon Oct 13 22:48:04 2008
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

#include <webview/request_dispatcher.h>
#include <webview/request_processor.h>
#include <webview/url_manager.h>
#include <webview/page_reply.h>
#include <webview/error_reply.h>
#include <webview/user_verifier.h>

#include <core/threading/mutex_locker.h>
#include <core/exception.h>
#include <utils/misc/string_urlescape.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <cstdarg>
#include <microhttpd.h>
#include <cstring>
#include <cstdlib>

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
 * pages to registered WebRequestProcessor instances or gives a 404 error if no
 * processor was registered for the given base url.
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
  __realm                 = NULL;
  __url_manager           = url_manager;
  __page_header_generator = headergen;
  __page_footer_generator = footergen;
}


/** Destructor. */
WebRequestDispatcher::~WebRequestDispatcher()
{
  if (__realm)  free(__realm);
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
  if (__realm)  free(__realm);
  __realm = NULL;
  __user_verifier = NULL;
  if (realm && verifier) {
    __realm = strdup(realm);
    __user_verifier = verifier;
  }
#else
  throw Exception("libmicrohttpd >= 0.9.4 is required for basic authentication, "
		  "which was not available at compile time.");
#endif
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


/** Callback based chunk-wise data.
 * Supplies data chunk based.
 * @param reply instance of DynamicWebReply
 * @param pos position in stream
 * @param buf buffer to put data in
 * @param max maximum number of bytes that can be put in buf
 * @return suitable libmicrohttpd return code
 */
#if MHD_VERSION >= 0x00090200
static ssize_t
dynamic_reply_data_cb(void *reply, uint64_t pos, char *buf, size_t max)
#else
static int
dynamic_reply_data_cb(void *reply, uint64_t pos, char *buf, int max)
#endif
{
  DynamicWebReply *dreply = static_cast<DynamicWebReply *>(reply);
  return dreply->next_chunk(pos, buf, max);
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
    wpreply->pack(__active_baseurl,
		  __page_header_generator, __page_footer_generator);
  } else {
    sreply->pack();
  }
  if (sreply->body_length() > 0) {
    response = MHD_create_response_from_data(sreply->body_length(),
					     (void*) sreply->body().c_str(),
					     /* free */ MHD_YES,
					     /* copy */ MHD_YES);
  } else {
    response = MHD_create_response_from_data(0, (void*) "",
					     /* free */ MHD_NO,
					     /* copy */ MHD_NO);
  }

  const WebReply::HeaderMap &headers = sreply->headers();
  WebReply::HeaderMap::const_iterator i;
  for (i = headers.begin(); i != headers.end(); ++i) {
    MHD_add_response_header(response, i->first.c_str(), i->second.c_str());
  }

  return response;
}

/** Queue a static web reply.
 * @param connection libmicrohttpd connection to queue response to
 * @param sreply static web reply to queue
 * @return suitable libmicrohttpd return code
 */
int
WebRequestDispatcher::queue_static_reply(struct MHD_Connection * connection,
					 StaticWebReply *sreply)
{
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
WebRequestDispatcher::queue_basic_auth_fail(struct MHD_Connection * connection)
{
  StaticWebReply sreply(WebReply::HTTP_UNAUTHORIZED, UNAUTHORIZED_REPLY);
#if MHD_VERSION >= 0x00090400
  struct MHD_Response *response = prepare_static_response(&sreply);

  int rv = MHD_queue_basic_auth_fail_response(connection, __realm, response);
  MHD_destroy_response(response);
#else
  sreply.add_header(MHD_HTTP_HEADER_WWW_AUTHENTICATE,
		    (std::string("Basic realm=") + __realm).c_str());
  
  int rv = queue_static_reply(connection, &sreply);
#endif
  return rv;
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
  std::string surl = url;
  static int dummy;
  int ret;

  if ((0 != strcmp(method, "GET")) && (0 != strcmp(method, "POST")))
    return MHD_NO; /* unexpected method */

  MutexLocker lock(__url_manager->mutex());
  WebRequestProcessor *proc = __url_manager->find_processor(surl);

  if (proc) {
    char *urlc = strdup(url);
    fawkes::hex_unescape(urlc);
    std::string urls = urlc;
    free(urlc);

    if (! proc->handles_session_data()) {
      if ( *session_data == NULL) {
	// The first time only the headers are valid,
	// do not respond in the first round...
	*session_data = &dummy;
	return MHD_YES;
      }
      *session_data = NULL; /* clear context pointer */
    } else {
      if ( *session_data == NULL) {
	WebReply *reply = proc->process_request(urls.c_str(), method, version,
						upload_data, upload_data_size,
						session_data);
	if ((reply != NULL) || (*session_data == NULL)) {
	  return MHD_NO;
	} else {
	  return MHD_YES;
	}
      }
    }

#if MHD_VERSION >= 0x00090400
    if (__realm) {
      char *user, *pass = NULL;
      user = MHD_basic_auth_get_username_password(connection, &pass);
      if ( (user == NULL) || (pass == NULL) ||
	   ! __user_verifier->verify_user(user, pass))
      {
	return queue_basic_auth_fail(connection);
      }
    }
#endif

    WebReply *reply = proc->process_request(urls.c_str(), method, version,
					    upload_data, upload_data_size,
					    session_data);
    if ( reply ) {
      StaticWebReply  *sreply = dynamic_cast<StaticWebReply *>(reply);
      DynamicWebReply *dreply = dynamic_cast<DynamicWebReply *>(reply);
      if (sreply) {
	ret = queue_static_reply(connection, sreply);
	delete reply;
      } else if (dreply) {
	struct MHD_Response *response;
	response = MHD_create_response_from_callback(dreply->size(),
						     dreply->chunk_size(),
						     dynamic_reply_data_cb,
						     dreply,
						     dynamic_reply_free_cb);
	ret = MHD_queue_response (connection, dreply->code(), response);
	MHD_destroy_response (response);
      } else {
	WebErrorPageReply ereply(WebReply::HTTP_INTERNAL_SERVER_ERROR);
	ret = queue_static_reply(connection, &ereply);
	delete reply;
      }
    } else {
      if (proc->handles_session_data()) {
	return MHD_YES;
      } else {
	WebErrorPageReply ereply(WebReply::HTTP_NOT_FOUND);
	ret = queue_static_reply(connection, &ereply);
      }
    }
  } else {
    if (surl == "/") {
      WebPageReply preply("Fawkes", "<h1>Welcome to Fawkes.</h1><hr />");
      ret = queue_static_reply(connection, &preply);
    } else {
      WebErrorPageReply ereply(WebReply::HTTP_NOT_FOUND);
      ret = queue_static_reply(connection, &ereply);
    }
  }
  return ret;
}

} // end namespace fawkes
