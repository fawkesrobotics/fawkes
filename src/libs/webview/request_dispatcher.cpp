
/***************************************************************************
 *  request_dispatcher.cpp - Web request dispatcher
 *
 *  Created: Mon Oct 13 22:48:04 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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
#include <webview/page_reply.h>
#include <webview/error_reply.h>

#include <utils/logging/cache.h>
#include <utils/misc/string_urlescape.h>

#include <microhttpd.h>
#include <cstring>
#include <cstdlib>

/** @class WebRequestDispatcher "request_dispatcher.h"
 * Web request dispatcher.
 * Takes web request received via a webserver run by libmicrohttpd and dispatches
 * pages to registered WebRequestProcessor instances or gives a 404 error if no
 * processor was registered for the given base url.
 * @author Tim Niemueller
 */

WebRequestDispatcher::WebRequestDispatcher(WebPageHeaderGenerator *headergen,
					   WebPageFooterGenerator *footergen)
{
  __page_header_generator = headergen;
  __page_footer_generator = footergen;
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
int
WebRequestDispatcher::dynamic_reply_data_cb(void *reply,
					    uint64_t pos, char *buf, int max)
{
  DynamicWebReply *dreply = static_cast<DynamicWebReply *>(reply);
  return dreply->next_chunk(pos, buf, max);
}


/** Callback to free dynamic web reply.
 * @param reply Instance of DynamicWebReply to free.
 */
void
WebRequestDispatcher::dynamic_reply_free_cb(void *reply)
{
  DynamicWebReply *dreply = static_cast<DynamicWebReply *>(reply);
  delete dreply;
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
  for (WebReply::HeaderMap::const_iterator i = headers.begin(); i != headers.end(); ++i) {
    MHD_add_response_header(response, i->first.c_str(), i->second.c_str());
  }

  int rv = MHD_queue_response(connection, sreply->code(), response);
  MHD_destroy_response(response);
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

  if (0 != strcmp(method, "GET"))
    return MHD_NO; /* unexpected method */

  if (&dummy != *session_data) {
    // The first time only the headers are valid,
    // do not respond in the first round...
    *session_data = &dummy;
    return MHD_YES;
  }
  *session_data = NULL; /* clear context pointer */

  WebRequestProcessor *proc = NULL;
  std::map<std::string, WebRequestProcessor *>::iterator __pit;
  for (__pit = __processors.begin(); (proc == NULL) && (__pit != __processors.end()); ++__pit) {
    if (surl.find(__pit->first) == 0) {
      __active_baseurl = __pit->first;
      proc = __pit->second;
    }
  }

  if ( surl == "/" ) {
    if ( __startpage_processor ) {
      proc = __startpage_processor;
    } else {
      WebPageReply preply("Fawkes", "<h1>Welcome to Fawkes.</h1><hr />");
      ret = queue_static_reply(connection, &preply);
    }
  }

  if (proc) {
    struct MHD_Response *response;

    char *urlc = strdup(url);
    fawkes::hex_unescape(urlc);

    WebReply *reply = proc->process_request(urlc, method, version, upload_data, upload_data_size, session_data);

    free(urlc);

    if ( reply ) {
      StaticWebReply  *sreply = dynamic_cast<StaticWebReply *>(reply);
      DynamicWebReply *dreply = dynamic_cast<DynamicWebReply *>(reply);
      if (sreply) {
	ret = queue_static_reply(connection, sreply);
	delete reply;
      } else if (dreply) {
	response = MHD_create_response_from_callback(dreply->size(),
						     dreply->chunk_size(),
						     WebRequestDispatcher::dynamic_reply_data_cb,
						     dreply,
						     WebRequestDispatcher::dynamic_reply_free_cb);
	ret = MHD_queue_response (connection, dreply->code(), response);
	MHD_destroy_response (response);
      } else {
	WebErrorPageReply ereply(WebReply::HTTP_INTERNAL_SERVER_ERROR);
	ret = queue_static_reply(connection, &ereply);
	delete reply;
      }
    } else {
      WebErrorPageReply ereply(WebReply::HTTP_NOT_FOUND);
      ret = queue_static_reply(connection, &ereply);
    }
  } else {
    WebErrorPageReply ereply(WebReply::HTTP_NOT_FOUND);
    ret = queue_static_reply(connection, &ereply);
  }
  return ret;
}

/** Add a request processor.
 * @param url_prefix baseurl this processor should handle
 * @param processor processor for baseurl
 */
void
WebRequestDispatcher::add_processor(const char *url_prefix,
				    WebRequestProcessor *processor)
{
  if (std::string(url_prefix) == "/") {
    __startpage_processor = processor;
  } else {
    __processors[url_prefix] = processor;
  }
}


/** Remove a request processor.
 * @param url_prefix baseurl the processor handled
 */
void
WebRequestDispatcher::remove_processor(const char *url_prefix)
{
  if (std::string(url_prefix) == "/") {
    __startpage_processor = NULL;
  } else {
    __processors.erase(url_prefix);
  }
}
