
/***************************************************************************
 *  request_dispatcher.h - Web request dispatcher
 *
 *  Created: Mon Oct 13 22:44:33 2008
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

#ifndef __LIBS_WEBVIEW_REQUEST_DISPATCHER_H_
#define __LIBS_WEBVIEW_REQUEST_DISPATCHER_H_

#include <utils/time/time.h>

#include <string>
#include <map>
#include <memory>
#include <vector>

#include <microhttpd.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class WebRequestProcessor;
class WebUrlManager;
class WebPageHeaderGenerator;
class WebPageFooterGenerator;
class StaticWebReply;
class DynamicWebReply;
class WebUserVerifier;
class WebRequest;
class WebviewAccessLog;
class Mutex;

class WebRequestDispatcher
{
 public:
  WebRequestDispatcher(WebUrlManager *url_manager,
		       WebPageHeaderGenerator *headergen = 0,
		       WebPageFooterGenerator *footergen = 0);
  ~WebRequestDispatcher();

  static int process_request_cb(void *callback_data,
				struct MHD_Connection * connection,
				const char *url,
				const char *method,
				const char *version,
				const char *upload_data,
				size_t *upload_data_size,
				void  **session_data);

  static void request_completed_cb(void *cls,
				   struct MHD_Connection *connection, void **con_cls,
				   enum MHD_RequestTerminationCode toe);

  static void * uri_log_cb(void *cls, const char *uri);

  void setup_basic_auth(const char *realm, WebUserVerifier *verifier);
  void setup_access_log(const char *filename);
  void setup_cors(bool allow_all, std::vector<std::string>&& origins, unsigned int max_age);

  unsigned int active_requests() const;
  Time last_request_completion_time() const;

 private:
  struct MHD_Response *  prepare_static_response(StaticWebReply *sreply);
  int queue_static_reply(struct MHD_Connection * connection, WebRequest *request,
			 StaticWebReply *sreply);
  int queue_dynamic_reply(struct MHD_Connection * connection, WebRequest *request,
			  DynamicWebReply *sreply);
  int queue_basic_auth_fail(struct MHD_Connection * connection, WebRequest *request);
  int process_request(struct MHD_Connection * connection,
		      const char *url, const char *method, const char *version,
		      const char *upload_data, size_t *upload_data_size,
		      void **session_data);
  void * log_uri(const char *uri);

  void request_completed(WebRequest *request,
			 MHD_RequestTerminationCode term_code);

 private:
  WebUrlManager            *url_manager_;
  WebviewAccessLog         *access_log_;

  std::string               active_baseurl_;
  WebPageHeaderGenerator   *page_header_generator_;
  WebPageFooterGenerator   *page_footer_generator_;

  char                     *realm_;
  WebUserVerifier          *user_verifier_;

  unsigned int              active_requests_;
  fawkes::Time             *last_request_completion_time_;
  fawkes::Mutex            *active_requests_mutex_;

  bool                      cors_allow_all_;
  std::vector<std::string>  cors_origins_;
  unsigned int              cors_max_age_;
};

} // end namespace fawkes

#endif
