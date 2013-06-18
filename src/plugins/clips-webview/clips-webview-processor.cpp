
/***************************************************************************
 *  clips-webview-processor.cpp - CLIPS introspection via webview
 *
 *  Created: Sat Jun 15 20:17:53 2013
 *  Copyright  2006-2013  Tim Niemueller [www.niemueller.de]
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

#include "clips-webview-processor.h"

#include <core/exception.h>
#include <core/threading/mutex_locker.h>
#include <webview/page_reply.h>
#include <webview/file_reply.h>
#include <webview/error_reply.h>
#include <webview/redirect_reply.h>
#include <utils/misc/string_conversions.h>

#include <cstring>

#include <clipsmm.h>
#include <clips/clips.h>

using namespace fawkes;

/** @class ClipsWebRequestProcessor "rrdweb_processor.h"
 * Clips web request processor.
 * Process web requests to the rrd URL space.
 * @author Tim Niemueller
 */

static int
clips_router_query(void *env, char *logical_name)
{
  if (strcmp(logical_name, WERROR) == 0) return TRUE;
  return FALSE;
}

static int
clips_router_print(void *env, char *logical_name, char *str)
{
  void *rc = GetEnvironmentRouterContext(env);
  ClipsWebRequestProcessor *reqproc = static_cast<ClipsWebRequestProcessor *>(rc);
  reqproc->add_error(str);
  EnvDeactivateRouter(env, (char *)"webview-reqproc");
  EnvPrintRouter(env, logical_name, str);
  EnvActivateRouter(env, (char *)"webview-reqproc");
  return TRUE;
}

static int
clips_router_exit(void *env, int exit_code)
{
  return TRUE;
}


/** Constructor.
 * @param clips CLIPS Environment to query for facts
 * @param logger logger to report problems
 * @param baseurl base URL of the Clips webrequest processor
 */
ClipsWebRequestProcessor::ClipsWebRequestProcessor(fawkes::LockPtr<CLIPS::Environment> &clips,
						   fawkes::Logger *logger, const char *baseurl)
{
  clips_  = clips;
  logger_ = logger;

  baseurl_     = baseurl;
  baseurl_len_ = strlen(baseurl);


  EnvAddRouterWithContext(clips->cobj(), (char *)"webview-reqproc",
			  /* exclusive */ 40,
			  clips_router_query,
			  clips_router_print,
			  /* getc */   NULL,
			  /* ungetc */ NULL,
			  clips_router_exit,
			  this);
}


/** Destructor. */
ClipsWebRequestProcessor::~ClipsWebRequestProcessor()
{
  EnvDeleteRouter(clips_->cobj(), (char *)"webview-reqproc");
}


/** Add an error string.
 * Used by CLIPS I/O router to provide error message.
 * @param str string to add to errors
 */
void
ClipsWebRequestProcessor::add_error(const char *str)
{
  errors_.push_back(str);
}

void
ClipsWebRequestProcessor::retract_fact(long int index)
{
  CLIPS::Fact::pointer fact = clips_->get_facts();
  while (fact) {
    if (fact->index() == index) {
      fact->retract();
      break;
    }
    fact = fact->next();
  }
}

WebReply *
ClipsWebRequestProcessor::process_request(const fawkes::WebRequest *request)
{
  if ( strncmp(baseurl_, request->url().c_str(), baseurl_len_) == 0 ) {
    // It is in our URL prefix range
    std::string subpath = request->url().substr(baseurl_len_);

    if (subpath == "/assert") {
      MutexLocker lock(clips_.objmutex_ptr());
      errors_.clear();
      if (! request->post_value("index").empty()) {
	long int index = StringConversions::to_long(request->post_value("index"));
	retract_fact(index);
      }

      clips_->assert_fact(request->post_value("fact"));
      if (! errors_.empty()) {
	WebPageReply *r = new WebPageReply("CLIPS");
	*r += "<h2>CLIPS Fact Assertion</h2>\n";
	r->append_body("<p><span style=\"color:red\">Asserting '%s' failed:</span>\n<pre>",
		       request->post_value("fact").c_str());
	for (auto e : errors_) {
	  *r += e;
	}
	*r += "</pre></p>";
	r->append_body("<p><a href=\"%s\">Back</a></p>", baseurl_);

	r->append_body("<form action=\"%s/assert\" method=\"post\">"
		       "<input type=\"hidden\" name=\"index\" value=\"%s\">"
		       "New fact: <input type=\"text\" name=\"fact\" value=\"%s\"/>"
		       "<input type=\"submit\" value=\"Assert\" />", baseurl_,
		       request->post_value("index").c_str(),
		       request->post_value("fact").c_str());

	return r;
      } else {
	return new WebRedirectReply(baseurl_);
      }
    } else if (subpath.find("/retract") == 0) {
      std::string index_str = subpath.substr(9); // length of "/retract/"
      long int index = StringConversions::to_long(index_str);
      retract_fact(index);
      return new WebRedirectReply(baseurl_);
    }

    MutexLocker lock(clips_.objmutex_ptr());

    WebPageReply *r = new WebPageReply("CLIPS");
    r->set_html_header("  <link type=\"text/css\" href=\"/static/css/jqtheme/"
		       "jquery-ui.custom.css\" rel=\"stylesheet\" />\n"
		       "  <script type=\"text/javascript\" src=\"/static/js/"
		       "jquery.min.js\"></script>\n"
		       "  <script type=\"text/javascript\" src=\"/static/js/"
		       "jquery-ui.custom.min.js\"></script>\n");

    *r +=
      "<style type=\"text/css\">\n"
      "  tr:hover { background-color: #eeeeee; }\n"
      "</style>";

    *r += "<h2>CLIPS Facts</h2>\n";

    *r += "<table>";
    *r += "<tr><th>Index</th><th>Fact</th><th>Action</th></tr>\n";

    CLIPS::Fact::pointer fact = clips_->get_facts();
    while (fact) {
      CLIPS::Template::pointer tmpl = fact->get_template();

      char tmp[16384];
      OpenStringDestination(clips_->cobj(), (char *)"ProcPPForm", tmp, 16383);
      PrintFact(clips_->cobj(), (char *)"ProcPPForm",
		(struct fact *)fact->cobj(), FALSE, FALSE);
      CloseStringDestination(clips_->cobj(), (char *)"ProcPPForm");

      r->append_body("<tr><td>f-%li</td><td>%s</td>"
		     "<td><a href=\"%s/retract/%li\">Retract</a></td>"
		     "</tr>\n",
		     fact->index(), tmp, baseurl_, fact->index());

      std::string escaped = tmp;
      size_t pos = 0;
      while ((pos = escaped.find("\"", pos)) != std::string::npos) {
	escaped.replace(pos, 1, "&quot;");
      }

      fact = fact->next();
    }
    

    *r += "</table>";

    r->append_body("<p><form action=\"%s/assert\" method=\"post\">"
		   "<input type=\"hidden\" name=\"index\" value=\"\">"
		   "New fact: <input type=\"text\" name=\"fact\" />"
		   "<input type=\"submit\" value=\"Assert\" /></form></p>", baseurl_);

    return r;
  } else {
    return NULL;
  }
}
