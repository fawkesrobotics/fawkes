
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

#include <cstring>

#include <clipsmm.h>
#include <clips/clips.h>

using namespace fawkes;

/** @class ClipsWebRequestProcessor "rrdweb_processor.h"
 * Clips web request processor.
 * Process web requests to the rrd URL space.
 * @author Tim Niemueller
 */

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
}


/** Destructor. */
ClipsWebRequestProcessor::~ClipsWebRequestProcessor()
{
  EnvDeleteRouter(clips_->cobj(), (char *)"webview-reqproc");
}

WebReply *
ClipsWebRequestProcessor::process_request(const fawkes::WebRequest *request)
{
  if ( strncmp(baseurl_, request->url().c_str(), baseurl_len_) == 0 ) {
    // It is in our URL prefix range
    std::string subpath = request->url().substr(baseurl_len_);

    WebPageReply *r = new WebPageReply("Clips Graphs");
    *r += "<h2>CLIPS Facts</h2>\n";

    MutexLocker lock(clips_.objmutex_ptr());

    *r += "<table>";
    *r += "<tr><th>Index</th><th>Fact</th></tr>\n";

    CLIPS::Fact::pointer fact = clips_->get_facts();
    while (fact) {
      CLIPS::Template::pointer tmpl = fact->get_template();

      char tmp[16384];
      OpenStringDestination(clips_->cobj(), (char *)"ProcPPForm", tmp, 16383);
      PrintFact(clips_->cobj(), (char *)"ProcPPForm",
		(struct fact *)fact->cobj(), FALSE, FALSE);
      CloseStringDestination(clips_->cobj(), (char *)"ProcPPForm");

      r->append_body("<tr><td>f-%li</td><td>%s</td></tr>\n",
		     fact->index(), tmp);

      fact = fact->next();
    }
    

    *r += "</table>";

    return r;
  } else {
    return NULL;
  }
}
