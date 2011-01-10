
/***************************************************************************
 *  rrdweb_processor.cpp - RRD web request processor
 *
 *  Created: Tue Dec 21 01:12:58 2010
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

#include "rrdweb_processor.h"
#include <plugins/rrd/aspect/rrd_manager.h>
#include <core/threading/scoped_rwlock.h>
#include <webview/page_reply.h>
#include <webview/file_reply.h>
#include <webview/error_reply.h>

#include <cstring>

using namespace fawkes;

/** @class RRDWebRequestProcessor "rrdweb_processor.h"
 * RRD web request processor.
 * Process web requests to the rrd URL space.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param rrd_manager RRD manager to query
 * @param logger logger to report problems
 * @param baseurl base URL of the RRD webrequest processor
 */
RRDWebRequestProcessor::RRDWebRequestProcessor(fawkes::RRDManager *rrd_manager,
					       fawkes::Logger *logger,
					       const char *baseurl)
  : WebRequestProcessor(/* handle session data */ false)
{
  __rrd_man = rrd_manager;
  __logger  = logger;

  __baseurl = baseurl;
  __baseurl_len = strlen(baseurl);
}


/** Destructor. */
RRDWebRequestProcessor::~RRDWebRequestProcessor()
{
}

WebReply *
RRDWebRequestProcessor::process_request(const char *url,
					const char *method,
					const char *version,
					const char *upload_data,
					size_t *upload_data_size,
					void **session_data)
{
  if ( strncmp(__baseurl, url, __baseurl_len) == 0 ) {
    // It is in our URL prefix range
    std::string subpath = std::string(url).substr(__baseurl_len);

    const RWLockVector<RRDGraphDefinition *> &graphs(__rrd_man->get_graphs());
    RWLockVector<RRDGraphDefinition *>::const_iterator g;

    ScopedRWLock(graphs.rwlock(), true, ScopedRWLock::LOCK_READ);

    if (subpath.find("/graph/") == 0) {
      std::string graph_name = subpath.substr(subpath.find_first_not_of("/", std::string("/graph/").length()));

      for (g = graphs.begin(); g != graphs.end(); ++g) {
	if (strcmp((*g)->get_rrd_def()->get_name(), graph_name.c_str()) == 0) {
	  return new DynamicFileWebReply((*g)->get_filename());
	}
      }
      return new WebErrorPageReply(WebReply::HTTP_NOT_FOUND, "Graph not found");
    } else {
      WebPageReply *r = new WebPageReply("RRD Graphs");
      *r += "<h2>RRD Graphs</h2>\n";

      std::string subpath = std::string(url).substr(__baseurl_len);

      for (g = graphs.begin(); g != graphs.end(); ++g) {
	r->append_body("<img src=\"/rrd/graph/%s\" />\n",
		       (*g)->get_rrd_def()->get_name());
      }

      return r;
    }
  } else {
    return NULL;
  }
}
