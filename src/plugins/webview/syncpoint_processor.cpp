
/***************************************************************************
 *  syncpoint_processor.cpp - Web request processor for SyncPoints
 *
 *  Created: Thu Aug 14 16:21:42 2014
 *  Copyright  2014  Till Hofmann
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

#include "syncpoint_processor.h"
#include <webview/page_reply.h>
#include <webview/file_reply.h>
#include <webview/error_reply.h>

#include <string>
#include <cstring>
#include <cstdlib>
#include <cerrno>

#include <gvc.h>
#include <gvcjob.h>

using namespace fawkes;

/** @class WebviewSyncPointRequestProcessor "tf_processor.h"
 * SyncPoint web request processor.
 * Visualizes SyncPoints.
 * @author Till Hofmann
 */

/** Constructor.
 * @param baseurl Base URL where processor is mounted
 * @param syncpoint_manager SyncPointManager which manages all SyncPoints
 * @param max_age Show only SyncPoint calls which are younger than max_age
 */
WebviewSyncPointRequestProcessor::WebviewSyncPointRequestProcessor(const char *baseurl,
  fawkes::SyncPointManager *syncpoint_manager, float max_age)
{
  baseurl_     = strdup(baseurl);
  baseurl_len_ = strlen(baseurl_);
  syncpoint_manager_ = syncpoint_manager;
  max_age_     = max_age;
}


/** Destructor. */
WebviewSyncPointRequestProcessor::~WebviewSyncPointRequestProcessor()
{
  free(baseurl_);
}


WebReply *
WebviewSyncPointRequestProcessor::process_request(const fawkes::WebRequest *request)
{
  if ( strncmp(baseurl_, request->url().c_str(), baseurl_len_) == 0 ) {
    // It is in our URL prefix range
    std::string subpath = request->url().substr(baseurl_len_);

    if (subpath == "/graph.png") {
      std::string graph = syncpoint_manager_->all_syncpoints_as_dot(max_age_);

      FILE *f = tmpfile();
      if (NULL == f) {
        return new WebErrorPageReply(WebReply::HTTP_INTERNAL_SERVER_ERROR,
            "cannot open tmp file: %s", strerror(errno));
      }
      GVC_t* gvc = gvContext();
      Agraph_t* G = agmemread((char *)graph.c_str());
      gvLayout(gvc, G, (char *)"dot");
      gvRender(gvc, G, "png", f);
      gvFreeLayout(gvc, G);
      agclose(G);
      gvFreeContext(gvc);

      try {
        DynamicFileWebReply *freply = new DynamicFileWebReply(f);
        return freply;
      } catch (fawkes::Exception &e) {
        return new WebErrorPageReply(WebReply::HTTP_INTERNAL_SERVER_ERROR, *(e.begin()));
      }

    } else {
      WebPageReply *r = new WebPageReply("SyncPoints");
      r->append_body("<p><img src=\"%s/graph.png\" /></p>", baseurl_);
      return r;
    }

    WebPageReply *r = new WebPageReply("SyncPoints");
    r->append_body("<p>Hello World</p>");
    return r;
  } else {
    return NULL;
  }
}
