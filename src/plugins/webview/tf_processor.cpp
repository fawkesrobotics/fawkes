
/***************************************************************************
 *  tf_processor.cpp - Web request processor for TF data
 *
 *  Created: Wed Jun 19 17:45:47 2013
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

#include "tf_processor.h"
#include <webview/page_reply.h>
#include <webview/file_reply.h>
#include <webview/error_reply.h>
#include <webview/url_manager.h>

#include <string>
#include <cstring>
#include <cstdlib>
#include <cerrno>
#include <functional>

#include <gvc.h>
#include <gvcjob.h>


using namespace fawkes;

/** @class WebviewTfRequestProcessor "tf_processor.h"
 * Transfrom data web request processor.
 * Provides access to the transforms data.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param url_manager URL manager to register with
 * @param transformer transformer listener to query for DOT graph
 */
WebviewTfRequestProcessor::WebviewTfRequestProcessor(fawkes::WebUrlManager *url_manager,
                                                     fawkes::tf::Transformer *transformer)
{
  transformer_ = transformer;
  url_manager_    = url_manager;

  url_manager_->add_handler(WebRequest::METHOD_GET, "/tf/graph.png",
                            std::bind(&WebviewTfRequestProcessor::process_graph, this));
  url_manager_->add_handler(WebRequest::METHOD_GET, "/tf/?",
                            std::bind(&WebviewTfRequestProcessor::process_overview, this));
}


/** Destructor. */
WebviewTfRequestProcessor::~WebviewTfRequestProcessor()
{
	url_manager_->remove_handler(WebRequest::METHOD_GET, "/tf/?");
	url_manager_->remove_handler(WebRequest::METHOD_GET, "/tf/graph.png");
}


WebReply *
WebviewTfRequestProcessor::process_graph()
{
	std::string graph = transformer_->all_frames_as_dot(true);

	FILE *f = tmpfile();
	if (NULL == f) {
		return new WebErrorPageReply(WebReply::HTTP_INTERNAL_SERVER_ERROR,
		                             "Cannot open temp file: %s", strerror(errno));
	}

	GVC_t* gvc = gvContext(); 
	Agraph_t* G = agmemread((char *)graph.c_str());
	gvLayout(gvc, G, (char *)"dot");
	gvRender(gvc, G, (char *)"png", f);
	gvFreeLayout(gvc, G);
	agclose(G);    
	gvFreeContext(gvc);

	try {
		DynamicFileWebReply *freply = new DynamicFileWebReply(f);
		return freply;
	} catch (fawkes::Exception &e) {
		return new WebErrorPageReply(WebReply::HTTP_INTERNAL_SERVER_ERROR, *(e.begin()));
	}
}

WebReply *
WebviewTfRequestProcessor::process_overview()
{
	WebPageReply *r = new WebPageReply("Transforms");
	r->append_body("<p><img src=\"/tf/graph.png\" /></p>");
	//r->append_body("<pre>%s</pre>", transformer_->all_frames_as_dot().c_str());
	return r;
}
