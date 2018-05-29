
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
#include <core/exception.h>
#include <webview/page_reply.h>
#include <webview/file_reply.h>
#include <webview/error_reply.h>
#include <webview/request.h>

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
 */
RRDWebRequestProcessor::RRDWebRequestProcessor(fawkes::RRDManager *rrd_manager,
                                               fawkes::Logger *logger)
{
  __rrd_man = rrd_manager;
  __logger  = logger;
}


/** Destructor. */
RRDWebRequestProcessor::~RRDWebRequestProcessor()
{
}

/** Process request for graph.
 * @param request incoming request, must have "graph" path argument.
 * @return web reply
 */
WebReply *
RRDWebRequestProcessor::process_graph(const fawkes::WebRequest *request)
{
	const RWLockVector<RRDGraphDefinition *> &graphs(__rrd_man->get_graphs());
	RWLockVector<RRDGraphDefinition *>::const_iterator g;

	ScopedRWLock(graphs.rwlock(), ScopedRWLock::LOCK_READ);

	std::string graph_name = request->path_arg("graph");

	for (g = graphs.begin(); g != graphs.end(); ++g) {
		if (graph_name == (*g)->get_name()) {
			try {
				return new DynamicFileWebReply((*g)->get_filename());
			} catch (Exception &e) {
				return new WebErrorPageReply(WebReply::HTTP_NOT_FOUND, e.what());
			}
		}
	}
	return new WebErrorPageReply(WebReply::HTTP_NOT_FOUND, "Graph not found");
}

/** Process incoming request for overview.
 * @return web reply
 */
WebReply *
RRDWebRequestProcessor::process_overview()
{
	const RWLockVector<RRDGraphDefinition *> &graphs(__rrd_man->get_graphs());

	WebPageReply *r = new WebPageReply("RRD Graphs");
	r->set_html_header("  <link rel=\"stylesheet\" type=\"text/css\" "
	                   "href=\"/static/css/rrdweb.css\" />\n");
	*r += "<h2>RRD Graphs</h2>\n";

	unsigned int i = 0;
	*r += "<table class=\"rrdgrid\">";
	for (auto g : graphs) {
		if ((i % 2) == 0) *r += "  <tr>";
		r->append_body("<td class=\"%s\"><img src=\"/rrd/graph/%s\" /></td>",
		               ((i % 2) == 0) ? "left" : "right",
		               g->get_name());
		if ((i++ % 2) == 1) *r += "  </tr>\n";
	}
	*r += "</table>";

	return r;
}
