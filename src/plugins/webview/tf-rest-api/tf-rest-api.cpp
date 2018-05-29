
/***************************************************************************
 *  tf-rest-api.cpp - Transforms REST API
 *
 *  Created: Wed Apr 11 10:11:50 2018
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

#include "tf-rest-api.h"

#include <webview/rest_api_manager.h>

using namespace fawkes;

/** @class TransformsRestApi "skiller-rest-api.h"
 * REST API backend for the transforms.
 * @author Tim Niemueller
 */

/** Constructor. */
TransformsRestApi::TransformsRestApi()
	: Thread("TransformsRestApi", Thread::OPMODE_WAITFORWAKEUP)
{
}

/** Destructor. */
TransformsRestApi::~TransformsRestApi()
{
}

void
TransformsRestApi::init()
{
	rest_api_ = new WebviewRestApi("transforms", logger);
	rest_api_->add_handler<TransformsGraph>
		(WebRequest::METHOD_GET, "/graph",
		 std::bind(&TransformsRestApi::cb_get_graph, this));
	webview_rest_api_manager->register_api(rest_api_);
}

void
TransformsRestApi::finalize()
{
	webview_rest_api_manager->unregister_api(rest_api_);
	delete rest_api_;
}


void
TransformsRestApi::loop()
{
}


TransformsGraph
TransformsRestApi::cb_get_graph()
{
	try {
		TransformsGraph graph;
		graph.set_kind("TransformsGraph");
		graph.set_apiVersion(TransformsGraph::api_version());
		graph.set_dotgraph(tf_listener->all_frames_as_dot(true));
		return graph;
	} catch (Exception &e) {
		throw WebviewRestException(WebReply::HTTP_INTERNAL_SERVER_ERROR,
		                           "Failed to retrieve transform graph: %s",
		                           e.what_no_backtrace());
	}
}
