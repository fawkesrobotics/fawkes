
/***************************************************************************
 *  rrdweb_thread.cpp - RRD Webview Thread
 *
 *  Created: Tue Dec 21 01:05:45 2010
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

#include "rrdweb_thread.h"
#include "rrdweb_processor.h"

#include <webview/url_manager.h>
#include <webview/nav_manager.h>

#include <functional>

using namespace fawkes;

#define RRD_URL_PREFIX "/rrd"

/** @class RRDWebThread "rrd_thread.h"
 * RRD Webview Thread.
 * This thread queries RRD graphs from RRDManager obtained via RRDAspect
 * and displays them on a Webview page.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
RRDWebThread::RRDWebThread()
  : Thread("RRDWebThread", Thread::OPMODE_WAITFORWAKEUP)
{
}


/** Destructor. */
RRDWebThread::~RRDWebThread()
{
}


void
RRDWebThread::init()
{
  __processor  = new RRDWebRequestProcessor(rrd_manager, logger);
  webview_url_manager->add_handler(WebRequest::METHOD_GET, "/rrd/graph/{graph}",
                                   std::bind(&RRDWebRequestProcessor::process_graph, __processor,
                                             std::placeholders::_1));
  webview_url_manager->add_handler(WebRequest::METHOD_GET, "/rrd/?",
                                   std::bind(&RRDWebRequestProcessor::process_overview, __processor));
  webview_nav_manager->add_nav_entry(RRD_URL_PREFIX, "RRD Graphs");
}


void
RRDWebThread::finalize()
{
	webview_url_manager->remove_handler(WebRequest::METHOD_GET, "/rrd/graph/{graph}");
	webview_url_manager->remove_handler(WebRequest::METHOD_GET, "/rrd/?");
  webview_nav_manager->remove_nav_entry(RRD_URL_PREFIX);
  delete __processor;
}


void
RRDWebThread::loop()
{
}

