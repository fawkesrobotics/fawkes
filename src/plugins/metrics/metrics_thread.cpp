/***************************************************************************
 *  metrics_thread.cpp - Metrics exporter for Prometheus plugin
 *
 *  Created: Sat May 06 19:44:55 2017 (German Open 2017)
 *  Copyright  2017  Tim Niemueller [www.niemueller.de]
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

#include "metrics_thread.h"
#include "metrics_processor.h"

#include <webview/url_manager.h>

using namespace fawkes;

#define CFG_PREFIX "/metrics/"
#define URL_PREFIX "/metrics"

/** @class MetricsThread "metrics_thread.h"
 * Thread to export metrics for Prometheus.
 * @author Tim Niemueller
 */

/** Constructor. */
MetricsThread::MetricsThread()
  : Thread("MetricsThread", Thread::OPMODE_WAITFORWAKEUP)
{
}


/** Destructor. */
MetricsThread::~MetricsThread()
{
}

void
MetricsThread::init()
{
	req_proc_ = new MetricsRequestProcessor(this, logger, URL_PREFIX);
	webview_url_manager->register_baseurl(URL_PREFIX, req_proc_);
}

void
MetricsThread::finalize()
{
	webview_url_manager->unregister_baseurl(URL_PREFIX);
	delete req_proc_;
}


void
MetricsThread::loop()
{
}

std::list<io::prometheus::client::MetricFamily>
MetricsThread::metrics()
{
	std::list<io::prometheus::client::MetricFamily> rv;
	return rv;
}
