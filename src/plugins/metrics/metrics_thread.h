/***************************************************************************
 *  metrics_thread.h - Metrics exporter for Prometheus plugin
 *
 *  Created: Sat May 06 19:43:10 2017 (German Open 2017)
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

#ifndef __PLUGINS_METRICS_METRICS_THREAD_H_
#define __PLUGINS_METRICS_METRICS_THREAD_H_

#include "metrics_supplier.h"

#include <core/threading/thread.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/webview.h>

class MetricsRequestProcessor;

class MetricsThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
	public fawkes::BlackBoardAspect,
	public fawkes::WebviewAspect,
	public MetricsSupplier
{
 public:
  MetricsThread();
  virtual ~MetricsThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

  /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run();}

  virtual std::list<io::prometheus::client::MetricFamily>  metrics();

 private:
  MetricsRequestProcessor *req_proc_;
};

#endif
