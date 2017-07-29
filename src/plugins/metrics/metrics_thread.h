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

#include "aspect/metrics_supplier.h"
#include "aspect/metrics_inifin.h"

#include <core/threading/thread.h>
#include <core/utils/lock_map.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/webview.h>
#include <aspect/blocked_timing.h>
#include <aspect/aspect_provider.h>

#include <blackboard/interface_observer.h>
#include <blackboard/interface_listener.h>

#include <interfaces/MetricFamilyInterface.h>

#include <memory>

class MetricsRequestProcessor;

namespace fawkes {
	class MetricCounterInterface;
	class MetricGaugeInterface;
	class MetricUntypedInterface;
	class MetricHistogramInterface;
	//MetricSummaryInterface;
}

namespace io {
	namespace prometheus {
		namespace client {
			class MetricFamily;
		}
	}
}

class MetricsThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
	public fawkes::BlackBoardAspect,
	public fawkes::WebviewAspect,
	public fawkes::BlockedTimingAspect,
	public fawkes::AspectProviderAspect,
  public fawkes::BlackBoardInterfaceObserver,
	public fawkes::BlackBoardInterfaceListener,
	public fawkes::MetricsSupplier,
	public fawkes::MetricsManager
{
 public:
  MetricsThread();
  virtual ~MetricsThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

  /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run();}

 private:
  typedef union {
	  fawkes::MetricCounterInterface *    counter;
	  fawkes::MetricGaugeInterface *      gauge;
	  fawkes::MetricUntypedInterface *    untyped;
	  fawkes::MetricHistogramInterface *  histogram;
	  //fawkes::MetricSummaryInterface *  summary;
  } MetricFamilyData;
  
  typedef struct {
	  fawkes::MetricFamilyInterface *  metric_family;
	  fawkes::MetricFamilyInterface::MetricType metric_type;
	  std::list<MetricFamilyData>      data;
  } MetricFamilyBB;

 private:
  // for BlackBoardInterfaceObserver
  virtual void bb_interface_created(const char *type, const char *id) throw();

  // for BlackBoardInterfaceListener
  virtual void bb_interface_writer_removed(fawkes::Interface *interface,
                                           unsigned int instance_serial) throw();
  virtual void bb_interface_reader_removed(fawkes::Interface *interface,
                                           unsigned int instance_serial) throw();
  virtual void bb_interface_data_changed(fawkes::Interface *interface) throw();

  // for MetricsSupplier
  virtual std::list<io::prometheus::client::MetricFamily>  metrics();

  // for MetricsManager
  virtual std::list<io::prometheus::client::MetricFamily>  all_metrics();

  virtual void add_supplier(MetricsSupplier *supplier);
  virtual void remove_supplier(MetricsSupplier *supplier);

  virtual const fawkes::LockList<MetricsSupplier *> &  metrics_suppliers() const;


  bool conditional_open(const std::string &id, MetricFamilyBB &mfbb);
  void conditional_close(fawkes::Interface *interface) throw();
  void parse_labels(const std::string &labels, io::prometheus::client::Metric *m);
  
 private: 
  MetricsRequestProcessor *req_proc_;
  fawkes::LockMap<std::string, MetricFamilyBB>  metric_bbs_;

  fawkes::MetricsAspectIniFin  metrics_aspect_inifin_;

  // Internal metric families
  std::shared_ptr<io::prometheus::client::MetricFamily> imf_loop_count_;
  std::shared_ptr<io::prometheus::client::MetricFamily> imf_metrics_requests_;
  std::shared_ptr<io::prometheus::client::MetricFamily> imf_metrics_proctime_;

  std::vector<std::shared_ptr<io::prometheus::client::MetricFamily>> internal_metrics_;

  fawkes::LockList<MetricsSupplier *>  metrics_suppliers_;
};

#endif
