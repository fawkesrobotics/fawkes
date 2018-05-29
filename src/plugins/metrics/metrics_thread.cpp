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

#include <core/threading/mutex_locker.h>

#include <interfaces/MetricCounterInterface.h>
#include <interfaces/MetricGaugeInterface.h>
#include <interfaces/MetricUntypedInterface.h>
#include <interfaces/MetricHistogramInterface.h>

#include <webview/url_manager.h>
#include <utils/misc/string_split.h>

#include <algorithm>
#include <functional>
#include <chrono>

using namespace fawkes;

#define CFG_PREFIX "/metrics/"
#define URL_PREFIX "/metrics"

/** @class MetricsThread "metrics_thread.h"
 * Thread to export metrics for Prometheus.
 * @author Tim Niemueller
 */

/** Constructor. */
MetricsThread::MetricsThread()
	: Thread("MetricsThread", Thread::OPMODE_WAITFORWAKEUP),
	  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_POST_LOOP),
	  AspectProviderAspect(&metrics_aspect_inifin_),
	  BlackBoardInterfaceListener("MetricsThread")
{
}


/** Destructor. */
MetricsThread::~MetricsThread()
{
}

void
MetricsThread::init()
{
	metrics_aspect_inifin_.set_manager(this);
	
	bbio_add_observed_create("MetricFamilyInterface", "*");
  blackboard->register_observer(this);

	MutexLocker lock(metric_bbs_.mutex());
  std::list<MetricFamilyInterface *> ifaces =
	  blackboard->open_multiple_for_reading<MetricFamilyInterface>("*");

  for (auto & i : ifaces) {
	  logger->log_info(name(), "Got metric family %s", i->id());
	  i->read();
	  MetricFamilyBB mfbb{.metric_family = i, .metric_type = i->metric_type()};
	  metric_bbs_[i->id()] = mfbb;
	  
	  if (! conditional_open(i->id(), metric_bbs_[i->id()])) {
		  bbil_add_data_interface(i);
	  }
	}

  blackboard->register_listener(this);

  lock.unlock();

  imf_loop_count_ = std::make_shared<io::prometheus::client::MetricFamily>();
  imf_loop_count_->set_name("fawkes_loop_count");
  imf_loop_count_->set_help("Number of Fawkes main loop iterations");
  imf_loop_count_->set_type(io::prometheus::client::COUNTER);
  imf_loop_count_->add_metric();
  internal_metrics_.push_back(imf_loop_count_);

  imf_metrics_requests_ = std::make_shared<io::prometheus::client::MetricFamily>();
  imf_metrics_requests_->set_name("fawkes_metrics_requests");
  imf_metrics_requests_->set_help("Number of requests for metrics");
  imf_metrics_requests_->set_type(io::prometheus::client::COUNTER);
  imf_metrics_requests_->add_metric();
  internal_metrics_.push_back(imf_metrics_requests_);

  try {
		std::vector<float> buckets_le = config->get_floats("/metrics/internal/metrics_requests/buckets");

		if (! buckets_le.empty()) {
			std::sort(buckets_le.begin(), buckets_le.end());

			imf_metrics_proctime_ = std::make_shared<io::prometheus::client::MetricFamily>();
			imf_metrics_proctime_->set_name("fawkes_metrics_proctime");
			imf_metrics_proctime_->set_help("Time required to process metrics");
			imf_metrics_proctime_->set_type(io::prometheus::client::HISTOGRAM);
			auto m = imf_metrics_proctime_->add_metric();
		  auto h = m->mutable_histogram();
		  for (float &b : buckets_le) {
			  h->add_bucket()->set_upper_bound(b);
		  }
		  internal_metrics_.push_back(imf_metrics_proctime_);
		}
  } catch (Exception &e) {
	  logger->log_warn(name(), "Internal metric metrics_proctime bucket bounds not configured, disabling");
  }

  metrics_suppliers_.push_back(this);

  req_proc_ = new MetricsRequestProcessor(this, logger, URL_PREFIX);
  webview_url_manager->add_handler(WebRequest::METHOD_GET, URL_PREFIX,
	                                 std::bind(&MetricsRequestProcessor::process_request,
	                                           req_proc_, std::placeholders::_1));
}

void
MetricsThread::finalize()
{
	webview_url_manager->remove_handler(WebRequest::METHOD_GET, URL_PREFIX);
	delete req_proc_;
}


void
MetricsThread::loop()
{
	imf_loop_count_->mutable_metric(0)->mutable_counter()->set_value
		(imf_loop_count_->metric(0).counter().value() + 1);
}


void
MetricsThread::bb_interface_created(const char *type, const char *id) throw()
{
  MutexLocker lock(metric_bbs_.mutex());
  MetricFamilyInterface *mfi;
  try {
    mfi = blackboard->open_for_reading<MetricFamilyInterface>(id);
    logger->log_info(name(), "Opened %s:%s", type, id);
  } catch (Exception &e) {
    // ignored
    logger->log_warn(name(), "Failed to open %s:%s: %s", type, id, e.what_no_backtrace());
    return;
  }

  try {
    bbil_add_reader_interface(mfi);
    bbil_add_writer_interface(mfi);
    bbil_add_data_interface(mfi);
    blackboard->update_listener(this);
  } catch (Exception &e) {
    logger->log_warn(name(), "Failed to register for %s:%s: %s",
                     type, id, e.what());
    try {
      bbil_remove_reader_interface(mfi);
      bbil_remove_writer_interface(mfi);
      blackboard->update_listener(this);
      blackboard->close(mfi);
    } catch (Exception &e) {
      logger->log_error(name(), "Failed to deregister %s:%s during error recovery: %s",
                        type, id, e.what());
    }
    return;
  }
  MetricFamilyBB mfbb{.metric_family = mfi, .metric_type = mfi->metric_type()};
  metric_bbs_[id] = mfbb;
}


std::list<io::prometheus::client::MetricFamily>
MetricsThread::metrics()
{
	std::chrono::high_resolution_clock::time_point proc_start =
		std::chrono::high_resolution_clock::now();
	
	imf_metrics_requests_->mutable_metric(0)->mutable_counter()->set_value
		(imf_metrics_requests_->metric(0).counter().value() + 1);

	std::list<io::prometheus::client::MetricFamily> rv;

  MutexLocker lock(metric_bbs_.mutex());
  for (auto & mbbp : metric_bbs_) {
		auto & mfbb = mbbp.second;

		io::prometheus::client::MetricFamily mf;
		mfbb.metric_family->read();
		mf.set_name(mfbb.metric_family->name());
		mf.set_help(mfbb.metric_family->help());

		switch (mfbb.metric_type) {
		case MetricFamilyInterface::COUNTER:
			mf.set_type(io::prometheus::client::COUNTER);
			for (const auto &d : mfbb.data) {
				d.counter->read();
				io::prometheus::client::Metric *m = mf.add_metric();
				parse_labels(d.counter->labels(), m);
				m->mutable_counter()->set_value(d.counter->value());
			}
			break;

		case MetricFamilyInterface::GAUGE:
			mf.set_type(io::prometheus::client::GAUGE);
			for (const auto &d : mfbb.data) {
				d.gauge->read();
				io::prometheus::client::Metric *m = mf.add_metric();
				parse_labels(d.gauge->labels(), m);
				m->mutable_gauge()->set_value(d.gauge->value());
			}
			break;

		case MetricFamilyInterface::UNTYPED:
			mf.set_type(io::prometheus::client::UNTYPED);
			for (const auto &d : mfbb.data) {
				d.untyped->read();
				io::prometheus::client::Metric *m = mf.add_metric();
				parse_labels(d.untyped->labels(), m);
				m->mutable_untyped()->set_value(d.untyped->value());
			}
			break;

		case MetricFamilyInterface::HISTOGRAM:
			mf.set_type(io::prometheus::client::HISTOGRAM);
			for (const auto &d : mfbb.data) {
				d.histogram->read();
				io::prometheus::client::Metric *m = mf.add_metric();
				parse_labels(d.histogram->labels(), m);
				io::prometheus::client::Histogram *h = m->mutable_histogram();
				h->set_sample_count(d.histogram->sample_count());
				h->set_sample_sum(d.histogram->sample_sum());
				for (unsigned int i = 0; i < d.histogram->bucket_count(); ++i) {
					io::prometheus::client::Bucket *b = h->add_bucket();
					b->set_cumulative_count(d.histogram->bucket_cumulative_count(i));
					b->set_upper_bound(d.histogram->bucket_upper_bound(i));
				}
			}
			break;

		case MetricFamilyInterface::NOT_INITIALIZED:
			// ignore
			break;
		}
		rv.push_back(std::move(mf));
	}

  if (imf_metrics_proctime_) {
	  std::chrono::high_resolution_clock::time_point proc_end =
		  std::chrono::high_resolution_clock::now();
	  const std::chrono::duration<double> proc_diff = proc_end - proc_start;
	  for (int i = 0; i < imf_metrics_proctime_->metric(0).histogram().bucket_size(); ++i) {
		  io::prometheus::client::Histogram *h =
			  imf_metrics_proctime_->mutable_metric(0)->mutable_histogram();
		  if (proc_diff.count() < h->bucket(i).upper_bound()) {
			  io::prometheus::client::Bucket *b = h->mutable_bucket(i);
			  b->set_cumulative_count(b->cumulative_count() + 1);
			  h->set_sample_count(h->sample_count() + 1);
			  h->set_sample_sum(h->sample_sum() + proc_diff.count());
			  break;
		  }
	  }
  }

  for (auto &im : internal_metrics_) {
		rv.push_back(std::move(*im));
	}

	return rv;
}


std::list<io::prometheus::client::MetricFamily>
MetricsThread::all_metrics()
{
	std::list<io::prometheus::client::MetricFamily> metrics;

	for (auto & s : metrics_suppliers_) {
		metrics.splice(metrics.begin(), std::move(s->metrics()));
	}

	return metrics;
}


void
MetricsThread::add_supplier(MetricsSupplier *supplier)
{
	MutexLocker lock(metrics_suppliers_.mutex());
	auto i = std::find(metrics_suppliers_.begin(), metrics_suppliers_.end(), supplier);
	if (i == metrics_suppliers_.end()) {
		metrics_suppliers_.push_back(supplier);
	}
}

void
MetricsThread::remove_supplier(MetricsSupplier *supplier)
{
	MutexLocker lock(metrics_suppliers_.mutex());
	auto i = std::find(metrics_suppliers_.begin(), metrics_suppliers_.end(), supplier);
	if ( i != metrics_suppliers_.end()) {
		metrics_suppliers_.erase(i);
	}
}

const fawkes::LockList<MetricsSupplier *> &
MetricsThread::metrics_suppliers() const
{
	return metrics_suppliers_;
}


void
MetricsThread::parse_labels(const std::string &labels, io::prometheus::client::Metric *m)
{
	std::vector<std::string> labelv = str_split(labels, ',');
	for (const std::string &l : labelv) {
		std::vector<std::string> key_value = str_split(l, '=');
		if (key_value.size() == 2) {
			io::prometheus::client::LabelPair *lp = m->add_label();
			lp->set_name(key_value[0]);
			lp->set_value(key_value[1]);
		} else {
			logger->log_warn(name(), "Invalid label '%s'", l.c_str());
		}
	}
}


void
MetricsThread::bb_interface_writer_removed(fawkes::Interface *interface,
                                           unsigned int instance_serial) throw()
{
	conditional_close(interface);
}

void
MetricsThread::bb_interface_reader_removed(fawkes::Interface *interface,
                                           unsigned int instance_serial) throw()
{
	conditional_close(interface);
}

void
MetricsThread::bb_interface_data_changed(fawkes::Interface *interface) throw()
{
	MetricFamilyInterface *mfi = dynamic_cast<MetricFamilyInterface *>(interface);
	if (! mfi) return;
	if (! mfi->has_writer()) return;

	mfi->read();
	if (mfi->metric_type() == MetricFamilyInterface::NOT_INITIALIZED) {
		logger->log_warn(name(), "Got data changed event for %s which is not yet initialized",
		                 mfi->uid());
		return;
	}

	MutexLocker lock(metric_bbs_.mutex());
	if (metric_bbs_.find(mfi->id()) == metric_bbs_.end()) {
		logger->log_warn(name(), "Got data changed event for %s which is not registered",
		                 mfi->uid());
		return;
	}

	metric_bbs_[mfi->id()].metric_type = mfi->metric_type();
	if (conditional_open(mfi->id(), metric_bbs_[mfi->id()])) {
		bbil_remove_data_interface(mfi);
		blackboard->update_listener(this);
	}
}


bool
MetricsThread::conditional_open(const std::string &id, MetricFamilyBB &mfbb)
{
	mfbb.metric_family->read();

  std::string data_id_pattern=id + "/*";

  switch (mfbb.metric_type) {
  case MetricFamilyInterface::COUNTER:
	  {
		  std::list<MetricCounterInterface *> ifaces =
			  blackboard->open_multiple_for_reading<MetricCounterInterface>(data_id_pattern.c_str());
		  if (ifaces.empty())  return false;
		  std::transform(ifaces.begin(), ifaces.end(), std::back_inserter(mfbb.data),
		                 [](MetricCounterInterface *iface) { return MetricFamilyData{.counter=iface}; });
	  }
	  break;

  case MetricFamilyInterface::GAUGE:
	  {
		  std::list<MetricGaugeInterface *> ifaces =
			  blackboard->open_multiple_for_reading<MetricGaugeInterface>(data_id_pattern.c_str());
		  if (ifaces.empty())  return false;
		  std::transform(ifaces.begin(), ifaces.end(), std::back_inserter(mfbb.data),
		                 [](MetricGaugeInterface *iface) { return MetricFamilyData{.gauge=iface}; });
	  }
	  break;

  case MetricFamilyInterface::UNTYPED:
	  {
		  std::list<MetricUntypedInterface *> ifaces =
			  blackboard->open_multiple_for_reading<MetricUntypedInterface>(data_id_pattern.c_str());
		  if (ifaces.empty())  return false;
		  std::transform(ifaces.begin(), ifaces.end(), std::back_inserter(mfbb.data),
		                 [](MetricUntypedInterface *iface) { return MetricFamilyData{.untyped=iface}; });
	  }
	  break;

  case MetricFamilyInterface::HISTOGRAM:
	  {
		  std::list<MetricHistogramInterface *> ifaces =
			  blackboard->open_multiple_for_reading<MetricHistogramInterface>(data_id_pattern.c_str());
		  if (ifaces.empty())  return false;
		  std::transform(ifaces.begin(), ifaces.end(), std::back_inserter(mfbb.data),
		                 [](MetricHistogramInterface *iface) { return MetricFamilyData{.histogram=iface}; });
	  }
	  break;

  case MetricFamilyInterface::NOT_INITIALIZED:
		logger->log_info(name(), "Metric family %s not yet initialized", id.c_str());
	  return false;
  }

  logger->log_info(name(), "Initialized metric %s", id.c_str());
  return true;
}


void
MetricsThread::conditional_close(Interface *interface) throw()
{
	MetricFamilyInterface *mfi = dynamic_cast<MetricFamilyInterface *>(interface);
	if (! mfi)  return;

  MutexLocker lock(metric_bbs_.mutex());

  if (metric_bbs_.find(mfi->id()) == metric_bbs_.end()) {
	  logger->log_warn(name(), "Called to close %s whic was not opened", mfi->uid());
	  return;
  }

  logger->log_info(name(), "Last on metric family %s, closing", interface->id());
  auto & mfbb(metric_bbs_[mfi->id()]);

  switch (mfbb.metric_type) {
  case MetricFamilyInterface::COUNTER:
	  std::for_each(mfbb.data.begin(), mfbb.data.end(),
	                [this](auto &d) { this->blackboard->close(d.counter); });
	  break;

  case MetricFamilyInterface::GAUGE:
	  std::for_each(mfbb.data.begin(), mfbb.data.end(),
	                [this](auto &d) { this->blackboard->close(d.gauge); });
	  break;
  case MetricFamilyInterface::UNTYPED:
	  std::for_each(mfbb.data.begin(), mfbb.data.end(),
	                [this](auto &d) { this->blackboard->close(d.untyped); });
	  break;
  case MetricFamilyInterface::HISTOGRAM:
	  std::for_each(mfbb.data.begin(), mfbb.data.end(),
	                [this](auto &d) { this->blackboard->close(d.histogram); });
	  break;
  case MetricFamilyInterface::NOT_INITIALIZED:
	  bbil_remove_data_interface(mfi);
	  break;
  }

  metric_bbs_.erase(mfi->id());
  lock.unlock();

  std::string uid = interface->uid();
  try {
	  bbil_remove_reader_interface(interface);
	  bbil_remove_writer_interface(interface);
	  blackboard->update_listener(this);
	  blackboard->close(interface);
  } catch (Exception &e) {
	  logger->log_error(name(), "Failed to unregister or close %s: %s",
	                    uid.c_str(), e.what());
  }
}
