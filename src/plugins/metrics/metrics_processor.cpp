
/***************************************************************************
 *  metrics_processor.cpp - Metrics exporter for prometheus request processor
 *
 *  Created: Sat May 06 19:48:50 2017 (German Open 2017)
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

#include "metrics_processor.h"
#include "aspect/metrics_manager.h"

#include <webview/page_reply.h>
#include <webview/request.h>
#include <logging/logger.h>

#include <sstream>
#if GOOGLE_PROTOBUF_VERSION >= 3000000
#  include <google/protobuf/util/json_util.h>
#  include <google/protobuf/util/message_differencer.h>
#endif
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>

#include <algorithm>

using namespace fawkes;

/** @class MetricsRequestProcessor "metrics_processor.h"
 * Metrics web request processor.
 * Process web requests to the metrics URL space.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param manager metrics manager
 * @param logger logger to report problems
 * @param baseurl base URL of the RRD webrequest processor
 */
MetricsRequestProcessor::MetricsRequestProcessor(MetricsManager *metrics_manager,
                                                 fawkes::Logger *logger,
                                                 const std::string &baseurl)
	: metrics_manager_(metrics_manager), logger_(logger), base_url_(baseurl)
{
}


/** Destructor. */
MetricsRequestProcessor::~MetricsRequestProcessor()
{
}

/** Process request.
 * @param request incoming request
 * @return web reply
 */
WebReply *
MetricsRequestProcessor::process_request(const fawkes::WebRequest *request)
{
	std::string accepted_encoding = "text/plain";
	if (request->has_header("Accept")) {
		accepted_encoding = request->header("Accept");
	}
	
	// std::string subpath = request->url().substr(base_url_.length());
	StaticWebReply *reply = new StaticWebReply(WebReply::HTTP_OK);
	
	std::list<io::prometheus::client::MetricFamily> metrics(std::move(metrics_manager_->all_metrics()));

	if (accepted_encoding.find("application/vnd.google.protobuf") != std::string::npos) {
		reply->add_header("Content-type",
		                  "application/vnd.google.protobuf; "
		                  "proto=io.prometheus.client.MetricFamily; "
		                  "encoding=delimited");
		std::ostringstream ss;
		for (auto&& metric : metrics) {
			{
				google::protobuf::io::OstreamOutputStream raw_output{&ss};
				google::protobuf::io::CodedOutputStream output(&raw_output);

				const int size = metric.ByteSize();
				output.WriteVarint32(size);
			}

			std::string buffer;
			metric.SerializeToString(&buffer);
			ss << buffer;
		}
		
		reply->append_body(ss.str());		
	} else if (accepted_encoding.find("application/json") != std::string::npos) {
#if GOOGLE_PROTOBUF_VERSION >= 3000000
		reply->add_header("Content-type", "application/json");
		std::stringstream ss;
		ss << "[";

		for (auto&& metric : metrics) {
			std::string result;
			google::protobuf::util::MessageToJsonString(metric, &result,
			                                            google::protobuf::util::JsonPrintOptions());
			ss << result;
			if (!google::protobuf::util::MessageDifferencer::Equals(metric, metrics.back())) {
				ss << ",";
			}
		}
		ss << "]";
		reply->append_body("%s", ss.str().c_str());
#else
		reply->set_code(WebReply::HTTP_NOT_IMPLEMENTED);
		reply->add_header("Content-type", "text/plain");
		reply->append_body("JSON output only supported with protobuf 3");
#endif
	} else {
		reply->add_header("Content-type", "text/plain; version=0.0.4");
		reply->append_body("# Fawkes Metrics\n");
		for (auto&& metric : metrics) {
			if (metric.metric_size() > 0) {
				reply->append_body("\n");
				if (metric.has_help()) {
					reply->append_body("# HELP %s %s\n", metric.name().c_str(),
					                   metric.help().c_str());
				}
				if (metric.has_type()) {
					const char *typestr = NULL;
					switch (metric.type()) {
					case io::prometheus::client::COUNTER:
						typestr = "counter"; break;
					case io::prometheus::client::GAUGE:
						typestr = "gauge"; break;
					case io::prometheus::client::UNTYPED:
						typestr = "untyped"; break;
					case io::prometheus::client::HISTOGRAM:
						typestr = "histogram"; break;
					case io::prometheus::client::SUMMARY:
						typestr = "summary"; break;
					}
					if (typestr != NULL) {
						reply->append_body("# TYPE %s %s\n", metric.name().c_str(), typestr);
					}
				}
				for (int i = 0; i < metric.metric_size(); ++i) { 				
					
					const io::prometheus::client::Metric &m = metric.metric(i);

					std::string labels;
					if (m.label_size() > 0) {
						std::ostringstream ss;
						ss << " {";
						ss << m.label(0).name() << "=" << m.label(0).value();
						for (int l = 1; l < m.label_size(); ++l) {
							const io::prometheus::client::LabelPair &label = m.label(l);
							ss << "," << label.name() << "=" << label.value();
						}
						ss << "}";
						labels = ss.str();
					}
					std::string timestamp;
					if (m.has_timestamp_ms()) {
						timestamp = " " + std::to_string(m.timestamp_ms());
					}

					switch (metric.type()) {
					case io::prometheus::client::COUNTER:
						if (m.has_counter()) {
							reply->append_body("%s%s %f%s\n", metric.name().c_str(), labels.c_str(),
							                   m.counter().value(), timestamp.c_str());
						} else {
							reply->append_body("# ERROR %s%svalue not set\n",
							                   metric.name().c_str(), labels.c_str());
						}
						break;

					case io::prometheus::client::GAUGE:
						if (m.has_gauge()) {
							reply->append_body("%s%s %f%s\n", metric.name().c_str(), labels.c_str(),
							                   m.gauge().value(), timestamp.c_str());
						} else {
							reply->append_body("# ERROR %s%svalue not set\n",
							                   metric.name().c_str(), labels.c_str());
						}
						break;
	
					case io::prometheus::client::UNTYPED:
						if (m.has_untyped()) {
							reply->append_body("%s%s %f%s\n", metric.name().c_str(), labels.c_str(),
							                   m.untyped().value(), timestamp.c_str());
						} else {
							reply->append_body("# ERROR %s%svalue not set\n",
							                   metric.name().c_str(), labels.c_str());
						}
						break;

					case io::prometheus::client::SUMMARY:
						if (m.has_summary()) {
							const io::prometheus::client::Summary &summary = m.summary();
							for (int q = 0; q < summary.quantile_size(); ++q) {
								const io::prometheus::client::Quantile &quantile = summary.quantile(q);
								std::string q_label;
								if (labels.empty()) {
									q_label = " {quantile=" + std::to_string(quantile.quantile()) + "}";
								} else {
									q_label = labels.substr(0, labels.size() - 1) +
										",quantile=" + std::to_string(quantile.quantile()) + "}";
								}
								reply->append_body("%s%s %f%s\n", metric.name().c_str(),
								                   q_label.c_str(), quantile.value(), timestamp.c_str());
							}
							reply->append_body("%s_sum%s %f%s\n", metric.name().c_str(),
							                   labels.c_str(), summary.sample_sum(), timestamp.c_str());
							reply->append_body("%s_count%s %f%s\n", metric.name().c_str(),
							                   labels.c_str(), summary.sample_count(), timestamp.c_str());
						} else {
							reply->append_body("# ERROR %s%svalue not set\n",
							                   metric.name().c_str(), labels.c_str());
						}
						break;

					case io::prometheus::client::HISTOGRAM:
						if (m.has_histogram()) {
							const io::prometheus::client::Histogram &histogram = m.histogram();
							for (int b = 0; b < histogram.bucket_size(); ++b) {
								const io::prometheus::client::Bucket &bucket = histogram.bucket(b);
								std::string b_label;
								if (labels.empty()) {
									b_label = " {le=" + std::to_string(bucket.upper_bound()) + "}";
								} else {
									b_label = labels.substr(0, labels.size() - 1) +
										",le=" + std::to_string(bucket.upper_bound()) + "}";
								}
								reply->append_body("%s%s %lu%s\n", metric.name().c_str(),
								                   b_label.c_str(), bucket.cumulative_count(), timestamp.c_str());
							}
							reply->append_body("%s_sum%s %f%s\n", metric.name().c_str(),
							                   labels.c_str(), histogram.sample_sum(), timestamp.c_str());
							reply->append_body("%s_count%s %lu%s\n", metric.name().c_str(),
							                   labels.c_str(), histogram.sample_count(), timestamp.c_str());
						} else {
							reply->append_body("# ERROR %s%svalue not set\n",
							                   metric.name().c_str(), labels.c_str());
						}
						break;
					}
				}
			}
		}
	}

	return reply;
}
