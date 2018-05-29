
/***************************************************************************
 *  metrics_processor.h - Metrics exporter
 *
 *  Created: Sat May 06 19:46:38 2017 (German Open 2017)
 *  Copyright  2017-2018  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_METRICS_METRICS_PROCESSOR_H_
#define __PLUGINS_METRICS_METRICS_PROCESSOR_H_

#include "protobuf/metrics.pb.h"

namespace fawkes {
  class Logger;
  class MetricsManager;
  class WebReply;
  class WebRequest;
}

class MetricsRequestProcessor
{
 public:
	MetricsRequestProcessor(fawkes::MetricsManager *manager,
	                        fawkes::Logger *logger,
	                        const std::string &base_url);

  virtual ~MetricsRequestProcessor();

  fawkes::WebReply * process_request(const fawkes::WebRequest *request);

 private:
  fawkes::MetricsManager *  metrics_manager_;
  fawkes::Logger         *  logger_;
  std::string               base_url_;
};

#endif
