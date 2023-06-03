
/***************************************************************************
 *  metrics_supplier.h - Metrics supplier interface
 *
 *  Created: Sat May 06 20:15:05 2017 (German Open 2017)
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

#ifndef _PLUGINS_METRICS_ASPECT_METRICS_SUPPLIER_H_
#define _PLUGINS_METRICS_ASPECT_METRICS_SUPPLIER_H_

#include <plugins/metrics/protobuf/metrics.pb.h>

#include <list>

namespace fawkes {

class MetricsSupplier
{
public:
	virtual ~MetricsSupplier();

	virtual std::list<io::prometheus::client::MetricFamily> metrics() = 0;
};

} // end namespace fawkes

#endif
