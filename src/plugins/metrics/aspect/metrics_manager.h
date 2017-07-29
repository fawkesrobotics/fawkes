
/***************************************************************************
 *  metrics_manager.h - Metrics supplier interface
 *
 *  Created: Fri Jul 28 22:08:16 2017
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

#ifndef __PLUGINS_METRICS_ASPECT_METRICS_MANAGER_H_
#define __PLUGINS_METRICS_ASPECT_METRICS_MANAGER_H_

#include <core/utils/lock_list.h>
#include <plugins/metrics/protobuf/metrics.pb.h>
#include <plugins/metrics/aspect/metrics_supplier.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class MetricsManager
{
 public:
  virtual ~MetricsManager();

  virtual std::list<io::prometheus::client::MetricFamily> all_metrics() = 0;

  virtual void add_supplier(MetricsSupplier *supplier) = 0;
  virtual void remove_supplier(MetricsSupplier *supplier) = 0;

  virtual const fawkes::LockList<MetricsSupplier *> &  metrics_suppliers() const = 0;
};

} // end namespace fawkes

#endif
