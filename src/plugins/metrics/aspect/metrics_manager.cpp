
/***************************************************************************
 *  metrics_manager.cpp - Metrics manager
 *
 *  Created: Sat Jul 29 00:35:59 2017
 *  Copyright  2006-2017  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#include <plugins/metrics/aspect/metrics_manager.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class MetricsManager <plugins/metrics/aspect/metrics_manager.h>
 * Base class for metrics managers.
 *
 * @fn std::list<io::prometheus::client::MetricFamily>  MetricsManager::all_metrics()
 * Get combination of all metrics.
 * @return list of currently available metrics
 *
 * @fn void MetricsManager::add_supplier(MetricsSupplier *supplier)
 * Add metrics supplier.
 * @param supplier supplier to add
 *
 * @fn void MetricsManager::remove_supplier(MetricsSupplier *supplier)
 * Remove metrics supplier.
 * @param supplier supplier to remove
 *
 * @fn const std::LockList<MetricsSupplier *> &  MetricsManager::metrics_suppliers() const
 * Get list of current metrics suppliers.
 * @return list of metrics suppliers


 * @author Tim Niemueller
 */

/** Virtual empty destructor. */
MetricsManager::~MetricsManager()
{
}

} // end namespace fawkes
