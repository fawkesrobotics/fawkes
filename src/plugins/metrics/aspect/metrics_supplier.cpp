
/***************************************************************************
 *  metrics_supplier.cpp - Metrics supplier interface
 *
 *  Created: Sat May 06 20:16:52 2017 (German Open 2017)
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

#include "metrics_supplier.h"

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class MetricsSupplier
 * Metrics supplier class.
 * @author Tim Niemueller
 *
 * @fn std::list<io::prometheus::client::MetricFamily>  MetricsSupplier::metrics() = 0
 * Metrics this supplier hosts.
 * @return list of metric families
 */

/** Virtual empty destructor. */
MetricsSupplier::~MetricsSupplier()
{
}

} // end namespace fawkes
