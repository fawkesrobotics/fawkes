/***************************************************************************
 *  metrics.cpp - Metrics aspect for Fawkes
 *
 *  Created: Fri Jul 28 20:10:20 2017
 *  Copyright  2006-2017  Tim Niemueller [www.niemueller.de]
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

#include <plugins/metrics/aspect/metrics.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class MetricsAspect <plugins/metrics/aspect/metrics_supplier.h>
 * Thread aspect to provide metrics.

 * @ingroup Aspects
 * @author Tim Niemueller
 */

/** Constructor.
 * @param metrics_supplier metrics supplier
 */
MetricsAspect::MetricsAspect(MetricsSupplier *metrics_supplier)
{
  add_aspect("MetricsAspect");
  metrics_supplier_ = metrics_supplier;
}


/** Virtual empty destructor. */
MetricsAspect::~MetricsAspect()
{
}


/** Get metrics supplier of this thread.
 * @return metrics supplier
 */
MetricsSupplier *
MetricsAspect::get_metrics_supplier() const
{
	return metrics_supplier_;
}


} // end namespace fawkes
