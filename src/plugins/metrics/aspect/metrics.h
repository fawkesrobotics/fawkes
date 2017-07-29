
/***************************************************************************
 *  metrics.h - Metrics aspect for Fawkes
 *
 *  Created: Fri Jul 28 20:07:43 2017
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

#ifndef __PLUGINS_METRICS_ASPECT_METRICS_H_
#define __PLUGINS_METRICS_ASPECT_METRICS_H_

#include <aspect/aspect.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class MetricsAspectIniFin;
class MetricsSupplier;

class MetricsAspect : public virtual Aspect
{
	friend MetricsAspectIniFin;

 public:
	MetricsAspect(MetricsSupplier *metrics_supplier) __attribute__((nonnull));
	virtual ~MetricsAspect();

 private:
	MetricsSupplier *  get_metrics_supplier() const;
	
 private:
	MetricsSupplier *  metrics_supplier_;
};

} // end namespace fawkes

#endif
