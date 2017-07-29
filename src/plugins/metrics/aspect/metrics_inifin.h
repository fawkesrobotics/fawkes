
/***************************************************************************
 *  metrics_inifin.h - Fawkes MetricsAspect initializer/finalizer
 *
 *  Created: Fri Jul 28 21:33:03 2017
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

#ifndef __PLUGINS_METRICS_ASPECT_METRICS_INIFIN_H_
#define __PLUGINS_METRICS_ASPECT_METRICS_INIFIN_H_

#include <aspect/inifins/inifin.h>
#include <plugins/metrics/aspect/metrics.h>
#include <plugins/metrics/aspect/metrics_manager.h>
#include <core/utils/lockptr.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class Logger;
class MetricsManager;

class MetricsAspectIniFin : public AspectIniFin
{
 public:
  MetricsAspectIniFin();
  ~MetricsAspectIniFin();

  virtual void init(Thread *thread);
  virtual void finalize(Thread *thread);

  void set_manager(MetricsManager * supplier_mgr);

 private:
  MetricsManager * metrics_mgr_;
};

} // end namespace fawkes

#endif
