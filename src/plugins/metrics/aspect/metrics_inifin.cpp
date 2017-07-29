
/***************************************************************************
 *  metrics_inifin.cpp - Fawkes MetricsAspect initializer/finalizer
 *
 *  Created: Fri Jul 28 21:34:47 2017
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

#include <plugins/metrics/aspect/metrics_inifin.h>
#include <plugins/metrics/aspect/metrics_manager.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class MetricsAspectIniFin <plugins/clips/aspect/clips_inifin.h>
 * MetricsAspect initializer/finalizer.
 * This initializer/finalizer will provide the Metrics node handle to
 * threads with the MetricsAspect.
 * @author Tim Niemueller
 */

/** Constructor. */
MetricsAspectIniFin::MetricsAspectIniFin()
  : AspectIniFin("MetricsAspect")
{
}

/** Destructor. */
MetricsAspectIniFin::~MetricsAspectIniFin()
{
}



void
MetricsAspectIniFin::init(Thread *thread)
{
  MetricsAspect *metrics_thread;
  metrics_thread = dynamic_cast<MetricsAspect *>(thread);
  if (metrics_thread == NULL) {
    throw CannotInitializeThreadException("Thread '%s' claims to have the "
					  "MetricsAspect, but RTTI says it "
					  "has not. ", thread->name());
  }
  
  metrics_mgr_->add_supplier(metrics_thread->get_metrics_supplier());
}

void
MetricsAspectIniFin::finalize(Thread *thread)
{
  MetricsAspect *metrics_thread;
  metrics_thread = dynamic_cast<MetricsAspect *>(thread);
  if (metrics_thread == NULL) {
    throw CannotFinalizeThreadException("Thread '%s' claims to have the "
					"MetricsAspect, but RTTI says it "
					"has not. ", thread->name());
  }

  metrics_mgr_->remove_supplier(metrics_thread->get_metrics_supplier());
}



/** Set Metrics environment manger.
 * @param metrics_mgr metrics manager
 */
void
MetricsAspectIniFin::set_manager(MetricsManager *metrics_mgr)
{
  metrics_mgr_ = metrics_mgr;
}

} // end namespace fawkes
