
/***************************************************************************
 *  plugin_director.cpp - Fawkes PluginDirector Aspect initializer/finalizer
 *
 *  Created: Tue Nov 23 23:06:13 2010
 *  Copyright  2006-2010  Tim Niemueller [www.niemueller.de]
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

#include <aspect/inifins/plugin_director.h>
#include <aspect/plugin_director.h>
#include <aspect/blocked_timing.h>
#include <plugin/manager.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class PluginDirectorAspectIniFin <aspect/inifins/plugin_director.h>
 * Initializer/finalizer for the PluginDirectorAspect.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param manager plugin manager instance to pass to threads
 */
PluginDirectorAspectIniFin::PluginDirectorAspectIniFin(PluginManager *manager)
  : AspectIniFin("PluginDirectorAspect")
{
  __manager = manager;
}


void
PluginDirectorAspectIniFin::init(Thread *thread)
{
  PluginDirectorAspect *plugin_director_thread;
  plugin_director_thread = dynamic_cast<PluginDirectorAspect *>(thread);
  if (plugin_director_thread == NULL) {
    throw CannotInitializeThreadException("Thread '%s' claims to have the "
					  "PluginDirectorAspect, but RTTI says it "
					  "has not. ", thread->name());
  }

  BlockedTimingAspect *blocked_timing_thread;
  blocked_timing_thread = dynamic_cast<BlockedTimingAspect *>(thread);
  if (blocked_timing_thread != NULL) {
    throw CannotInitializeThreadException("Thread '%s' cannot have BlockedTimingAspect "
					  "(conflicts with PluginDirectorAspect)",
					  thread->name());
  }

  plugin_director_thread->init_PluginDirectorAspect(__manager);
}


void
PluginDirectorAspectIniFin::finalize(Thread *thread)
{
}


} // end namespace fawkes
