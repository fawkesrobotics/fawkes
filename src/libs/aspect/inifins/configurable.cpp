
/***************************************************************************
 *  configurable.cpp - Fawkes Configurable Aspect initializer/finalizer
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

#include <aspect/inifins/configurable.h>
#include <aspect/configurable.h>
#include <config/config.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class ConfigurableAspectIniFin <aspect/inifins/configurable.h>
 * Initializer/finalizer for the ConfigurableAspect.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param config configuration instance to pass to threads
 */
ConfigurableAspectIniFin::ConfigurableAspectIniFin(Configuration *config)
  : AspectIniFin("ConfigurableAspect")
{
  __config = config;
}


void
ConfigurableAspectIniFin::init(Thread *thread)
{
  ConfigurableAspect *configurable_thread;
  configurable_thread = dynamic_cast<ConfigurableAspect *>(thread);
  if (configurable_thread == NULL) {
    throw CannotInitializeThreadException("Thread '%s' claims to have the "
					  "ConfigurableAspect, but RTTI says it "
					  "has not. ", thread->name());
  }

  configurable_thread->init_ConfigurableAspect(__config);
}


void
ConfigurableAspectIniFin::finalize(Thread *thread)
{
}


} // end namespace fawkes
