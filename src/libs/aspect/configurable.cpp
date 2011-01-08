
/***************************************************************************
 *  configurable.cpp - Configurable aspect for Fawkes
 *
 *  Created: Fri Jan 12 14:34:12 2007
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

#include <aspect/configurable.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class ConfigurableAspect <aspect/configurable.h>
 * Thread aspect to access configuration data.
 * Give this aspect to your thread to gain access to the configuration.
 * This aspects defines a thread as being configurable.
 * It is guaranteed that if used properly from within plugins that
 * setConfiguration() is called before the thread is started and that
 * you can access the configuration via the config member.
 *
 * It is higly recommended to also implement ConfigurationChangeHandler
 * to get notified about configuration changes as soon as they happen.
 * All threads which are configurable shall react immediately to config
 * changes and use the new configuration. Therefore all inner structures
 * shall be updated as necessary and member reinitialized if needed.
 * Only this way no restart or at least plugin load/unload cycle is needed
 * anymore to change the configuration. This should tremendously help
 * to decrease debugging, testing and parameter tuning time!
 *
 * @ingroup Aspects
 * @author Tim Niemueller
 */


/** @var Configuration *  ConfigurableAspect::config
 * This is the Configuration member used to access the configuration.
 * The configuration will remain valid for the whole lifetime of the
 * thread.
 */

/** Constructor. */
ConfigurableAspect::ConfigurableAspect()
{
  add_aspect("ConfigurableAspect");
}

/** Virtual empty Destructor. */
ConfigurableAspect::~ConfigurableAspect()
{
}


/** Set the configuration
 * It is guaranteed that this is called for a configurable thread before
 * Thread::start() is called (when running regularly inside Fawkes).
 * @param config Configuration instance to use.
 */
void
ConfigurableAspect::init_ConfigurableAspect(Configuration *config)
{
  this->config = config;
}

} // end namespace fawkes
