
/***************************************************************************
 *  motion_plugin.cpp - Plugin to provide NaoQi motion to Fawkes
 *
 *  Created: Thu Jun 09 13:06:19 2011
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
 *
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

#include <core/plugin.h>

#include "motion_thread.h"

using namespace fawkes;

/** Plugin to provide NaoQi motion to Fawkes.
 * This plugin uses ALMotion to provide motion patterns to Fawkes.
 * @author Tim Niemueller
 */
class NaoQiMotionPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  NaoQiMotionPlugin(Configuration *config)
    : Plugin(config)
  {
    thread_list.push_back(new NaoQiMotionThread());
  }
};

PLUGIN_DESCRIPTION("NaoQi motion provider for Fawkes")
EXPORT_PLUGIN(NaoQiMotionPlugin)
