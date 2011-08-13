
/***************************************************************************
 *  led_plugin.cpp - Plugin to provide NaoQi LEDs to Fawkes
 *
 *  Created: Thu Aug 11 11:37:07 2011
 *  Copyright  2011  Tim Niemueller [www.niemueller.de]
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

#include "led_thread.h"

using namespace fawkes;

/** Plugin to provide NaoQi LEDs to Fawkes.
 * @author Tim Niemueller
 */
class NaoQiLedPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  NaoQiLedPlugin(Configuration *config)
    : Plugin(config)
  {
    thread_list.push_back(new NaoQiLedThread());
  }
};

PLUGIN_DESCRIPTION("NaoQi LED provider")
EXPORT_PLUGIN(NaoQiLedPlugin)
