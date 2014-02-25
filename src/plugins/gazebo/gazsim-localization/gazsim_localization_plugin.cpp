/***************************************************************************
 *  gazsim_localization_plugin.cpp - Plugin provides 
 *     the simulated position of a robot in Gazeo
 *
 *  Created: Thu Aug 08 17:37:38 2013
 *  Copyright  2013 Frederik Zwilling
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

#include "gazsim_localization_thread.h"

using namespace fawkes;

/** Plugin provides the simulated position in Gazebo
 *
 *
 * @author Frederik Zwilling
 */
class GazsimLocalizationPlugin : public fawkes::Plugin
{
 public:

  /** Constructor.
   * @param config Fawkes configuration
   */
  GazsimLocalizationPlugin(Configuration *config)
    : Plugin(config)
  {
    thread_list.push_back(new LocalizationSimThread());
  }
};

PLUGIN_DESCRIPTION("Simulation of the Localization in Gazebo")
EXPORT_PLUGIN(GazsimLocalizationPlugin)
