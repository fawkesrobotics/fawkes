
/***************************************************************************
 *  naoqi_plugin.cpp - Plugin to access NaoQi features
 *
 *  Created: Thu May 12 19:00:45 2011
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

#include "broker_thread.h"
#include "dcm_thread.h"

using namespace fawkes;

/** Plugin to access NaoQi from Fawkes.
 * This plugin integrates NaoQi and provides access to the NaoQi broker
 * to other plugins.
 * @author Tim Niemueller
 */
class NaoQiPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  NaoQiPlugin(Configuration *config)
    : Plugin(config)
  {
    thread_list.push_back(new NaoQiBrokerThread());
    thread_list.push_back(new NaoQiDCMThread());
  }
};

PLUGIN_DESCRIPTION("NaoQi integration base plugin")
EXPORT_PLUGIN(NaoQiPlugin)
