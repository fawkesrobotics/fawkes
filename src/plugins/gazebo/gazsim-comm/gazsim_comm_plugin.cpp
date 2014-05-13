/***************************************************************************
 *  gazsim_comm_plugin.cpp - Plugin simulates peer-to-peer communication over
 *                    an network with configurable instability and manages
 *                    the frowarding of messages to different ports on
 *                    the same machine.
 *
 *  Created: Thu Sep 12 11:06:41 2013
 *  Copyright  2013  Frederik Zwilling
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

#include "gazsim_comm_thread.h"
#include <core/plugin.h>

using namespace fawkes;

/** Plugin simulates communication
 * @author Frederik Zwilling
 */
class GazsimCommPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  GazsimCommPlugin(Configuration *config) : Plugin(config)
  {
    thread_list.push_back(new GazsimCommThread());
  }
};


PLUGIN_DESCRIPTION("Simulates and manages communication for testing with Gazebo")
EXPORT_PLUGIN(GazsimCommPlugin)
