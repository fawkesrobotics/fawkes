
/***************************************************************************
 *  llsfrbcomm_plugin.cpp - Plugin is a adapter between 
 *       Protobuf Refbox communication and 
 *       Protobuf communication over the gazebo node
 *
 *  Created: Wed Aug 21 15:18:27 2013
 *  Copyright  2013  Frederik Zwilling
 *                   Tim Niemueller [www.niemueller.de]
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

#include "gazsim_llsfrbcomm_thread.h"
#include <core/plugin.h>

using namespace fawkes;

/** Plugin is a adapter between 
 *       Protobuf Refbox communication and 
 *       Protobuf communication over the gazebo node
 * @author Frederik Zwilling, Tim Niemueller
 */
class GazsimLLSFRbCommPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  GazsimLLSFRbCommPlugin(Configuration *config) : Plugin(config)
  {
    thread_list.push_back(new GazsimLLSFRbCommThread());
  }
};


PLUGIN_DESCRIPTION("Adapter PB Refbox Comm - PB Gazebo Comm")
EXPORT_PLUGIN(GazsimLLSFRbCommPlugin)
