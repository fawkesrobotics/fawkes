
/***************************************************************************
 *  gossip_plugin.cpp - Robot Group Communication Plugin
 *
 *  Created: Fri Feb 28 11:08:28 2014
 *  Copyright  2006-2014  Tim Niemueller [www.niemueller.de]
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

#include "gossip_thread.h"
#include <core/plugin.h>

using namespace fawkes;

/** Robot Group Communication Plugin.
 * @author Tim Niemueller
 */
class GossipPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  GossipPlugin(Configuration *config) : Plugin(config)
  {
    thread_list.push_back(new GossipThread());
  }
};


PLUGIN_DESCRIPTION("Robot Group Communication")
EXPORT_PLUGIN(GossipPlugin)
