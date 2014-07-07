
/***************************************************************************
 *  gex_sender_plugin.cpp - Gossip Example - Sender
 *
 *  Created: Wed Mar 05 14:26:12 2014
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

#include "gex_sender_thread.h"
#include <core/plugin.h>

using namespace fawkes;

/** Robot Group Communication Example Plugin - Sender.
 * @author Tim Niemueller
 */
class GossipExampleSenderPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  GossipExampleSenderPlugin(Configuration *config) : Plugin(config)
  {
    thread_list.push_back(new GossipExampleSenderThread());
  }
};


PLUGIN_DESCRIPTION("Gossip Example Plugin - Sender")
EXPORT_PLUGIN(GossipExampleSenderPlugin)
