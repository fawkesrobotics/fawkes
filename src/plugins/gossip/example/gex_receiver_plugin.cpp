
/***************************************************************************
 *  gex_receiver_plugin.cpp - Gossip Example - Receiver
 *
 *  Created: Thu Mar 06 10:39:41 2014
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

#include "gex_receiver_thread.h"
#include <core/plugin.h>

using namespace fawkes;

/** Robot Group Communication Example Plugin - Receiver.
 * @author Tim Niemueller
 */
class GossipExampleReceiverPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  GossipExampleReceiverPlugin(Configuration *config) : Plugin(config)
  {
    thread_list.push_back(new GossipExampleReceiverThread());
  }
};


PLUGIN_DESCRIPTION("Gossip Example Plugin - Receiver")
EXPORT_PLUGIN(GossipExampleReceiverPlugin)
