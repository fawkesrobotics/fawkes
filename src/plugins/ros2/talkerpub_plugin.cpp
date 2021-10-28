
/***************************************************************************
 *  talkerpub_plugin.cpp - Example ROSAspect plugin to publish talker msgs
 *
 *  Created: Thu May 05 18:47:32 2011
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

#include "talkerpub_plugin.h"

#include "talkerpub_thread.h"

using namespace fawkes;

/** @class ROS2TalkerPubPlugin "talkerpub_plugin.h"
 * Talker publisher example plugin.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param config Fawkes configuration
 */
ROS2TalkerPubPlugin::ROS2TalkerPubPlugin(Configuration *config) : Plugin(config)
{
	thread_list.push_back(new ROS2TalkerPubThread());
}

PLUGIN_DESCRIPTION("Publish talker messages via ROS")
EXPORT_PLUGIN(ROS2TalkerPubPlugin)
