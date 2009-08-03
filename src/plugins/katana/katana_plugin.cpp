
/***************************************************************************
 *  katana_plugin.cpp - Plugin to drive Neuronics' Katana arm
 *
 *  Created: Mon Jun 08 17:57:16 2009
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#include "katana_plugin.h"
#include "act_thread.h"
#include "sensor_thread.h"

using namespace fawkes;

/** @class KatanaPlugin "katana_plugin.h"
 * Plugin to drive Neuronics' Katana arm with Fawkes.
 * This plugin integrates Neuronics' KNI library to control a Katana arm.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param config Fawkes configuration
 */
KatanaPlugin::KatanaPlugin(Configuration *config)
  : Plugin(config)
{
  KatanaActThread *act_thread = new KatanaActThread();
  thread_list.push_back(act_thread);
  thread_list.push_back(new KatanaSensorThread(act_thread));
}


PLUGIN_DESCRIPTION("Run Neuronics' Katana arm with Fawkes.")
EXPORT_PLUGIN(KatanaPlugin)
