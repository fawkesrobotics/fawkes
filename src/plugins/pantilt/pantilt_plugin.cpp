
/***************************************************************************
 *  pantilt_plugin.cpp - Plugin to drive pan/tilt units (e.g. for cameras)
 *
 *  Created: Wed Jun 17 19:27:08 2009
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

#include "pantilt_plugin.h"
#include "robotis/rx28_thread.h"
#include "sony/evid100p_thread.h"
#include "dirperc/dp_thread.h"
#include "sensor_thread.h"

#include <set>

using namespace fawkes;

/** @class PanTiltPlugin "pantilt_plugin.h"
 * Plugin to drive pan/tilt units with Fawkes.
 * This plugin integrates a number of known pan/tilt units.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param config Fawkes configuration
 */
PanTiltPlugin::PanTiltPlugin(Configuration *config)
  : Plugin(config)
{
  std::set<std::string> ptus;
  std::set<std::string> ignored_ptus;

  std::string prefix = "/hardware/pantilt/";
  std::string ptus_prefix = prefix + "ptus/";

  PanTiltSensorThread *sensor_thread = new PanTiltSensorThread();

  Configuration::ValueIterator *i = config->search(ptus_prefix.c_str());
  while (i->next()) {
    std::string ptu = std::string(i->path()).substr(ptus_prefix.length());
    ptu = ptu.substr(0, ptu.find("/"));

    if ( (ptus.find(ptu) == ptus.end()) &&
	 (ignored_ptus.find(ptu) == ignored_ptus.end()) ) {

      std::string ptu_prefix = ptus_prefix + ptu + "/";

      bool active = true;
      try {
	active = config->get_bool((ptu_prefix + "active").c_str());
      } catch (Exception &e) {} // ignored, assume enabled

      if (active) {
	//printf("Adding sync thread for peer %s\n", peer.c_str());
	std::string type = config->get_string((ptu_prefix + "type").c_str());
	PanTiltActThread *act_thread;

	if (type == "RX28") {
	  act_thread = new PanTiltRX28Thread(prefix, ptu_prefix, ptu);
	} else if (type == "EviD100P") {
	  act_thread = new PanTiltSonyEviD100PThread(prefix, ptu_prefix, ptu);
	} else if (type == "DirPercASCII") {
	  act_thread = new PanTiltDirectedPerceptionThread(prefix, ptu_prefix, ptu);
	} else {
	  throw Exception("Unknown PTU type %s", type.c_str());
	}

	ptus.insert(ptu);
	thread_list.push_back(act_thread);
	sensor_thread->add_act_thread(act_thread);
      } else {
	//printf("Ignoring PTU %s\n", ptu.c_str());
	ignored_ptus.insert(ptu);
      }
    }
  }
  delete i;

  if ( thread_list.empty() ) {
    throw Exception("No synchronization peers configured, aborting");
  } else {
  }
  thread_list.push_back(sensor_thread);
}


PLUGIN_DESCRIPTION("Use pan/tilt units with Fawkes.")
EXPORT_PLUGIN(PanTiltPlugin)
