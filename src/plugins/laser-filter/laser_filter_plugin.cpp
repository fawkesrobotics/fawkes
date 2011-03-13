
/***************************************************************************
 *  laser_filter_plugin.cpp - Fawkes Laser Filter Plugin
 *
 *  Created: Sun Mar 13 01:06:51 2011
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

#include "laser_filter_plugin.h"

#include "filter_thread.h"

#include <set>
#include <memory>

using namespace fawkes;

/** @class LaserFilterPlugin "laser_filter_plugin.h"
 * Laser filter plugin for Fawkes.
 * This plugin filters laser data. It reads laser data from one or more
 * interfaces, filters it, and writes to an output interface. It supports
 * a virtually arbitrary number of active filters.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param config Fawkes configuration
 */
LaserFilterPlugin::LaserFilterPlugin(Configuration *config)
  : Plugin(config)
{
  std::set<std::string> configs;
  std::set<std::string> ignored_configs;

  std::string prefix = "/plugins/laser-filter/";

  std::auto_ptr<Configuration::ValueIterator> i(config->search(prefix.c_str()));
  while (i->next()) {
    std::string cfg_name = std::string(i->path()).substr(prefix.length());
    cfg_name = cfg_name.substr(0, cfg_name.find("/"));

    if ( (configs.find(cfg_name) == configs.end()) &&
	 (ignored_configs.find(cfg_name) == ignored_configs.end()) ) {

      std::string cfg_prefix = prefix + cfg_name + "/";

      bool active = true;
      try {
	active = config->get_bool((cfg_prefix + "active").c_str());
      } catch (Exception &e) {} // ignored, assume enabled

      try {
	if (active) {
	  thread_list.push_back(new LaserFilterThread(cfg_name, cfg_prefix));
	  configs.insert(cfg_name);
	} else {
	  //printf("Ignoring laser config %s\n", cfg_name.c_str());
	  ignored_configs.insert(cfg_name);
	}
      } catch(Exception &e) {
	for (ThreadList::iterator i = thread_list.begin();
	     i != thread_list.end(); ++i) {
	  delete *i;
	}
	throw;
      }
    }
  }

  if ( thread_list.empty() ) {
    throw Exception("No active laser filters configured, aborting");
  }
}


PLUGIN_DESCRIPTION("Filter laser data in blackboard")
EXPORT_PLUGIN(LaserFilterPlugin)
