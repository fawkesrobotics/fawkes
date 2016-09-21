
/***************************************************************************
 *  laser-cluster-plugin.cpp - Detect a cluster from 2D laser data
 *
 *  Created: Sun Apr 21 01:15:54 2013
 *  Copyright  2011-2013  Tim Niemueller [www.niemueller.de]
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

#include "laser-cluster-thread.h"

using namespace fawkes;

/** Plugin to detect a cluster in 2D laser data.
 * @author Tim Niemueller
 */
class LaserClusterPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  LaserClusterPlugin(Configuration *config)
    : Plugin(config)
  {

    std::set<std::string> configs;
    std::set<std::string> ignored_configs;

    std::string prefix = "/laser-cluster/";

    // Read configurations and spawn LaserFilterThreads
#if __cplusplus >= 201103L
    std::unique_ptr<Configuration::ValueIterator> i(config->search(prefix.c_str()));
#else
    std::auto_ptr<Configuration::ValueIterator> i(config->search(prefix.c_str()));
#endif
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
	    LaserClusterThread *thread = new LaserClusterThread(cfg_name, cfg_prefix);
	    thread_list.push_back(thread);
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
};

PLUGIN_DESCRIPTION("Detect cluster in 2D laser data")
EXPORT_PLUGIN(LaserClusterPlugin)
