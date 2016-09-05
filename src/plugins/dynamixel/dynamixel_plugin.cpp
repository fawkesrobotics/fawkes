
/***************************************************************************
 *  dynamixel_plugin.cpp - Robotis Servo driver plugin
 *
 *  Created: Mon Mar 23 20:12:10 2015
 *  Copyright  2011-2015  Tim Niemueller [www.niemueller.de]
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

#include "driver_thread.h"
#include "sensor_thread.h"
#include "act_thread.h"

#include <set>

using namespace fawkes;

/** Driver plugin for Robotis dynamixel servos.
 * @author Tim Niemueller
 */
class DynamixelPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  DynamixelPlugin(Configuration *config)
    : Plugin(config)
  {
    DynamixelSensorThread *sensor_thread = new DynamixelSensorThread();
    DynamixelActThread *act_thread = new DynamixelActThread();


    std::set<std::string> configs;
    std::set<std::string> ignored_configs;

    std::string prefix = "/dynamixel/";

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
	    DynamixelDriverThread *drv_thread = new DynamixelDriverThread(cfg_name, cfg_prefix);
	    act_thread->add_driver_thread(drv_thread);
	    sensor_thread->add_driver_thread(drv_thread);
	    thread_list.push_back(drv_thread);
	    configs.insert(cfg_name);
	  } else {
	    //printf("Ignoring dynamixel config %s\n", cfg_name.c_str());
	    ignored_configs.insert(cfg_name);
	  }
	} catch(Exception &e) {
	  for (ThreadList::iterator i = thread_list.begin();
	       i != thread_list.end(); ++i) {
	    delete *i;
	  }
	  delete act_thread;
	  delete sensor_thread;
	  throw;
	}
      }
    }

    if ( thread_list.empty() ) {
      delete act_thread;
      delete sensor_thread;
      throw Exception("No active servo configs, aborting");
    }

    thread_list.push_back(sensor_thread);
    thread_list.push_back(act_thread);
  }
};

PLUGIN_DESCRIPTION("Robotis Dynamixel servo driver plugin")
EXPORT_PLUGIN(DynamixelPlugin)
