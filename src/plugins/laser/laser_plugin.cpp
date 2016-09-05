
/***************************************************************************
 *  laser_plugin.cpp - Fawkes Laser Plugin
 *
 *  Created: Tue Aug 05 13:11:02 2008
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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

#include <plugins/laser/laser_plugin.h>

#include "sensor_thread.h"
#ifdef HAVE_LIBPCAN
#  include "lase_edl_aqt.h"
#endif
#ifdef HAVE_URG
#  include "urg_aqt.h"
#endif
#ifdef HAVE_URG_GBX
#  include "urg_gbx_aqt.h"
#endif
#ifdef HAVE_LIBUSB
#  include "sick_tim55x_usb_aqt.h"
#endif
#ifdef HAVE_SICK55X_BOOST
#  include "sick_tim55x_ethernet_aqt.h"
#endif

#include <set>
#include <memory>

using namespace fawkes;

/** @class LaserPlugin "laser_plugin.h"
 * Laser plugin for Fawkes.
 * This plugin integrates Fawkes with Laser, for example for accessing
 * a simulator.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param config Fawkes configuration
 */
LaserPlugin::LaserPlugin(Configuration *config)
  : Plugin(config)
{
  std::set<std::string> configs;
  std::set<std::string> ignored_configs;

  std::string prefix = "/hardware/laser/";

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
	  std::string type = config->get_string((cfg_prefix + "type").c_str());

	  //printf("Adding laser acquisition thread for %s\n", cfg_name.c_str());
	  LaserAcquisitionThread *aqt = NULL;
#ifdef HAVE_URG
	  if ( type == "urg" ) {
	    aqt = new HokuyoUrgAcquisitionThread(cfg_name, cfg_prefix);
	  } else
#endif

#ifdef HAVE_LIBPCAN
	  if ( type == "lase_edl" ) {
	    aqt = new LaseEdlAcquisitionThread(cfg_name, cfg_prefix);
	  } else
#endif

#ifdef HAVE_URG_GBX
	  if ( type == "urg_gbx" ) {
	    aqt = new HokuyoUrgGbxAcquisitionThread(cfg_name, cfg_prefix);
	  } else 
#endif

#ifdef HAVE_LIBUSB
	  if ( type == "TiM55x-USB" ) {
	    aqt = new SickTiM55xUSBAcquisitionThread(cfg_name, cfg_prefix);
	  } else 
#endif

#ifdef HAVE_SICK55X_BOOST
	  if ( type == "TiM55x-Ethernet" ) {
	    aqt = new SickTiM55xEthernetAcquisitionThread(cfg_name, cfg_prefix);
	  } else 
#endif

	  {
	    throw Exception("Unknown lasertype '%s' for config %s",
			    type.c_str(), cfg_name.c_str());
	  }

	  thread_list.push_back(aqt);
	  thread_list.push_back(new LaserSensorThread(cfg_name, cfg_prefix, aqt));

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
    throw Exception("No laser devices configured, aborting");
  } else {
  }
}


PLUGIN_DESCRIPTION("Hardware driver for laser range finders")
EXPORT_PLUGIN(LaserPlugin)
