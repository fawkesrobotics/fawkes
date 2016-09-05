
/***************************************************************************
 *  imu_plugin.cpp - Fawkes IMU Plugin
 *
 *  Created: Sun Jun 22 21:41:38 2014
 *  Copyright  2006-2014  Tim Niemueller [www.niemueller.de]
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

#include "sensor_thread.h"
#include "acquisition_thread.h"
#ifdef HAVE_CRUIZCORE
#  include "imu_cruizcore_xg1010.h"
#endif

#include <set>
#include <memory>

using namespace fawkes;

/** IMU driver plugin.
 * @author Tim Niemueller
 */
class IMUPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  IMUPlugin(Configuration *config)
    : Plugin(config)
  {
    std::set<std::string> configs;
    std::set<std::string> ignored_configs;

    std::string prefix = "/hardware/imu/";

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
	    bool continuous = false;
	    try {
	      continuous = config->get_bool((cfg_prefix + "continuous").c_str());
	    } catch (Exception &e) {} // ignored, use default

	    //printf("Adding IMU acquisition thread for %s\n", cfg_name.c_str());
	    IMUAcquisitionThread *aqt = NULL;
#ifdef HAVE_CRUIZCORE
	    if ( type == "CruizCore-XG1010" ) {
	      aqt = new CruizCoreXG1010AcquisitionThread(cfg_name, cfg_prefix, continuous);
	    } else
#endif

	    {
	      throw Exception("Unknown IMU type '%s' for config %s",
			      type.c_str(), cfg_name.c_str());
	    }

	    thread_list.push_back(aqt);
	    if (! continuous) {
	      thread_list.push_back(new IMUSensorThread(cfg_name, cfg_prefix, aqt));
	    }

	    configs.insert(cfg_name);
	  } else {
	    //printf("Ignoring IMU config %s\n", cfg_name.c_str());
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
      throw Exception("No IMU devices configured, aborting");
    }
  }
};

PLUGIN_DESCRIPTION("Driver for inertial measurement units (IMU)")
EXPORT_PLUGIN(IMUPlugin)
