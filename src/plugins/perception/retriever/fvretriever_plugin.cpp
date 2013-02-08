
/***************************************************************************
 *  fvretriever_plugin.cpp - FireVision Retriever Plugin
 *
 *  Created: Tue Jun 26 17:35:33 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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

#include "fvretriever_plugin.h"
#include "retriever_thread.h"

#include <core/exceptions/software.h>

#include <set>

using namespace fawkes;
using namespace firevision;

/** @class FvRetrieverPlugin "fvretriever_plugin.h"
 * FireVision Retriever Plugin.
 * This is the FireVision retriever plugin. It is a simple plugin that will
 * fetch images from a specific camera defined as a configuration setting.
 *
 * @author Tim Niemueller
 */

/** Constructor.
 * @param config Fawkes configuration
 */
FvRetrieverPlugin::FvRetrieverPlugin(Configuration *config)
  : Plugin(config)
{

  std::set<std::string> configs;
  std::set<std::string> ignored_configs;

  std::string prefix = "/firevision/retriever/camera/";

  Configuration::ValueIterator *vi = config->search(prefix.c_str());
  while (vi->next()) {

    std::string cfg_name = std::string(vi->path()).substr(prefix.length());
    cfg_name = cfg_name.substr(0, cfg_name.find("/"));

    if ( (configs.find(cfg_name) == configs.end()) &&
	 (ignored_configs.find(cfg_name) == ignored_configs.end()) )
    {
      std::string cfg_prefix = prefix + cfg_name + "/";

      bool active = true;
      try {
	active = config->get_bool((cfg_prefix + "active").c_str());
      } catch (Exception &e) {} // ignored, assume enabled
      std::string cam_string = config->get_string((cfg_prefix + "string").c_str());

      if (active) {
        thread_list.push_back(new FvRetrieverThread(cam_string,
                                                    cfg_name, cfg_prefix));
        configs.insert(cfg_name);
      } else {
        //printf("Ignoring retriever config %s\n", cfg_name.c_str());
        ignored_configs.insert(cfg_name);
      }
    }
  }

  delete vi;

  if ( thread_list.empty() ) {
    throw Exception("No cameras have been set for fvretriever");
  }

}

PLUGIN_DESCRIPTION("Reads images from cameras and stores them in shared memory")
EXPORT_PLUGIN(FvRetrieverPlugin)
