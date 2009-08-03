
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

using namespace fawkes;

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

  std::string prefix = "/firevision/retriever/camera/";
  Configuration::ValueIterator *vi = config->search(prefix.c_str());

  while (vi->next()) {
    if ( ! vi->is_string() ) {
      throw TypeMismatchException("Only values of type string are valid for camera"
				  " argument strings, but got %s for %s",
				  vi->type(), vi->path());
    }

    std::string id = std::string(vi->path()).substr(prefix.length());

    thread_list.push_back(new FvRetrieverThread(vi->get_string().c_str(), id.c_str()));
  }

  if ( thread_list.empty() ) {
    throw Exception("No cameras have been set for fvretriever");
  }

  delete vi;
}

PLUGIN_DESCRIPTION("Reads images from cameras and stores them in shared memory")
EXPORT_PLUGIN(FvRetrieverPlugin)
