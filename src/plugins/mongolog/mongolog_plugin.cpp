
/***************************************************************************
 *  mongolog_plugin.cpp - Fawkes MongoDB Logging Plugin
 *
 *  Created: Wed Dec 08 23:04:33 2010
 *  Copyright  2006-2010  Tim Niemueller [www.niemueller.de]
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

#include "mongolog_plugin.h"
#include "mongolog_thread.h"
#include "mongolog_logger_thread.h"

using namespace fawkes;

/** @class MongoLogPlugin "mongolog_plugin.h"
 * MongoDB Logging Plugin.
 * This plugin provides logging of BlackBoard data to MongoDB.
 *
 * @author Tim Niemueller
 */

/** Constructor.
 * @param config Fawkes configuration
 */
MongoLogPlugin::MongoLogPlugin(Configuration *config)
  : Plugin(config)
{
  thread_list.push_back(new MongoLogThread());

  bool enable_logger = true;
  try {
    enable_logger = config->get_bool("/plugins/mongolog/enable_logger");
  } catch (Exception &e) {}
  if (enable_logger) thread_list.push_back(new MongoLogLoggerThread());
}


PLUGIN_DESCRIPTION("Logging of BlackBoard data to MongoDB")
EXPORT_PLUGIN(MongoLogPlugin)
