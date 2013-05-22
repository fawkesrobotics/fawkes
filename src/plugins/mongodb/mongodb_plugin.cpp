
/***************************************************************************
 *  mongodb_plugin.cpp - Fawkes MongoDB Plugin
 *
 *  Created: Sun Dec 05 23:22:23 2010 (Steelers vs. Baltimore, Touchdown!)
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

#include <core/plugin.h>

#include "mongodb_thread.h"

using namespace fawkes;

/** MongoDB Connector Plugin.
 * This plugin provides access to MongoDB for other Fawkes plugins. If
 * enabled it also writes log messages to the database.
 *
 * @author Tim Niemueller
 */
class MongoDBPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  MongoDBPlugin(Configuration *config) : Plugin(config)
  {
    thread_list.push_back(new MongoDBThread());
  }
};

PLUGIN_DESCRIPTION("MongoDB Connector Plugin")
EXPORT_PLUGIN(MongoDBPlugin)
