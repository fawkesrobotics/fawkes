
/***************************************************************************
 *  mongorrd_plugin.cpp - Fawkes MongoDB RRD Plugin
 *
 *  Created: Wed Dec 08 23:04:33 2010
 *  Copyright  2006-2012  Tim Niemueller [www.niemueller.de]
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

#include "mongorrd_thread.h"
#include <core/plugin.h>

using namespace fawkes;

/** MongoDB RRD Plugin.
 * This plugin records MongoDB performance data using RRDs.
 * @author Tim Niemueller
 */
class MongoRRDPlugin : public fawkes::Plugin
{
 public:
/** Constructor.
 * @param config Fawkes configuration
 */
  MongoRRDPlugin(Configuration *config) : Plugin(config)
  {
    thread_list.push_back(new MongoRRDThread());
  }
};

PLUGIN_DESCRIPTION("Log MongoDB performance data using RRD")
EXPORT_PLUGIN(MongoRRDPlugin)
