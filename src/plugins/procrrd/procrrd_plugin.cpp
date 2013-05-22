
/***************************************************************************
 *  systemrrd_plugin.cpp - Fawkes Proc RRD Plugin
 *
 *  Created: Mon Dec 17 12:54:00 2012
 *  Copyright  2012  Bastian Klingen
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

#include "procrrd_thread.h"
#include <core/plugin.h>

using namespace fawkes;

/** Proc RRD Plugin.
 * This plugin records system performance via the /proc data using RRDs.
 * @author Bastian Klingen
 */
class ProcRRDPlugin : public fawkes::Plugin
{
 public:
/** Constructor.
 * @param config Fawkes configuration
 */
  ProcRRDPlugin(Configuration *config) : Plugin(config)
  {
    thread_list.push_back(new ProcRRDThread());
  }
};

PLUGIN_DESCRIPTION("Log /proc data using RRD")
EXPORT_PLUGIN(ProcRRDPlugin)
