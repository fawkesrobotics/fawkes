
/***************************************************************************
 *  rrdweb_plugin.c - Fawkes RRD Webview Plugin
 *
 *  Created: Tue Dec 21 01:02:46 2010
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

#include "rrdweb_plugin.h"
#include "rrdweb_thread.h"

using namespace fawkes;

/** @class RRDWebPlugin "rrdweb_plugin.h"
 * RRD Webview plugin.
 * This plugin provides access to graphs created from RRDs via the web.
 *
 * @author Tim Niemueller
 */

/** Constructor.
 * @param config Fawkes configuration
 */
RRDWebPlugin::RRDWebPlugin(Configuration *config)
  : Plugin(config)
{
  thread_list.push_back(new RRDWebThread());
}


PLUGIN_DESCRIPTION("RRD graph displaying via Webview")
EXPORT_PLUGIN(RRDWebPlugin)
