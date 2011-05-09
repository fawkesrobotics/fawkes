
/***************************************************************************
 *  webview_plugin.cpp - Provide webview features to ROS
 *
 *  Created: Fri May 06 10:44:47 2011
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#include "webview_plugin.h"
#include "webview_thread.h"

using namespace fawkes;

/** @class ROSWebviewPlugin "webview_plugin.h"
 * ROS webview plugin.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param config Fawkes configuration
 */
ROSWebviewPlugin::ROSWebviewPlugin(Configuration *config)
  : Plugin(config)
{
  thread_list.push_back(new ROSWebviewThread());
}


PLUGIN_DESCRIPTION("Provide webview to ROS nodes")
EXPORT_PLUGIN(ROSWebviewPlugin)
