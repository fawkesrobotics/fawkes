
/***************************************************************************
 *  webview-ptzcam-plugin.cpp - Pan/Tilt/Zoom Camera controller for webview
 *
 *  Created: Fri Feb 07 16:00:18 2014
 *  Copyright  2006-2014  Tim Niemueller [www.niemueller.de]
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

#include "webview-ptzcam-thread.h"
#include <core/plugin.h>

using namespace fawkes;

/** CLIPS webview plugin.
 * @author Tim Niemueller
 */
class WebviewPtzCamPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  WebviewPtzCamPlugin(Configuration *config) : Plugin(config)
  {
    thread_list.push_back(new WebviewPtzCamThread());
  }
};


PLUGIN_DESCRIPTION("Pan/tilt/zoom camera control via webview")
EXPORT_PLUGIN(WebviewPtzCamPlugin)
