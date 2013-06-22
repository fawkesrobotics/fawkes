
/***************************************************************************
 *  clips-webview-plugin.cpp - CLIPS introspection via webview
 *
 *  Created: Sat Jun 15 19:53:25 2013
 *  Copyright  2006-2013  Tim Niemueller [www.niemueller.de]
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

#include "clips-webview-thread.h"
#include <core/plugin.h>

using namespace fawkes;

/** CLIPS webview plugin.
 * @author Tim Niemueller
 */
class ClipsWebviewPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  ClipsWebviewPlugin(Configuration *config) : Plugin(config)
  {
    std::string cfg_clips_env = config->get_string("/clips-webview/env-name");
    thread_list.push_back(new ClipsWebviewThread(cfg_clips_env));
  }
};


PLUGIN_DESCRIPTION("CLIPS introspection via webview")
EXPORT_PLUGIN(ClipsWebviewPlugin)
