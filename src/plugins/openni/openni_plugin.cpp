
/***************************************************************************
 *  openni_plugin.cpp - Plugin to access OpenNI features
 *
 *  Created: Thu Feb 17 10:23:03 2011
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

#include "context_thread.h"
#include <core/plugin.h>

using namespace fawkes;

/** Plugin to access OpenNI from Fawkes.
 * This plugin integrates OpenNI and provides access to the OpenNI context
 * to other plugins.
 * @author Tim Niemueller
 */
class OpenNiPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  OpenNiPlugin(Configuration *config) : Plugin(config)
  {
    thread_list.push_back(new OpenNiContextThread());
  }
};

PLUGIN_DESCRIPTION("OpenNI integration base plugin")
EXPORT_PLUGIN(OpenNiPlugin)
