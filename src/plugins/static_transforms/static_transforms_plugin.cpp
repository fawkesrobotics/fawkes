
/***************************************************************************
 *  static_transforms_plugin.cpp - Plugin to publish static transforms
 *
 *  Created: Tue Oct 25 16:30:56 2011
 *  Copyright  2011  Tim Niemueller [www.niemueller.de]
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

#include "static_transforms_thread.h"

using namespace fawkes;

/** Plugin to publish static transforms.
 * @author Tim Niemueller
 */
class StaticTransformsPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  StaticTransformsPlugin(Configuration *config)
    : Plugin(config)
  {
    thread_list.push_back(new StaticTransformsThread());
  }
};

PLUGIN_DESCRIPTION("Static transform publisher")
EXPORT_PLUGIN(StaticTransformsPlugin)
