
/***************************************************************************
 *  clips_tf_plugin.cpp - Transforms CLIPS Feature Plugin
 *
 *  Created: Sat Apr 11 17:26:46 2015
 *  Copyright  2015  Tim Niemueller [www.niemueller.de]
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

#include "clips_tf_thread.h"
#include <core/plugin.h>

using namespace fawkes;

/** CLIPS navgraph plugin.
 * @author Tim Niemueller
 */
class ClipsTFPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  ClipsTFPlugin(Configuration *config) : Plugin(config)
  {
    thread_list.push_back(new ClipsTFThread());
  }
};


PLUGIN_DESCRIPTION("CLIPS feature to access transforms")
EXPORT_PLUGIN(ClipsTFPlugin)
