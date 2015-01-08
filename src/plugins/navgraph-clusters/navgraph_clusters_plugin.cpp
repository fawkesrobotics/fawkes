
/***************************************************************************
 *  navgraph_clusters_plugin.cpp - block paths based on laser clusters
 *
 *  Created: Sun Jul 13 15:26:01 2014
 *  Copyright  2014  Tim Niemueller [www.niemueller.de]
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

#include "navgraph_clusters_thread.h"

using namespace fawkes;

/** Block navgraph paths based on laser clusters.
 * @author Tim Niemueller
 */
class NavGraphClustersPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  NavGraphClustersPlugin(Configuration *config)
    : Plugin(config)
  {
    thread_list.push_back(new NavGraphClustersThread());
  }
};

PLUGIN_DESCRIPTION("Block paths based on laser clusters")
EXPORT_PLUGIN(NavGraphClustersPlugin)
