
/***************************************************************************
 *  navgraph_stconstr_plugin.cpp - Static constraints for navgraph
 *
 *  Created: Fri Jul 11 17:30:28 2014
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

#include "navgraph_stconstr_thread.h"

using namespace fawkes;

/** Static constraints for navgraph.
 * @author Tim Niemueller
 */
class NavGraphStaticConstraintsPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  NavGraphStaticConstraintsPlugin(Configuration *config)
    : Plugin(config)
  {
    thread_list.push_back(new NavGraphStaticConstraintsThread());
  }
};

PLUGIN_DESCRIPTION("Static blocking of navgraph nodes")
EXPORT_PLUGIN(NavGraphStaticConstraintsPlugin)
