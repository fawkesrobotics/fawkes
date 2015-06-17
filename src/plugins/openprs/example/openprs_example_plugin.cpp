
/***************************************************************************
 *  openprs_example_plugin.cpp - Example plugin to use OpenPRS
 *
 *  Created: Tue Aug 19 11:49:39 2014
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

#include "openprs_example_thread.h"
#include <core/plugin.h>

using namespace fawkes;

/** Example plugin to use OpenPRS from Fawkes.
 * @author Tim Niemueller
 */
class OpenPRSExamplePlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  OpenPRSExamplePlugin(Configuration *config) : Plugin(config)
  {
    thread_list.push_back(new OpenPRSExampleThread());
  }
};


PLUGIN_DESCRIPTION("OpenPRS example plugin")
EXPORT_PLUGIN(OpenPRSExamplePlugin)
