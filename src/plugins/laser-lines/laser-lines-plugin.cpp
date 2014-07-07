
/***************************************************************************
 *  laser-lines-plugin.cpp - Detect a lines from 2D laser data
 *
 *  Created: Fri May 23 18:10:33 2014
 *  Copyright  2011-2014  Tim Niemueller [www.niemueller.de]
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

#include "laser-lines-thread.h"

using namespace fawkes;

/** Plugin to detect lines in 2D laser data.
 * @author Tim Niemueller
 */
class LaserLinesPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  LaserLinesPlugin(Configuration *config)
    : Plugin(config)
  {
    thread_list.push_back(new LaserLinesThread());
  }
};

PLUGIN_DESCRIPTION("Detect lines in 2D laser data")
EXPORT_PLUGIN(LaserLinesPlugin)
