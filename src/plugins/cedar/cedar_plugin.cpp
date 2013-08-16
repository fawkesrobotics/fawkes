
/***************************************************************************
 *  cedar_plugin.cpp - CLIPS-based Error Detection, Analysis, and Recovery
 *
 *  Created: Fri Aug 16 18:00:32 2013 +0200
 *  Copyright  2006-2013  Tim Niemueller [www.niemueller.de]
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

#include "cedar_thread.h"
#include <core/plugin.h>

using namespace fawkes;

/** CLIPS agent plugin.
 * @author Tim Niemueller
 */
class CedarPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  CedarPlugin(Configuration *config) : Plugin(config)
  {
    thread_list.push_back(new CedarThread());
  }
};


PLUGIN_DESCRIPTION("CLIPS-based Error Detection, Analysis, and Recovery")
EXPORT_PLUGIN(CedarPlugin)
