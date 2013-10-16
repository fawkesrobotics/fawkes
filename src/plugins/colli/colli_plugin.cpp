/***************************************************************************
 *  colli_plugin.cpp - Fawkes Colli Plugin
 *
 *  Created: Wed Oct 16 18:00:00 2013
 *  Copyright  2013  AllemaniACs
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

using namespace fawkes;

class ColliPlugin : public fawkes::Plugin
{
public:
  ColliPlugin(Configuration *config)
      : Plugin(config)
  {
  }
};

PLUGIN_DESCRIPTION("Local locomotion path planning with collision avoidance")
EXPORT_PLUGIN(ColliPlugin)

