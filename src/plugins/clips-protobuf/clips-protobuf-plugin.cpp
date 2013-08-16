
/***************************************************************************
 *  clips-protobuf-plugin.cpp - Agent plugin based on CLIPS
 *
 *  Created: Tue Apr 16 13:00:01 2013
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

#include "clips-protobuf-thread.h"
#include <core/plugin.h>

using namespace fawkes;

/** CLIPS agent plugin.
 * @author Tim Niemueller
 */
class ClipsProtobufPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  ClipsProtobufPlugin(Configuration *config) : Plugin(config)
  {
    thread_list.push_back(new ClipsProtobufThread());
  }
};


PLUGIN_DESCRIPTION("Protobuf communication for CLIPS")
EXPORT_PLUGIN(ClipsProtobufPlugin)
