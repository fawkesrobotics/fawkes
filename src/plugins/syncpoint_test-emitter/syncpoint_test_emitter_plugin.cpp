/***************************************************************************
 *  syncpoint_test_emitter_plugin.cpp - SyncPoint Test Emitter Plugin
 *
 *   Created on Wed Jan 08 17:12:13 2014
 *   Copyright  2014  Till Hofmann
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

#include "syncpoint_test_emitter_thread.h"

using namespace fawkes;

/** Plugin to test sync points
 * This plugin emits sync points
 * @author Till Hofmann
 */
class SyncPointTestEmitterPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  SyncPointTestEmitterPlugin(Configuration *config)
    : Plugin(config)
  {
    thread_list.push_back(new SyncPointTestEmitterThread());
  }
};

PLUGIN_DESCRIPTION("SyncPoint Test Emitter Plugin")
EXPORT_PLUGIN(SyncPointTestEmitterPlugin)
