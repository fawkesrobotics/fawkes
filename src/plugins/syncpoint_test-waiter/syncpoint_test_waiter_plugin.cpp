/***************************************************************************
 *  syncpoint_test_waiter_plugin.cpp - SyncPoint Test Waiter Plugin
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

#include "syncpoint_test_waiter_thread.h"

using namespace fawkes;

/** Plugin to test sync points
 * This plugin waits for a given sync point every loop
 * @author Till Hofmann
 */
class SyncPointTestWaiterPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  SyncPointTestWaiterPlugin(Configuration *config)
    : Plugin(config)
  {
    thread_list.push_back(new SyncPointTestWaiterThread());
  }
};

PLUGIN_DESCRIPTION("SyncPoint Test Waiter Plugin")
EXPORT_PLUGIN(SyncPointTestWaiterPlugin)
