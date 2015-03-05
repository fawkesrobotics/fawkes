/***************************************************************************
 *  test_plugin.cpp - SyncPoint Test plugin
 *
 *  Created: Wed Mar 04 17:57:42 2015
 *  Copyright  2015  Till Hofmann
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
#include "test_thread.h"
#include <aspect/blocked_timing.h>
#include <vector>
#include <algorithm>

using namespace fawkes;

/** Plugin to test syncpoints
 * @author Till Hofmann
 */
class SyncPointTestPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
    SyncPointTestPlugin(Configuration *config)
    : Plugin(config)
  {
      std::vector<BlockedTimingAspect::WakeupHook> hooks;
      hooks = {
          BlockedTimingAspect::WAKEUP_HOOK_PRE_LOOP, /**< before each loop */
          BlockedTimingAspect::WAKEUP_HOOK_SENSOR_ACQUIRE, /**< sensor acquisition thread,
                                       *  acquire data from sensor */
          BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PREPARE, /**< sensor data preparation thread,
                                       * convert acquired data to usable format */
          BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS, /**< sensor data processing thread */
          BlockedTimingAspect::WAKEUP_HOOK_WORLDSTATE, /**< world state thread */
          BlockedTimingAspect::WAKEUP_HOOK_THINK,    /**< think thread (agent) */
          BlockedTimingAspect::WAKEUP_HOOK_SKILL,    /**< skill thread (skill module) */
          BlockedTimingAspect::WAKEUP_HOOK_ACT,    /**< act thread (motor module etc.) */
          BlockedTimingAspect::WAKEUP_HOOK_ACT_EXEC, /**< act execution thread */
          BlockedTimingAspect::WAKEUP_HOOK_POST_LOOP /**< run after loop */
      };

      for (std::vector<BlockedTimingAspect::WakeupHook>::iterator it = hooks.begin();
          it != hooks.end(); it++) {
        std::string name = "SyncPointTestThread-";
        std::string hook_name = BlockedTimingAspect::blocked_timing_hook_to_string(*it);
        std::transform(hook_name.begin(), hook_name.end(), hook_name.begin(), ::tolower);
        name.append(hook_name);
        thread_list.push_back(new SyncPointTestThread(name.c_str(), *it));
      }
  }
};

PLUGIN_DESCRIPTION("Plugin to test SyncPoints and the BlockedTimingAspect")
EXPORT_PLUGIN(SyncPointTestPlugin)

