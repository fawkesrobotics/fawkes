
/***************************************************************************
 *  example_plugin.cpp - Fawkes Example Plugin
 *
 *  Generated: Wed Nov 22 17:06:33 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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

#include <plugins/examples/basics/example_plugin.h>
#include <plugins/examples/basics/thread.h>
#include <plugins/examples/basics/net_thread.h>
#include <plugins/examples/basics/finalize_nettler_thread.h>
#include <plugins/examples/basics/blackboard_thread.h>

/** @class ExamplePlugin plugins/examples/basics/example_plugin.h
 * Simple example plugin.
 * Creates a few threads for different wakeup hooks that print out messages
 * every 10 iterations.
 *
 * @author Tim Niemueller
 */

using namespace fawkes;

/** Modulo count */
#define MODC 100

/** Constructor.
 * @param config Fawkes configuration
 */
ExamplePlugin::ExamplePlugin(Configuration *config)
  : Plugin(config)
{
  // printf("ExamplePlugin constructor called\n");
  thread_list.push_back(new ExampleThread(BlockedTimingAspect::WAKEUP_HOOK_PRE_LOOP,
					  "PreLoopThread", MODC));
  thread_list.push_back(new ExampleThread(BlockedTimingAspect::WAKEUP_HOOK_SENSOR,
					  "SensorThread", MODC));
  thread_list.push_back(new ExampleThread(BlockedTimingAspect::WAKEUP_HOOK_WORLDSTATE,
					  "WorldStateThread", MODC));
  thread_list.push_back(new ExampleThread(BlockedTimingAspect::WAKEUP_HOOK_THINK,
					  "ThinkThread", MODC));
  thread_list.push_back(new ExampleThread(BlockedTimingAspect::WAKEUP_HOOK_SKILL,
					  "SkillThread", MODC));
  thread_list.push_back(new ExampleThread(BlockedTimingAspect::WAKEUP_HOOK_ACT,
					  "ActThread", MODC));
  thread_list.push_back(new ExampleThread(BlockedTimingAspect::WAKEUP_HOOK_POST_LOOP,
					  "PostLoopThread", MODC));
  thread_list.push_back(new ExampleNetworkThread("NetworkThread"));
  thread_list.push_back(new ExampleFinalizeNettlerThread("FinalizeNettlerThread"));
  thread_list.push_back(new ExampleBlackBoardThread(/* reader */ true));
  thread_list.push_back(new ExampleBlackBoardThread(/* reader */ false));
}

PLUGIN_DESCRIPTION("Example plugin demonstrating Fawkes basics")
EXPORT_PLUGIN(ExamplePlugin)
