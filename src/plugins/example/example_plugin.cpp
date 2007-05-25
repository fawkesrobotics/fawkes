
/***************************************************************************
 *  example_plugin.cpp - Fawkes Example Plugin
 *
 *  Generated: Wed Nov 22 17:06:33 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <plugins/example/example_plugin.h>
#include <plugins/example/thread.h>
#include <plugins/example/net_thread.h>
#include <plugins/example/finalize_nettler_thread.h>

/** @class ExamplePlugin plugins/example/example_plugin.h
 * Simple example plugin.
 * Creates a few threads for different wakeup hooks that print out messages
 * every 10 iterations.
 *
 * @author Tim Niemueller
 */

/** Modulo count */
#define MODC 100

/** Constructor. */
ExamplePlugin::ExamplePlugin()
  : Plugin(Plugin::MOTION, "example_plugin")
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

}

EXPORT_PLUGIN(ExamplePlugin)
