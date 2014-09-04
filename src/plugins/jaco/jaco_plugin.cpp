
/***************************************************************************
 *  jaco_plugin.cpp - Fawkes Kinova Jaco Plugin
 *
 *  Created: Tue Jun 04 13:13:20 2013
 *  Copyright  2013  Bahram Maleki-Fard
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

#include "jaco_plugin.h"

#include "info_thread.h"
#include "act_thread.h"
#include "goto_thread.h"
#include "openrave_single_thread.h"
#include "openrave_dual_thread.h"

using namespace fawkes;

/** @class JacoPlugin <plugins/jaco/jaco_plugin.h>
 * Kinova Jaco Arm plugin.
 *
 * @author Bahram Maleki-Fard
 */

/** Constructor.
 * @param config Fawkes configuration
 */
JacoPlugin::JacoPlugin(Configuration *config)
  : Plugin(config)
{
  JacoInfoThread *info_thread = new JacoInfoThread();
  thread_list.push_back(info_thread);

  // load different/multiple threads if using dual-arm setup
  bool is_dual_arm = config->get_bool("/hardware/jaco/dual_arm/active");
  if( !is_dual_arm ) {
    JacoGotoThread *goto_thread = new JacoGotoThread("JacoGotoThread");
    thread_list.push_back(goto_thread);

    JacoOpenraveBaseThread *openrave_thread = NULL;
#ifdef HAVE_OPENRAVE
    openrave_thread = new JacoOpenraveSingleThread("JacoOpenraveThread", "fingertip");
    thread_list.push_back(openrave_thread);
#endif

    thread_list.push_back(new JacoActThread(info_thread, goto_thread, openrave_thread));

  } else {
    // each arm gets 1 goto-thread.
    JacoGotoThread *goto_thread_l = new JacoGotoThread("JacoGotoThreadLeft");
    JacoGotoThread *goto_thread_r = new JacoGotoThread("JacoGotoThreadRight");
    thread_list.push_back(goto_thread_l);
    thread_list.push_back(goto_thread_r);

    // each arm gets 1 openrave-thread, providing planning and updating the openrave-model of that arm.
    // additionally we need a separate openrave thread for planning symmetric bimanual manipulation.
    JacoOpenraveSingleThread *openrave_thread_l = NULL;
    JacoOpenraveSingleThread *openrave_thread_r = NULL;
    JacoOpenraveDualThread   *openrave_thread_dual = NULL;

#ifdef HAVE_OPENRAVE
    openrave_thread_dual = new JacoOpenraveDualThread();
    thread_list.push_back(openrave_thread_dual);

    openrave_thread_l = new JacoOpenraveSingleThread("JacoOpenraveThreadLeft", "arm_left", /*load_robot=*/false);
    openrave_thread_r = new JacoOpenraveSingleThread("JacoOpenraveThreadRight", "arm_right", /*load_robot=*/false);
    thread_list.push_back(openrave_thread_l);
    thread_list.push_back(openrave_thread_r);
#endif

    thread_list.push_back(new JacoActThread(info_thread,
                                              goto_thread_l, goto_thread_r,
                                              openrave_thread_l, openrave_thread_r,
                                              openrave_thread_dual));
  }
}

PLUGIN_DESCRIPTION("Jaco Plugin")
EXPORT_PLUGIN(JacoPlugin)
