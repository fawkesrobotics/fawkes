
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
#include "openrave_thread.h"

#include "bimanual_act_thread.h"
#include "bimanual_goto_thread.h"
#include "bimanual_openrave_thread.h"

#include "types.h"

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
  // load different/multiple threads if using dual-arm setup
  bool is_dual_arm = config->get_bool("/hardware/jaco/config/dual_arm");
  if( !is_dual_arm ) {
    jaco_arm_t* arm = new jaco_arm_t();
    arm->config=CONFIG_SINGLE;

    JacoActThread* act_thread = new JacoActThread("JacoActThread", arm);
    JacoInfoThread* info_thread = new JacoInfoThread("JacoInfoThread", arm);
    JacoGotoThread* goto_thread = new JacoGotoThread("JacoGotoThread", arm);
    JacoOpenraveThread* openrave_thread = NULL;
#ifdef HAVE_OPENRAVE
    openrave_thread = new JacoOpenraveThread("JacoOpenraveThread", arm);
#endif

    arm->goto_thread = goto_thread;
    arm->openrave_thread = openrave_thread;

    thread_list.push_back( act_thread );
    thread_list.push_back( info_thread );
    thread_list.push_back( goto_thread );
#ifdef HAVE_OPENRAVE
    thread_list.push_back( openrave_thread );
#endif

  } else {
    jaco_arm_t* arm_l = new jaco_arm_t();
    jaco_arm_t* arm_r = new jaco_arm_t();
    arm_l->config=CONFIG_LEFT;
    arm_r->config=CONFIG_RIGHT;


    // each arm gets a separate set of threads for independent manipulation.
    JacoActThread* act_thread_l = new JacoActThread("JacoActThreadLeft", arm_l);
    JacoInfoThread* info_thread_l = new JacoInfoThread("JacoInfoThreadLeft", arm_l);
    JacoGotoThread* goto_thread_l = new JacoGotoThread("JacoGotoThreadLeft", arm_l);

    JacoActThread* act_thread_r =  new JacoActThread("JacoActThreadRight", arm_r);
    JacoInfoThread* info_thread_r = new JacoInfoThread("JacoInfoThreadRight", arm_r);
    JacoGotoThread* goto_thread_r = new JacoGotoThread("JacoGotoThreadRight", arm_r);

    JacoOpenraveThread* openrave_thread_l = NULL;
    JacoOpenraveThread* openrave_thread_r = NULL;
#ifdef HAVE_OPENRAVE
    // each arm gets 1 openrave-thread, providing planning and updating the openrave-model of that arm.
    // additionally we need a separate openrave thread for planning symmetric bimanual manipulation.
    openrave_thread_l = new JacoOpenraveThread("JacoOpenraveThreadLeft", arm_l, /*load_robot=*/false);
    openrave_thread_r = new JacoOpenraveThread("JacoOpenraveThreadRight", arm_r, /*load_robot=*/false);
#endif

    arm_l->goto_thread = goto_thread_l;
    arm_l->openrave_thread = openrave_thread_l;

    arm_r->goto_thread = goto_thread_r;
    arm_r->openrave_thread = openrave_thread_r;

    thread_list.push_back( act_thread_l );
    thread_list.push_back( info_thread_l );
    thread_list.push_back( goto_thread_l );

    thread_list.push_back( act_thread_r );
    thread_list.push_back( info_thread_r );
    thread_list.push_back( goto_thread_r );

    // we also need a set of threads for coordinated bimanual manipulation
    jaco_dual_arm_t* arms = new jaco_dual_arm_t();
    arms->left = arm_l;
    arms->right = arm_r;

    JacoBimanualActThread* act_thread = new JacoBimanualActThread(arms);
    JacoBimanualGotoThread* goto_thread = new JacoBimanualGotoThread(arms);

    JacoBimanualOpenraveThread* openrave_thread = NULL;
#ifdef HAVE_OPENRAVE
    openrave_thread = new JacoBimanualOpenraveThread(arms);
#endif

    arms->goto_thread = goto_thread;
    arms->openrave_thread = openrave_thread;

    thread_list.push_back( act_thread );
    thread_list.push_back( goto_thread );
#ifdef HAVE_OPENRAVE
    thread_list.push_back( openrave_thread );
    thread_list.push_back( openrave_thread_l );
    thread_list.push_back( openrave_thread_r );
#endif
  }
}

PLUGIN_DESCRIPTION("Jaco Plugin")
EXPORT_PLUGIN(JacoPlugin)
