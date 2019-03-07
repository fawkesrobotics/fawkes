
/***************************************************************************
 *  openrave_thread.cpp - Kinova Jaco plugin OpenRAVE base Thread
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

#include "openrave_base_thread.h"

#include <core/threading/mutex.h>
#include <interfaces/JacoInterface.h>

#include <cmath>
#include <cstring>
#include <stdio.h>

#ifdef HAVE_OPENRAVE
#	include <plugins/openrave/environment.h>
#	include <plugins/openrave/manipulator.h>
#	include <plugins/openrave/manipulators/kinova_jaco.h>
#	include <plugins/openrave/robot.h>
using namespace OpenRAVE;
#endif

using namespace fawkes;
using namespace std;

/** @class JacoOpenraveBaseThread "openrave_base_thread.h"
 * Base Jaco Arm thread, integrating OpenRAVE
 *
 * @author Bahram Maleki-Fard
 */

/** Constructor.
 * @param name thread name
 */
JacoOpenraveBaseThread::JacoOpenraveBaseThread(const char *name)
: Thread(name, Thread::OPMODE_CONTINUOUS)
{
#ifdef HAVE_OPENRAVE
	cfg_OR_auto_load_ik_ = false;
	plot_current_        = false;
#endif
}

/** Destructor. */
JacoOpenraveBaseThread::~JacoOpenraveBaseThread()
{
#ifdef HAVE_OPENRAVE
	viewer_env_.env   = NULL;
	viewer_env_.robot = NULL;
	viewer_env_.manip = NULL;
#endif
}

/** Initializer.
 * Reads common config entries, and loads the viewer-environment.
 * It calls the _init() and _load_robot() methods from inherited classes,
 * which can be used to initialize additional data, and load the robot
 * into the OpenRAVE environment.
 */
void
JacoOpenraveBaseThread::init()
{
	planning_mutex_ = new Mutex();

#ifdef HAVE_OPENRAVE
	cfg_OR_use_viewer_   = config->get_bool("/hardware/jaco/openrave/use_viewer");
	cfg_OR_auto_load_ik_ = config->get_bool("/hardware/jaco/openrave/auto_load_ik");
	cfg_OR_sampling_     = config->get_float("/hardware/jaco/openrave/sampling");

	cfg_OR_plot_traj_manip_ =
	  config->get_bool("/hardware/jaco/openrave/plotting/planned_manipulator");
	cfg_OR_plot_traj_joints_ = config->get_bool("/hardware/jaco/openrave/plotting/planned_joints");
	cfg_OR_plot_cur_manip_ = config->get_bool("/hardware/jaco/openrave/plotting/current_manipulator");
	cfg_OR_plot_cur_joints_ = config->get_bool("/hardware/jaco/openrave/plotting/current_joints");

	// perform other initialization stuff (for child classes, that do not want to overload "init()")
	_init();

	viewer_env_.env = openrave->get_environment();
	viewer_env_.env->enable_debug();
	viewer_env_.env->set_name("Viewer");

	// load robot
	_load_robot();

	if (cfg_OR_use_viewer_)
		openrave->start_viewer();

	_post_init();
#endif
}

void
JacoOpenraveBaseThread::finalize()
{
	delete planning_mutex_;
	planning_mutex_ = NULL;

#ifdef HAVE_OPENRAVE
	viewer_env_.robot = NULL;
	viewer_env_.manip = NULL;
	viewer_env_.env   = NULL;
#endif
}

/** Set planner parameters.
 * The parameter string is passed as is to OpenRAVE's BaseManipulator
 * or DualManipulation module. Errors in the string will result in
 * planning failures.
 * @param params parameters string
 */
void
JacoOpenraveBaseThread::set_plannerparams(const std::string &params)
{
#ifdef HAVE_OPENRAVE
	plannerparams_ = params;
#endif
}

/** Set planner parameters.
 * The parameter string is passed as is to OpenRAVE's BaseManipulator
 * or DualManipulation module. Errors in the string will result in
 * planning failures.
 * @param params parameters string
 */
void
JacoOpenraveBaseThread::set_plannerparams(const char *params)
{
#ifdef HAVE_OPENRAVE
	plannerparams_ = params;
#endif
}

/** Enable/Disable plotting of the current arm position.
 * @param enable Set the "enabled" state
 */
void
JacoOpenraveBaseThread::plot_current(bool enable)
{
#ifdef HAVE_OPENRAVE
	plot_current_ = enable;
#endif
}
