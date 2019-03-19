
/***************************************************************************
 *  goto_openrave_thread.h - Katana goto one-time thread using openrave lib
 *
 *  Created: Wed Jun 10 11:44:24 2009
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
 *             2011-2014  Bahram Maleki-Fard
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

#ifndef _PLUGINS_KATANA_GOTO_OPENRAVE_THREAD_H_
#define _PLUGINS_KATANA_GOTO_OPENRAVE_THREAD_H_

#include "motion_thread.h"

#ifdef HAVE_OPENRAVE
#	include <plugins/openrave/types.h>
#endif

#include <string>
#include <vector>

namespace fawkes {
class OpenRaveConnector;
}

/** class KatanaGotoOpenRaveThread */
class KatanaGotoOpenRaveThread : public KatanaMotionThread
{
#ifdef HAVE_OPENRAVE
public:
	KatanaGotoOpenRaveThread(fawkes::RefPtr<fawkes::KatanaController> katana,
	                         fawkes::Logger *                         logger,
	                         fawkes::OpenRaveConnector *              openrave,
	                         unsigned int                             poll_interval_ms,
	                         const std::string &                      robot_file,
	                         const std::string &                      arm_model,
	                         bool                                     autoload_IK,
	                         bool                                     use_viewer);

	virtual void once();
	virtual void init();
	virtual void finalize();

	void set_target(float x, float y, float z, float phi, float theta, float psi);
	void
	     set_target(float x, float y, float z, float quat_x, float quat_y, float quat_z, float quat_w);
	void set_target(const std::string &object_name, float rot_x);
	void set_theta_error(float error);
	void set_move_straight(bool move_straight);
	void set_arm_extension(bool arm_extension);
	void set_plannerparams(std::string &params, bool straight = false);
	void set_plannerparams(const char *params, bool straight = false);

	virtual bool plan_target();
	virtual void update_openrave_data();
	virtual bool update_motor_data();
	virtual bool move_katana();

	static const std::string DEFAULT_PLANNERPARAMS;
	static const std::string DEFAULT_PLANNERPARAMS_STRAIGHT;

private:
	fawkes::OpenRaveRobotPtr       OR_robot_;
	fawkes::OpenRaveManipulatorPtr OR_manip_;

	std::string                               target_object_;
	std::vector<std::vector<float>> *         target_traj_;
	std::vector<std::vector<float>>::iterator it_;

	std::vector<int>   motor_encoders_;
	std::vector<float> motor_angles_;

	const std::string cfg_robot_file_;
	const std::string cfg_arm_model_;
	bool              cfg_autoload_IK_;
	bool              cfg_use_viewer_;

	bool        is_target_object_;
	bool        has_target_quaternion_;
	bool        move_straight_;
	bool        is_arm_extension_;
	std::string plannerparams_;
	std::string plannerparams_straight_;

	fawkes::OpenRaveConnector *_openrave;

	float        x_, y_, z_;
	float        phi_, theta_, psi_, theta_error_;
	float        quat_x_, quat_y_, quat_z_, quat_w_;
	unsigned int poll_interval_usec_;

#endif //HAVE_OPENRAVE
};

#endif
