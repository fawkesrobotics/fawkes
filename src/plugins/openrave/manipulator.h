
/***************************************************************************
 *  manipulator.h - Fawkes to OpenRAVE Manipulator Data
 *
 *  Created: Thu Sep 16 14:50:34 2010
 *  Copyright  2010  Bahram Maleki-Fard, AllemaniACs RoboCup Team
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

#ifndef _PLUGINS_OPENRAVE_MANIPULATOR_H_
#define _PLUGINS_OPENRAVE_MANIPULATOR_H_

#include "types.h"

#include <vector>

namespace fawkes {

class OpenRaveManipulator
{
public:
	OpenRaveManipulator(unsigned int count, unsigned int count_device);
	virtual ~OpenRaveManipulator();

	/** Create a new copy of this OpenRaveManipulator instance
   * @return RefPtr to the copied OpenRaveManipulator
   */
	virtual OpenRaveManipulatorPtr copy() = 0;

	void add_motor(unsigned int number, unsigned int number_device);

	template <typename T_from, typename T_to>
	void angles_or_to_device(std::vector<T_from> &from, std::vector<T_to> &to) const;
	template <typename T>
	void get_angles(std::vector<T> &to) const; // angles of OpenRAVE model
	template <typename T>
	void get_angles_device(std::vector<T> &to) const; // angles of real device

	template <typename T>
	void set_angles(std::vector<T> &angles);
	template <typename T>
	void set_angles_device(std::vector<T> &angles);

protected:
	/** Transform single OpenRAVE motor angle to real device angle
   * @param number motor number of real device
   * @param angle motor angle of OpenRAVE model
   * @return transformed angle
   */
	virtual float angle_OR_to_device(unsigned int number, float angle) const = 0;

	/** Transform single device motor angle to OpenRAVE angle
   * @param number motor number of real device
   * @param angle motor angle of real device
   * @return transformed angle
   */
	virtual float angle_device_to_OR(unsigned int number, float angle) const = 0;

	std::vector<motor_t> motors_;     /**< vector of motors */
	unsigned int         cnt_;        /**< number of motors on OpenRAVE model */
	unsigned int         cnt_device_; /**< number of motors on real device */
};

/* ########## getter ########## */
/** Get motor angles of OpenRAVE model
 * @param to target tvector of angles
 */
template <typename T>
void
OpenRaveManipulator::get_angles(std::vector<T> &to) const
{
	to.resize(cnt_);
	for (unsigned int i = 0; i < motors_.size(); i++) {
		to[motors_[i].no] = (T)motors_[i].angle;
	}
}

/** Get motor angles of real device
 * @param to target vector of angles
 */
template <typename T>
void
OpenRaveManipulator::get_angles_device(std::vector<T> &to) const
{
	std::vector<float> tmp;
	get_angles(tmp);
	angles_or_to_device(tmp, to);
	//to = angles_or_to_device(tmp);
}

/** Transform OpenRAVE motor angles to real device angles
 * @param from motor angles of OpenRAVE model
 * @param to motor angles of real device
 */
template <typename T_from, typename T_to>
void
OpenRaveManipulator::angles_or_to_device(std::vector<T_from> &from, std::vector<T_to> &to) const
{
	to.resize(cnt_device_);

	for (unsigned int i = 0; i < motors_.size(); i++) {
		to[motors_[i].no_device] =
		  (T_to)angle_OR_to_device(motors_[i].no_device, (float)from[motors_[i].no]);
	}
}

/* ########## setter ########## */
/** Set motor angles of OpenRAVE model
 * @param angles motor angles
 */
template <typename T>
void
OpenRaveManipulator::set_angles(std::vector<T> &angles)
{
	if (angles.size() < motors_.size()) {
		angles.reserve(motors_.size());
	}
	for (unsigned int i = 0; i < motors_.size(); i++) {
		motors_[i].angle = (float)angles[motors_[i].no];
	}
}

/** Set motor angles of real device
 * @param angles motor angles
 */
template <typename T>
void
OpenRaveManipulator::set_angles_device(std::vector<T> &angles)
{
	if (angles.size() < motors_.size()) {
		angles.reserve(motors_.size());
	}
	for (unsigned int i = 0; i < motors_.size(); i++) {
		motors_[i].angle =
		  angle_device_to_OR(motors_[i].no_device, (float)angles[motors_[i].no_device]);
	}
}

} // end of namespace fawkes

#endif
