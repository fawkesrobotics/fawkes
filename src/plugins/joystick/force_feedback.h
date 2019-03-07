
/***************************************************************************
 *  force_feedback.h - Force feedback for joysticks using Linux input API
 *
 *  Created: Sun Feb 06 23:50:57 2011 (Super Bowl XLV)
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#ifndef _PLUGINS_JOYSTICK_FORCE_FEEDBACK_H_
#define _PLUGINS_JOYSTICK_FORCE_FEEDBACK_H_

#include <linux/input.h>

#include <stdint.h>

class JoystickForceFeedback
{
public:
	/** Direction of the effect. */
	typedef enum {
		DIRECTION_DOWN  = 0x0000, /**< Downward effect direction. */
		DIRECTION_LEFT  = 0x4000, /**< Left effect direction. */
		DIRECTION_UP    = 0x8000, /**< Upward effect direction. */
		DIRECTION_RIGHT = 0xC000  /**< Right effect direction. */
	} Direction;

	JoystickForceFeedback(const char *device_name);
	~JoystickForceFeedback();

	void rumble(uint16_t  strong_magnitude,
	            uint16_t  weak_magnitude,
	            Direction direction = DIRECTION_DOWN,
	            uint16_t  length    = 0,
	            uint16_t  delay     = 0);

	void stop_all();
	void stop_rumble();

	bool
	is_rumbling()
	{
		return (rumble_.id != -1);
	}
	bool
	can_rumble()
	{
		return can_rumble_;
	}
	bool
	can_periodic()
	{
		return can_periodic_;
	}
	bool
	can_constant()
	{
		return can_constant_;
	}
	bool
	can_spring()
	{
		return can_spring_;
	}
	bool
	can_friction()
	{
		return can_friction_;
	}
	bool
	can_damper()
	{
		return can_damper_;
	}
	bool
	can_inertia()
	{
		return can_inertia_;
	}
	bool
	can_ramp()
	{
		return can_ramp_;
	}
	bool
	can_square()
	{
		return can_square_;
	}
	bool
	can_triangle()
	{
		return can_triangle_;
	}
	bool
	can_sine()
	{
		return can_sine_;
	}
	bool
	can_saw_up()
	{
		return can_saw_up_;
	}
	bool
	can_saw_down()
	{
		return can_saw_down_;
	}
	bool
	can_custom()
	{
		return can_custom_;
	}

private:
	int              fd_;
	struct ff_effect rumble_;

	int num_effects_;

	bool can_rumble_;
	bool can_periodic_;
	bool can_constant_;
	bool can_spring_;
	bool can_friction_;
	bool can_damper_;
	bool can_inertia_;
	bool can_ramp_;
	bool can_square_;
	bool can_triangle_;
	bool can_sine_;
	bool can_saw_up_;
	bool can_saw_down_;
	bool can_custom_;
};

#endif
