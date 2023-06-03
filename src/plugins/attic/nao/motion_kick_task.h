
/***************************************************************************
 *  motion_kick_task.h - Make the robot kick asses... ehm soccer balls
 *
 *  Created: Fri Jan 23 18:33:41 2009
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

#ifndef _PLUGINS_NAO_MOTION_KICK_TASK_H_
#define _PLUGINS_NAO_MOTION_KICK_TASK_H_

#include <alcore/alptr.h>
#include <alproxies/almotionproxy.h>
#include <althread/altask.h>
#include <interfaces/HumanoidMotionInterface.h>

class NaoQiMotionKickTask : public AL::ALTask
{
public:
	NaoQiMotionKickTask(AL::ALPtr<AL::ALMotionProxy>             almotion,
	                    fawkes::HumanoidMotionInterface::LegEnum leg);
	virtual ~NaoQiMotionKickTask();

	virtual void exitTask();
	virtual void run();

private: /* methods */
	void goto_start_pos(AL::ALValue speed, bool concurrent = false);

private:
	bool                                     quit_;
	AL::ALPtr<AL::ALMotionProxy>             almotion_;
	fawkes::HumanoidMotionInterface::LegEnum leg_;
};

#endif
