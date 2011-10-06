
/***************************************************************************
 *  motion_standup_task.h - Task for making the robot stand up
 *
 *  Created: Mon Jan 19 14:16:54 2009
 *  Copyright  2009-2011  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_NAO_MOTION_STANDUP_TASK_H_
#define __PLUGINS_NAO_MOTION_STANDUP_TASK_H_

#include <interfaces/HumanoidMotionInterface.h>

#include <althread/altask.h>
#include <alcore/alptr.h>
#include <alproxies/almotionproxy.h>

class NaoQiMotionStandupTask : public AL::ALTask
{
 public:
  NaoQiMotionStandupTask(AL::ALPtr<AL::ALMotionProxy> almotion,
			 fawkes::HumanoidMotionInterface::StandupEnum from_pos,
			 float accel_x, float accel_y, float accel_z);
  virtual ~NaoQiMotionStandupTask();

  virtual void run();

 private: /* methods */
  void goto_start_pos();
  void standup_from_back();
  void standup_from_front();

 private:
  AL::ALPtr<AL::ALMotionProxy>  __almotion;
  fawkes::HumanoidMotionInterface::StandupEnum __from_pos;

  float __accel_x;
  float __accel_y;
  float __accel_z;
};

#endif
