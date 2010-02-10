
/***************************************************************************
 *  qa_player_setmotor.cpp - Player QA App: set motor values
 *
 *  Created: Tue Sep 30 23:40:57 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#include <blackboard/remote.h>
#include <interfaces/MotorInterface.h>

#include <unistd.h>
#include <cstdio>

using namespace fawkes;

int
main(int argc, char **argv)
{
  BlackBoard *bb = new RemoteBlackBoard("localhost", 1910);

  MotorInterface *motor = bb->open_for_reading<MotorInterface>("Player Motor");
  motor->read();

  printf("Motor x=%f   y=%f   z=%f\n",
	 motor->odometry_position_x(), motor->odometry_position_y(),
	 motor->odometry_orientation());

  printf("Setting relative (2, 0, 0), this is (%f, %f, %f) global\n",
	 motor->odometry_position_x() + 2, motor->odometry_position_y(),
	 motor->odometry_orientation());

  motor->msgq_enqueue(new MotorInterface::GotoMessage(motor->odometry_position_x() + 2,
						      motor->odometry_position_y(),
						      motor->odometry_orientation(),
						      1 /* sec */));

  bb->close(motor);

  delete bb;
  return 0;
}
