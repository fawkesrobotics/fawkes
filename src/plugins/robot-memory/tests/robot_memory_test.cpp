
/***************************************************************************
 *  robot_memory_test.cpp - Unit tests for the robot memory
 *    
 *
 *  Created: Aug 18, 2016 12:40:46 PM 2016
 *  Copyright  2016  Frederik Zwilling
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

#include <gtest/gtest.h>

#include <plugins/robot-memory/robot_memory_thread.h>
#include <stdexcept>

using namespace fawkes;


TEST(RobotMemoryTest, TestsWorking)
{
  ASSERT_EQ(1, 3-2);
}
