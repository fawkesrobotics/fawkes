/***************************************************************************
 *  robot_memory_test.cpp - Test for the RobotMemory and their test class
 *    
 *
 *  Created: 3:11:53 PM 2016
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

#include "robot_memory_test.h"

//init static variable
RobotMemory* RobotMemoryTestEnvironment::robot_memory = NULL;



/**
 * Setup for each test
 */
void RobotMemoryTest::SetUp()
{
  robot_memory = RobotMemoryTestEnvironment::robot_memory;
}

TEST_F(RobotMemoryTest, TestsWorking)
{
  ASSERT_EQ(1, 3-2);
}

TEST_F(RobotMemoryTest, AspectAvailable)
{
  ASSERT_FALSE(robot_memory==NULL);
}




