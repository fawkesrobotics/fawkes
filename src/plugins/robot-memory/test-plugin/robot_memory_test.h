/***************************************************************************
 *  robot_memory_test.h - Test for the RobotMemory and their test class
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

#ifndef __PLUGINS_ROBOT_MEMORY_TEST_H_
#define __PLUGINS_ROBOT_MEMORY_TEST_H_

#include <gtest/gtest.h>
#include "plugins/robot-memory/robot_memory.h"


/** Environment for running Tests of the RobotMemory
 * Necessary for making object such as the robot memory available in tests.
 */
class RobotMemoryTestEnvironment : public ::testing::Environment
{
 public:
  RobotMemoryTestEnvironment(RobotMemory* robot_memory)
  {
    this->robot_memory = robot_memory;
  }
  virtual ~RobotMemoryTestEnvironment() {}
  void SetUp() {}
  virtual void TearDown(){}
 public:
  static RobotMemory* robot_memory;
};

/** Class for Tests of the RobotMemory
 */
class RobotMemoryTest : public ::testing::Test
{
 protected:
  virtual void SetUp();
  RobotMemory* robot_memory;

 protected:
  ::testing::AssertionResult contains_pairs(mongo::BSONObj obj, mongo::BSONObj exp);
};

/**
 * Class to register callbacks independent of how many tests are using them at the moment
 */
class RobotMemoryCallback
{
  public:
    RobotMemoryCallback()
  {
      callback_counter = 0;
  };
   ~RobotMemoryCallback(){};
   int callback_counter;
   void callback_test(mongo::BSONObj update)
   {
     callback_counter++;
   }
};


#endif
