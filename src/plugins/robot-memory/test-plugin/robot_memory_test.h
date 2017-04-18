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
#include <blackboard/blackboard.h>
#include "plugins/robot-memory/robot_memory.h"
#include <stdio.h>


/** Environment for running Tests of the RobotMemory
 * Necessary for making object such as the robot memory available in tests.
 */
class RobotMemoryTestEnvironment : public ::testing::Environment
{
  public:
    /**
     * Constructor with objects of the thread
     * @param robot_memory Robot Memory
     * @param blackboard Blackboard
     */
    RobotMemoryTestEnvironment(RobotMemory* robot_memory, fawkes::BlackBoard* blackboard)
    {
      this->robot_memory = robot_memory;
      this->blackboard = blackboard;
    }
    virtual ~RobotMemoryTestEnvironment() {}
    /// Setup the environment
    void SetUp() {}
    /// TearDown the environment
    virtual void TearDown(){}
  public:
    /// Access to Robot Memory
    static RobotMemory* robot_memory;
    /// Access to blackboard
    static fawkes::BlackBoard* blackboard;
};

/** Class for Tests of the RobotMemory
 */
class RobotMemoryTest : public ::testing::Test
{
  protected:
    virtual void SetUp();
    /// Access to Robot Memory
    RobotMemory* robot_memory;
    /// Access to blackboard
    fawkes::BlackBoard* blackboard;

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
    /// Counter for how often the callback was called
    int callback_counter;
    /**
     * Test callback function
     * @param update Trigger update
     */
    void callback_test(mongo::BSONObj update)
    {
      callback_counter++;
    }
};

/**
 * Class providing a computable function
 */
class TestComputable
{
  public:
    TestComputable(){};
    ~TestComputable(){};
    //Different functions for computables:
    /**
     * Computable function for static document
     * @param query Input query
     * @param collection Corresponding collection
     * @return Computed docs
     */
    std::list<mongo::BSONObj> compute(mongo::BSONObj query, std::string collection)
    {
      std::list<mongo::BSONObj> res;
      res.push_back(mongo::fromjson("{computed:true, result:'this is computed'}"));
      return res;
    }
    /**
     * Computable function for addition
     * @param query Input query
     * @param collection Corresponding collection
     * @return Computed docs
     */
    std::list<mongo::BSONObj> compute_sum(mongo::BSONObj query, std::string collection)
    {
      std::list<mongo::BSONObj> res;
      int x = query.getField("x").Int();
      int y = query.getField("y").Int();
      int sum = x + y;
      mongo::BSONObjBuilder b;
      b << "compute" << "sum" << "x" << x << "y" << y
          << "sum" << sum;
      res.push_back(b.obj());
      return res;
    }
    /**
     * Computable function for multiple static document
     * @param query Input query
     * @param collection Corresponding collection
     * @return Computed docs
     */
    std::list<mongo::BSONObj> compute_multiple(mongo::BSONObj query, std::string collection)
    {
      std::list<mongo::BSONObj> res;
      res.push_back(mongo::fromjson("{compute:'multiple', count:1}"));
      res.push_back(mongo::fromjson("{compute:'multiple', count:2}"));
      res.push_back(mongo::fromjson("{compute:'multiple', count:3}"));
      return res;
    }
};


#endif
