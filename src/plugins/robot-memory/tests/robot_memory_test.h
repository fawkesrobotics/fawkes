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

#ifndef _PLUGINS_ROBOT_MEMORY_TEST_H_
#define _PLUGINS_ROBOT_MEMORY_TEST_H_

#include "plugins/robot-memory/robot_memory.h"

#include <blackboard/blackboard.h>
#include <gtest/gtest.h>

#include <bsoncxx/builder/basic/document.hpp>
#include <bsoncxx/document/value.hpp>
#include <bsoncxx/document/view.hpp>
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
	RobotMemoryTestEnvironment(RobotMemory *robot_memory, fawkes::BlackBoard *blackboard)
	{
		this->robot_memory = robot_memory;
		this->blackboard   = blackboard;
	}
	virtual ~RobotMemoryTestEnvironment()
	{
	}
	/// Setup the environment
	void
	SetUp()
	{
	}
	/// TearDown the environment
	virtual void
	TearDown()
	{
	}

public:
	/// Access to Robot Memory
	static RobotMemory *robot_memory;
	/// Access to blackboard
	static fawkes::BlackBoard *blackboard;
};

/** Class for Tests of the RobotMemory
 */
class RobotMemoryTest : public ::testing::Test
{
protected:
	virtual void SetUp();
	/// Access to Robot Memory
	RobotMemory *robot_memory;
	/// Access to blackboard
	fawkes::BlackBoard *blackboard;

protected:
	::testing::AssertionResult contains_pairs(const bsoncxx::document::view_or_value &obj,
	                                          const bsoncxx::document::view_or_value &exp);
	::testing::AssertionResult contains_pairs(mongocxx::cursor &                      cursor,
	                                          const bsoncxx::document::view_or_value &exp);
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
	void
	callback_test(const bsoncxx::document::view &update)
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
	std::list<bsoncxx::document::value>
	compute(const bsoncxx::document::view &query, const std::string &collection)
	{
		std::list<bsoncxx::document::value> res;
		using namespace bsoncxx::builder;
		basic::document doc;
		doc.append(basic::kvp("computed", true));
		doc.append(basic::kvp("result", "this is computed"));
		res.push_back(doc.extract());
		return res;
	}
	/**
     * Computable function for addition
     * @param query Input query
     * @param collection Corresponding collection
     * @return Computed docs
     */
	std::list<bsoncxx::document::value>
	compute_sum(const bsoncxx::document::view &query, const std::string &collection)
	{
		std::list<bsoncxx::document::value> res;
		int                                 x   = query["x"].get_int64();
		int                                 y   = query["y"].get_int64();
		int                                 sum = x + y;
		using namespace bsoncxx::builder;
		basic::document b;
		b.append(basic::kvp("compute", "sum"));
		b.append(basic::kvp("x", x));
		b.append(basic::kvp("y", y));
		b.append(basic::kvp("sum", sum));
		res.push_back(b.extract());
		return res;
	}
	/**
     * Computable function for multiple static document
     * @param query Input query
     * @param collection Corresponding collection
     * @return Computed docs
     */
	std::list<bsoncxx::document::value>
	compute_multiple(const bsoncxx::document::view &query, const std::string &collection)
	{
		std::list<bsoncxx::document::value> res;
		using namespace bsoncxx::builder;
		for (auto i : {1, 2, 3}) {
			basic::document doc;
			doc.append(basic::kvp("compute", "multiple"));
			doc.append(basic::kvp("count", i));
			res.push_back(doc.extract());
		}
		return res;
	}
};

#endif
