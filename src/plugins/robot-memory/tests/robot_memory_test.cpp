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

#include <interfaces/Position3DInterface.h>

#include <algorithm>
#include <bsoncxx/exception/exception.hpp>
#include <list>
#include <math.h>
#include <mongocxx/exception/exception.hpp>

using namespace fawkes;
using namespace mongocxx;

//init static variable
RobotMemory *RobotMemoryTestEnvironment::robot_memory = NULL;
BlackBoard * RobotMemoryTestEnvironment::blackboard   = NULL;

/**
 * Setup for each test
 */
void
RobotMemoryTest::SetUp()
{
	robot_memory = RobotMemoryTestEnvironment::robot_memory;
	blackboard   = RobotMemoryTestEnvironment::blackboard;
}

TEST_F(RobotMemoryTest, TestsWorking)
{
	ASSERT_EQ(1, 3 - 2);
}

TEST_F(RobotMemoryTest, AspectAvailable)
{
	ASSERT_FALSE(robot_memory == NULL);
}

TEST_F(RobotMemoryTest, QueryResultEmpty)
{
	ASSERT_TRUE(
	  robot_memory->insert(bsoncxx::from_json("{'insert':'something to have the namespace'}")));
	auto qres = robot_memory->query(bsoncxx::from_json("{somekey:'should_not_exist'}"));
	ASSERT_EQ(qres.begin(), qres.end());
}

TEST_F(RobotMemoryTest, StoreAndQuery)
{
	ASSERT_TRUE(robot_memory->insert(bsoncxx::from_json("{'testkey':'value'}")));
	auto qres = robot_memory->query(bsoncxx::from_json("{'testkey':'value'}"));
	ASSERT_TRUE(contains_pairs(qres, bsoncxx::from_json("{'testkey':'value'}")));
}

TEST_F(RobotMemoryTest, StoreAndQueryOtherCollection)
{
	ASSERT_TRUE(
	  robot_memory->insert(bsoncxx::from_json("{'testkey':'value'}"), "robmem.othercollection"));
	auto qres =
	  robot_memory->query(bsoncxx::from_json("{'testkey':'value'}"), "robmem.othercollection");
	ASSERT_TRUE(contains_pairs(qres, bsoncxx::from_json("{'testkey':'value'}")));
}

TEST_F(RobotMemoryTest, StoreUpdateQuery)
{
	ASSERT_TRUE(robot_memory->insert("{'inserting':'something',as:0.5}"));
	ASSERT_TRUE(
	  robot_memory->update(bsoncxx::from_json("{'inserting':'something',as:0.5}"),
	                       bsoncxx::from_json("{'updated':'something',as:3.0,extra:true}")));
	auto qres = robot_memory->query(bsoncxx::from_json("{'updated':'something'}"));
	ASSERT_TRUE(
	  contains_pairs(qres, bsoncxx::from_json("{'updated':'something',as:3.0,extra:true}")));
}

TEST_F(RobotMemoryTest, StoreRemoveQuery)
{
	ASSERT_TRUE(robot_memory->insert(bsoncxx::from_json("{to_be:'removed'}")));
	ASSERT_TRUE(robot_memory->remove(bsoncxx::from_json("{to_be:'removed'}")));
	auto qres = robot_memory->query(bsoncxx::from_json("{to_be:'removed'}"));
	ASSERT_EQ(qres.begin(), qres.end());
}

TEST_F(RobotMemoryTest, Upsert)
{
	ASSERT_TRUE(robot_memory->update(bsoncxx::from_json("{upsert:'not existing'}"),
	                                 bsoncxx::from_json("{upsert:'should not exist'}"),
	                                 "",
	                                 false));
	auto qres = robot_memory->query(bsoncxx::from_json("{upsert:'should not exist'}"));
	ASSERT_EQ(qres.begin(), qres.end());
	ASSERT_TRUE(robot_memory->update(bsoncxx::from_json("{upsert:'not existing'}"),
	                                 bsoncxx::from_json("{upsert:'should exist'}"),
	                                 "",
	                                 true));
	qres = robot_memory->query(bsoncxx::from_json("{upsert:'should exist'}"));
	ASSERT_NE(qres.begin(), qres.end());
}

TEST_F(RobotMemoryTest, QueryInvalid)
{
	ASSERT_THROW(robot_memory->query(bsoncxx::from_json("{key-:+'not existing'}")),
	             bsoncxx::exception);
}

TEST_F(RobotMemoryTest, InsertInvalidCaught)
{
	ASSERT_THROW(robot_memory->insert(bsoncxx::from_json("{'testkey'::'value'}")),
	             bsoncxx::exception);
	ASSERT_THROW(robot_memory->insert(bsoncxx::from_json("warbagarbl")), bsoncxx::exception);
}

TEST_F(RobotMemoryTest, UpdateInvalidCaught)
{
	ASSERT_THROW(robot_memory->update(bsoncxx::from_json("{'testkey':'good'}"),
	                                  bsoncxx::from_json("{'bad':1.2.3}")),
	             bsoncxx::exception);
	ASSERT_THROW(robot_memory->update(bsoncxx::from_json("{([})]"), bsoncxx::from_json("{'key':4}")),
	             bsoncxx::exception);
}

TEST_F(RobotMemoryTest, RemoveInvalidCaught)
{
	ASSERT_THROW(robot_memory->remove(bsoncxx::from_json("{___:4.56!}")), bsoncxx::exception);
	ASSERT_THROW(robot_memory->remove(bsoncxx::from_json("{([})]")), bsoncxx::exception);
}

/*
TEST_F(RobotMemoryTest, AggregationSumQuery)
{
	ASSERT_TRUE(robot_memory->insert(bsoncxx::from_json("{'agg':'summand',value:0.5}")));
	ASSERT_TRUE(robot_memory->insert(bsoncxx::from_json("{'agg':'summand',value:0.7}")));
	ASSERT_TRUE(robot_memory->insert(bsoncxx::from_json("{'agg':'not-summand',value:0.9}")));

	std::vector<BSONObj> pipeline;
	pipeline.push_back(bsoncxx::from_json("{'$match': {'agg':'summand'}}"));
	pipeline.push_back(bsoncxx::from_json("{'$group': {'_id': null, 'total': {'$sum': '$value'}}}"));
	BSONObj res = robot_memory->aggregate(pipeline);
	ASSERT_TRUE(contains_pairs(res.getField("result").Array()[0].Obj(), bsoncxx::from_json("{'total':1.2}")));
}
*/

TEST_F(RobotMemoryTest, JavaScriptQuery)
{
	ASSERT_TRUE(robot_memory->insert(bsoncxx::from_json("{'testname':'js-query',a:1,b:2}")));
	ASSERT_TRUE(robot_memory->insert(bsoncxx::from_json("{'testname':'js-query',a:2,b:4}")));
	ASSERT_TRUE(robot_memory->insert(bsoncxx::from_json("{'testname':'js-query',a:3,b:5}")));
	auto qres = robot_memory->query(
	  bsoncxx::from_json("{'testname':'js-query', $where: \"return obj.a * 2 == obj.b\"}"));
	ASSERT_NE(qres.begin(), qres.end());
	ASSERT_NE(qres.begin(), qres.end());
	ASSERT_EQ(qres.begin(), qres.end());
}

TEST_F(RobotMemoryTest, DumpAndResore)
{
	ASSERT_TRUE(robot_memory->drop_collection("robmem.test"));
	ASSERT_TRUE(robot_memory->insert(bsoncxx::from_json("{'testkey':'value',v:1}")));
	ASSERT_TRUE(robot_memory->insert(bsoncxx::from_json("{'testkey':'value',v:2}")));
	ASSERT_TRUE(robot_memory->insert(bsoncxx::from_json("{'testkey':'value',v:3}")));
	ASSERT_TRUE(robot_memory->dump_collection("robmem.test"));
	ASSERT_TRUE(robot_memory->drop_collection("robmem.test"));
	ASSERT_TRUE(robot_memory->restore_collection("robmem.test"));
	auto           qres   = robot_memory->query(bsoncxx::from_json("{'testkey':'value'}"));
	std::list<int> values = {3, 2, 1};
	for (auto __attribute__((unused)) i : values) {
		auto doc = qres.begin();
		ASSERT_NE(doc, qres.end());
		int got = (*doc)["v"].get_int64();
		ASSERT_TRUE(std::find(values.begin(), values.end(), got) != values.end());
		values.remove(got);
	}
	ASSERT_EQ(0, values.size());
	ASSERT_EQ(qres.begin(), qres.end());
}

TEST_F(RobotMemoryTest, EventTriggerLocal)
{
	RobotMemoryCallback *rmc = new RobotMemoryCallback();
	rmc->callback_counter    = 0;
	EventTrigger *trigger1   = robot_memory->register_trigger(bsoncxx::from_json("{test:1}"),
                                                          "robmem.test",
                                                          &RobotMemoryCallback::callback_test,
                                                          rmc);
	EventTrigger *trigger2   = robot_memory->register_trigger(bsoncxx::from_json("{test:2}"),
                                                          "robmem.test",
                                                          &RobotMemoryCallback::callback_test,
                                                          rmc);
	robot_memory->insert(bsoncxx::from_json("{test:0, updateid:55}"), "robmem.test");
	robot_memory->insert(bsoncxx::from_json("{test:1, updateid:42}"), "robmem.test");
	robot_memory->update(bsoncxx::from_json("{updateid:42}"),
	                     bsoncxx::from_json("{test:2, updateid:42}"),
	                     "robmem.test");

	//wait for robot memory to call triggers
	usleep(1000000);

	ASSERT_EQ(2, rmc->callback_counter);

	robot_memory->remove_trigger(trigger1);
	robot_memory->remove_trigger(trigger2);
}

TEST_F(RobotMemoryTest, EventTriggerReplica)
{
	RobotMemoryCallback *rmc = new RobotMemoryCallback();
	rmc->callback_counter    = 0;
	EventTrigger *trigger1   = robot_memory->register_trigger(bsoncxx::from_json("{test:1}"),
                                                          "syncedrobmem.test",
                                                          &RobotMemoryCallback::callback_test,
                                                          rmc);
	EventTrigger *trigger2   = robot_memory->register_trigger(bsoncxx::from_json("{test:2}"),
                                                          "syncedrobmem.test",
                                                          &RobotMemoryCallback::callback_test,
                                                          rmc);

	robot_memory->insert(bsoncxx::from_json("{test:0, updateid:55}"), "syncedrobmem.test");
	robot_memory->insert(bsoncxx::from_json("{test:1, updateid:42}"), "syncedrobmem.test");
	robot_memory->update(bsoncxx::from_json("{updateid:42}"),
	                     bsoncxx::from_json("{test:2, updateid:42}"),
	                     "syncedrobmem.test");

	//wait for robot memory to call triggers
	usleep(1000000);

	ASSERT_EQ(2, rmc->callback_counter);

	robot_memory->remove_trigger(trigger1);
	robot_memory->remove_trigger(trigger2);
}

/**
 * Function for testing if a document contains all key-value pairs of another document
 * @param obj Document that should be tested
 * @param exp Document containing all expected key-value pairs
 * @return Assertion Result
 */
::testing::AssertionResult
RobotMemoryTest::contains_pairs(const bsoncxx::document::view_or_value &obj,
                                const bsoncxx::document::view_or_value &exp)
{
	for (auto expected_element : exp.view()) {
		if (obj.view().find(expected_element.key()) == obj.view().end()
		    || obj.view()[expected_element.key()].get_value() != expected_element.get_value()) {
			return ::testing::AssertionFailure()
			       << bsoncxx::to_json(obj) << " does not include {"
			       << bsoncxx::to_json(expected_element.get_document()) << "}";
		}
	}
	return ::testing::AssertionSuccess();
}

/**
 * Function for testing if a cursor to a query results contains all key-value
 * pairs of another document
 * @param cursor Cursor to a query result that should be tested
 * @param exp Document containing all expected key-value pairs
 * @return Assertion Result
 */
::testing::AssertionResult
RobotMemoryTest::contains_pairs(mongocxx::cursor &                      cursor,
                                const bsoncxx::document::view_or_value &exp)
{
	bsoncxx::builder::basic::document doc;
	for (auto e : cursor) {
		doc.append(bsoncxx::builder::concatenate(e));
	}
	return contains_pairs(doc.extract(), exp);
}

TEST_F(RobotMemoryTest, MapReduceQuery)
{
	//Test sums up the amount of ordered products
	ASSERT_TRUE(
	  robot_memory->insert("{'testname':'mapreduce',order:1, product:1, amount:1}", "robmem.test"));
	ASSERT_TRUE(
	  robot_memory->insert("{'testname':'mapreduce',order:2, product:1, amount:2}", "robmem.test"));
	ASSERT_TRUE(
	  robot_memory->insert("{'testname':'mapreduce',order:3, product:2, amount:3}", "robmem.test"));
	ASSERT_TRUE(
	  robot_memory->insert("{'testname':'mapreduce',order:4, product:2, amount:4}", "robmem.test"));
	ASSERT_TRUE(robot_memory->insert("{'testname':'not mapreduce',order:1, product:1, amount:2}"));
	auto res = robot_memory->mapreduce(bsoncxx::from_json("{'testname':'mapreduce'}"),
	                                   "robmem.test",
	                                   "function() { emit( this.product, this.amount);}",
	                                   "function(key, values) { return Array.sum( values )}");
	ASSERT_TRUE(contains_pairs(
	  res.view(),
	  bsoncxx::from_json("{ok: 1.0, results:[{_id:1.0, value:3.0}, {_id:2.0, value: 7.0}]}")));
}

TEST_F(RobotMemoryTest, AggregationQuery)
{
	//Test finds maximum with aggregation
	ASSERT_TRUE(robot_memory->insert(bsoncxx::from_json("{'testname':'agg', v:1}"), "robmem.test"));
	ASSERT_TRUE(robot_memory->insert(bsoncxx::from_json("{'testname':'agg', v:333}"), "robmem.test"));
	ASSERT_TRUE(robot_memory->insert(bsoncxx::from_json("{'testname':'agg', v:-20}"), "robmem.test"));
	ASSERT_TRUE(
	  robot_memory->insert(bsoncxx::from_json("{'testname':'not agg', v:666}"), "robmem.test"));
	auto qres = robot_memory->aggregate(
	  bsoncxx::from_json("[{$match:{testname:'agg'}}, {$group: {_id:null, max:{$max: '$v'}}}]"),
	  "robmem.test");
	ASSERT_TRUE(contains_pairs(qres, bsoncxx::from_json("{max: 333}")));
}

TEST_F(RobotMemoryTest, ComputableRegisterRemove)
{
	TestComputable *tc   = new TestComputable();
	Computable *    comp = robot_memory->register_computable(bsoncxx::from_json("{somekey:'value'}"),
                                                       "robmem.test",
                                                       &TestComputable::compute,
                                                       tc);
	robot_memory->remove_computable(comp);
}

TEST_F(RobotMemoryTest, ComputableCall)
{
	TestComputable *tc   = new TestComputable();
	Computable *    comp = robot_memory->register_computable(bsoncxx::from_json("{computed:true}"),
                                                       "robmem.test",
                                                       &TestComputable::compute,
                                                       tc);
	auto            qres = robot_memory->query(bsoncxx::from_json("{computed:true}"), "robmem.test");
	ASSERT_TRUE(contains_pairs(qres, bsoncxx::from_json("{result:'this is computed'}")));
	robot_memory->remove_computable(comp);
}

TEST_F(RobotMemoryTest, ComputableCallAddition)
{
	TestComputable *tc = new TestComputable();
	Computable *    comp =
	  robot_memory->register_computable(bsoncxx::from_json(
	                                      "{compute:'sum',x:{$exists:true},y:{$exists:true}}"),
	                                    "robmem.test",
	                                    &TestComputable::compute_sum,
	                                    tc);
	auto qres = robot_memory->query(bsoncxx::from_json("{compute:'sum',x:15,y:4}"), "robmem.test");
	ASSERT_TRUE(contains_pairs(qres, bsoncxx::from_json("{sum:19}")));
	robot_memory->remove_computable(comp);
}

TEST_F(RobotMemoryTest, ComputableMultiple)
{
	TestComputable *tc = new TestComputable();
	Computable *comp   = robot_memory->register_computable(bsoncxx::from_json("{compute:'multiple'}"),
                                                       "robmem.test",
                                                       &TestComputable::compute_multiple,
                                                       tc);
	auto        qres = robot_memory->query(bsoncxx::from_json("{compute:'multiple'}"), "robmem.test");
	std::list<int> values = {3, 2, 1};
	for (auto __attribute__((unused)) i : values) {
		auto doc = qres.begin();
		ASSERT_NE(doc, qres.end());
		int got = (*doc)["count"].get_int32();
		ASSERT_TRUE(std::find(values.begin(), values.end(), got) != values.end());
		values.remove(got);
	}
	ASSERT_EQ(0, values.size());
	ASSERT_EQ(qres.begin(), qres.end());
	robot_memory->remove_computable(comp);
}

TEST_F(RobotMemoryTest, BlackboardComputable)
{
	Position3DInterface *if3d = blackboard->open_for_writing<Position3DInterface>("test1");
	if3d->set_frame("test_frame");
	if3d->set_translation(0, 1.1);
	if3d->set_translation(1, 2.2);
	if3d->set_translation(2, 3.3);
	if3d->write();
	auto qres =
	  robot_memory->query(bsoncxx::from_json("{interface:'Position3DInterface',id:'test1'}"),
	                      "robmem.blackboard");
	ASSERT_TRUE(
	  contains_pairs(qres,
	                 bsoncxx::from_json("{interface:'Position3DInterface',id:'test1',frame:'test_"
	                                    "frame',translation:[1.1, 2.2, 3.3]}")));
	blackboard->close(if3d);
}

TEST_F(RobotMemoryTest, BlackboardComputableMultiple)
{
	Position3DInterface *if3d = blackboard->open_for_writing<Position3DInterface>("test");
	if3d->set_frame("test_frame");
	if3d->write();
	Position3DInterface *if3d_2 = blackboard->open_for_writing<Position3DInterface>("test_2");
	if3d_2->set_frame("test_frame");
	if3d_2->write();
	auto qres = robot_memory->query(bsoncxx::from_json("{interface:'Position3DInterface',id:'test'}"),
	                                "robmem.blackboard");
	ASSERT_TRUE(contains_pairs(qres, bsoncxx::from_json("{interface:'Position3DInterface'}")));
	blackboard->close(if3d);
	blackboard->close(if3d_2);
}

TEST_F(RobotMemoryTest, TransformComputable)
{
	robot_memory->insert(bsoncxx::from_json(
	                       "{name:'test pos', frame:'cam_tag', translation:[0.0, 0.0, 0.0], "
	                       "rotation:[0.0, 0.0, 0.0, 1.0]}"),
	                     "robmem.test");
	auto qres =
	  robot_memory->query(bsoncxx::from_json("{name:'test pos', frame:'base_link', allow_tf:true}"),
	                      "robmem.test");
	auto res = *(qres.begin());
	ASSERT_EQ("base_link", res["frame"].get_utf8().value.to_string());
	bsoncxx::array::view trans_view{res["translation"].get_array()};
	ASSERT_TRUE(fabs(0.1 - trans_view[0].get_double()) < 0.001);
	bsoncxx::array::view rot_view{res["rotation"].get_array()};
	ASSERT_TRUE(fabs(-0.5 - rot_view[0].get_double()) < 0.001);
}
