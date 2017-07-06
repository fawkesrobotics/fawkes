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
#include <list>
#include <algorithm>
#include <math.h>

using namespace fawkes;
using namespace mongo;

//init static variable
RobotMemory* RobotMemoryTestEnvironment::robot_memory = NULL;
BlackBoard* RobotMemoryTestEnvironment::blackboard = NULL;

/**
 * Setup for each test
 */
void RobotMemoryTest::SetUp()
{
  robot_memory = RobotMemoryTestEnvironment::robot_memory;
  blackboard = RobotMemoryTestEnvironment::blackboard;
}

TEST_F(RobotMemoryTest, TestsWorking)
{
  ASSERT_EQ(1, 3-2);
}

TEST_F(RobotMemoryTest, AspectAvailable)
{
  ASSERT_FALSE(robot_memory==NULL);
}

TEST_F(RobotMemoryTest, QueryResultEmpty)
{
  ASSERT_TRUE(robot_memory->insert("{'insert':'something to have the namespace'}"));
  QResCursor qres = robot_memory->query("{somekey:'should_not_exist'}");
  ASSERT_FALSE(qres->more());
}

TEST_F(RobotMemoryTest, StoreAndQuery)
{
  ASSERT_TRUE(robot_memory->insert("{'testkey':'value'}"));
  QResCursor qres = robot_memory->query("{'testkey':'value'}");
  ASSERT_TRUE(qres->more());
  ASSERT_TRUE(contains_pairs(qres->next(), fromjson("{'testkey':'value'}")));
}

TEST_F(RobotMemoryTest, StoreAndQueryOtherCollection)
{
  ASSERT_TRUE(robot_memory->insert("{'testkey':'value'}", "robmem.othercollection"));
  QResCursor qres = robot_memory->query("{'testkey':'value'}", "robmem.othercollection");
  ASSERT_TRUE(qres->more());
  ASSERT_TRUE(contains_pairs(qres->next(), fromjson("{'testkey':'value'}")));
}

TEST_F(RobotMemoryTest, StoreUpdateQuery)
{
  ASSERT_TRUE(robot_memory->insert("{'inserting':'something',as:0.5}"));
  ASSERT_TRUE(robot_memory->update("{'inserting':'something',as:0.5}",
      "{'updated':'something',as:3.0,extra:true}"));
  QResCursor qres = robot_memory->query("{'updated':'something'}");
  ASSERT_TRUE(qres->more());
  ASSERT_TRUE(contains_pairs(qres->next(), fromjson(
      "{'updated':'something',as:3.0,extra:true}")));
}

TEST_F(RobotMemoryTest, StoreRemoveQuery)
{
  ASSERT_TRUE(robot_memory->insert("{to_be:'removed'}"));
  ASSERT_TRUE(robot_memory->remove("{to_be:'removed'}"));
  QResCursor qres = robot_memory->query("{to_be:'removed'}");
  ASSERT_FALSE(qres->more());
}

TEST_F(RobotMemoryTest, Upsert)
{
  ASSERT_TRUE(robot_memory->update("{upsert:'not existing'}", "{upsert:'should not exist'}", "", false));
  QResCursor qres = robot_memory->query("{upsert:'should not exist'}");
  ASSERT_FALSE(qres->more());
  ASSERT_TRUE(robot_memory->update("{upsert:'not existing'}", "{upsert:'should exist'}", "", true));
  qres = robot_memory->query("{upsert:'should exist'}");
  ASSERT_TRUE(qres->more());
}

TEST_F(RobotMemoryTest, QueryInvalid)
{
  ASSERT_THROW(robot_memory->query("{key-:+'not existing'}"), mongo::DBException);
}

TEST_F(RobotMemoryTest, InsertInvalidCaught)
{
  ASSERT_THROW(robot_memory->insert("{'testkey'::'value'}"), mongo::DBException);
  ASSERT_THROW(robot_memory->insert("warbagarbl"), mongo::DBException);
}

TEST_F(RobotMemoryTest, UpdateInvalidCaught)
{
  ASSERT_THROW(robot_memory->update("{'testkey':'good'}",
      "{'bad':1.2.3}"), mongo::DBException);
  ASSERT_THROW(robot_memory->update("{([})]", "{'key':4}"), mongo::DBException);
}

TEST_F(RobotMemoryTest, RemoveInvalidCaught)
{
  ASSERT_THROW(robot_memory->remove("{____:4.56!}"), mongo::DBException);
  ASSERT_THROW(robot_memory->remove("{([})]"), mongo::DBException);
}

TEST_F(RobotMemoryTest, AggregationSumQuery)
{
  ASSERT_TRUE(robot_memory->insert("{'agg':'summand',value:0.5}"));
  ASSERT_TRUE(robot_memory->insert("{'agg':'summand',value:0.7}"));
  ASSERT_TRUE(robot_memory->insert("{'agg':'not-summand',value:0.9}"));

  std::vector<BSONObj> pipeline;
  pipeline.push_back(fromjson("{'$match': {'agg':'summand'}}"));
  pipeline.push_back(fromjson("{'$group': {'_id': null, 'total': {'$sum': '$value'}}}"));
  BSONObj res = robot_memory->aggregate(pipeline);
  ASSERT_TRUE(contains_pairs(res.getField("result").Array()[0].Obj(), fromjson("{'total':1.2}")));
}

TEST_F(RobotMemoryTest, JavaScriptQuery)
{
  ASSERT_TRUE(robot_memory->insert("{'testname':'js-query',a:1,b:2}"));
  ASSERT_TRUE(robot_memory->insert("{'testname':'js-query',a:2,b:4}"));
  ASSERT_TRUE(robot_memory->insert("{'testname':'js-query',a:3,b:5}"));
  QResCursor qres = robot_memory->query("{'testname':'js-query', $where: \"return obj.a * 2 == obj.b\"}");
  ASSERT_TRUE(qres->more());
  qres->next();
  ASSERT_TRUE(qres->more());
  qres->next();
  ASSERT_FALSE(qres->more());
}

TEST_F(RobotMemoryTest, DumpAndResore)
{
  ASSERT_TRUE(robot_memory->drop_collection("robmem.test"));
  ASSERT_TRUE(robot_memory->insert("{'testkey':'value',v:1}"));
  ASSERT_TRUE(robot_memory->insert("{'testkey':'value',v:2}"));
  ASSERT_TRUE(robot_memory->insert("{'testkey':'value',v:3}"));
  ASSERT_TRUE(robot_memory->dump_collection("robmem.test"));
  ASSERT_TRUE(robot_memory->drop_collection("robmem.test"));
  ASSERT_TRUE(robot_memory->restore_collection("robmem.test"));
  QResCursor qres = robot_memory->query("{'testkey':'value'}");
  std::list<int> values = {3, 2, 1};
  ASSERT_TRUE(qres->more());
  int got = qres->next().getField("v").Int();
  ASSERT_TRUE(std::find(values.begin(), values.end(), got) != values.end());
  values.remove(got);
  ASSERT_TRUE(qres->more());
  got = qres->next().getField("v").Int();
  ASSERT_TRUE(std::find(values.begin(), values.end(), got) != values.end());
  values.remove(got);
  ASSERT_TRUE(qres->more());
  got = qres->next().getField("v").Int();
  ASSERT_TRUE(std::find(values.begin(), values.end(), got) != values.end());
  values.remove(got);
  ASSERT_EQ(0, values.size());
  ASSERT_FALSE(qres->more());
}

TEST_F(RobotMemoryTest, EventTriggerLocal)
{
  RobotMemoryCallback* rmc = new RobotMemoryCallback();
  rmc->callback_counter = 0;
  EventTrigger* trigger1 = robot_memory->register_trigger(fromjson("{test:1}"),
      "robmem.test", &RobotMemoryCallback::callback_test, rmc);
  EventTrigger* trigger2 = robot_memory->register_trigger(fromjson("{test:2}"),
      "robmem.test", &RobotMemoryCallback::callback_test, rmc);
  robot_memory->insert(fromjson("{test:0, updateid:55}"), "robmem.test");
  robot_memory->insert(fromjson("{test:1, updateid:42}"), "robmem.test");
  robot_memory->update(fromjson("{updateid:42}"), fromjson("{test:2, updateid:42}"), "robmem.test");

  //wait for robot memory to call triggers
  usleep(1000000);

  ASSERT_EQ(2, rmc->callback_counter);

  robot_memory->remove_trigger(trigger1);
  robot_memory->remove_trigger(trigger2);
}

TEST_F(RobotMemoryTest, EventTriggerReplica)
{
  RobotMemoryCallback* rmc = new RobotMemoryCallback();
  rmc->callback_counter = 0;
  EventTrigger* trigger1 = robot_memory->register_trigger(fromjson("{test:1}"),
      "syncedrobmem.test", &RobotMemoryCallback::callback_test, rmc);
  EventTrigger* trigger2 = robot_memory->register_trigger(fromjson("{test:2}"),
      "syncedrobmem.test", &RobotMemoryCallback::callback_test, rmc);

  robot_memory->insert(fromjson("{test:0, updateid:55}"), "syncedrobmem.test");
  robot_memory->insert(fromjson("{test:1, updateid:42}"), "syncedrobmem.test");
  robot_memory->update(fromjson("{updateid:42}"), fromjson("{test:2, updateid:42}"), "syncedrobmem.test");

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
::testing::AssertionResult RobotMemoryTest::contains_pairs(BSONObj obj, BSONObj exp)
{
  for(BSONObjIterator it = exp.begin(); it.more();)
  {
    BSONElement kvpair = it.next();
    //printf("checking %s\n", kvpair.toString().c_str());
    if(!obj.hasElement(kvpair.fieldName())
        || obj.getField(kvpair.fieldName()) != kvpair)
    {
      return ::testing::AssertionFailure() << obj.toString()
          << " does not include {" << kvpair.toString() << "}";
    }
  }
  return ::testing::AssertionSuccess();
}

TEST_F(RobotMemoryTest, MapReduceQuery)
{
  //Test sums up the amount of ordered products
  ASSERT_TRUE(robot_memory->insert("{'testname':'mapreduce',order:1, product:1, amount:1}", "robmem.test"));
  ASSERT_TRUE(robot_memory->insert("{'testname':'mapreduce',order:2, product:1, amount:2}", "robmem.test"));
  ASSERT_TRUE(robot_memory->insert("{'testname':'mapreduce',order:3, product:2, amount:3}", "robmem.test"));
  ASSERT_TRUE(robot_memory->insert("{'testname':'mapreduce',order:4, product:2, amount:4}", "robmem.test"));
  ASSERT_TRUE(robot_memory->insert("{'testname':'not mapreduce',order:1, product:1, amount:2}"));
  BSONObj res = robot_memory->mapreduce(fromjson("{'testname':'mapreduce'}"), "robmem.test",
      "function() { emit( this.product, this.amount);}",
      "function(key, values) { return Array.sum( values )}");
  ASSERT_TRUE(contains_pairs(res, fromjson("{ok: 1.0, results:[{_id:1.0, value:3.0}, {_id:2.0, value: 7.0}]}")));
}

TEST_F(RobotMemoryTest, AggregationQuery)
{
  //Test finds maximum with aggregation
  ASSERT_TRUE(robot_memory->insert("{'testname':'agg', v:1}", "robmem.test"));
  ASSERT_TRUE(robot_memory->insert("{'testname':'agg', v:333}", "robmem.test"));
  ASSERT_TRUE(robot_memory->insert("{'testname':'agg', v:-20}", "robmem.test"));
  ASSERT_TRUE(robot_memory->insert("{'testname':'not agg', v:666}", "robmem.test"));
  QResCursor qres = robot_memory->aggregate(
      fromjson("[{$match:{testname:'agg'}}, {$group: {_id:null, max:{$max: '$v'}}}]"), "robmem.test");
  ASSERT_TRUE(qres->more());
  ASSERT_TRUE(contains_pairs(qres->next(), fromjson("{max: 333}")));
}


TEST_F(RobotMemoryTest, ComputableRegisterRemove)
{
  TestComputable* tc = new TestComputable();
  Computable* comp = robot_memory->register_computable(fromjson("{somekey:'value'}"), "robmem.test", &TestComputable::compute, tc);
  robot_memory->remove_computable(comp);
}


TEST_F(RobotMemoryTest, ComputableCall)
{
  TestComputable* tc = new TestComputable();
  Computable* comp = robot_memory->register_computable(fromjson("{computed:true}"), "robmem.test", &TestComputable::compute, tc);
  QResCursor qres = robot_memory->query(fromjson("{computed:true}"), "robmem.test");
  ASSERT_TRUE(qres->more());
  ASSERT_TRUE(contains_pairs(qres->next(), fromjson("{result:'this is computed'}")));
  robot_memory->remove_computable(comp);
}


TEST_F(RobotMemoryTest, ComputableCallAddition)
{
  TestComputable* tc = new TestComputable();
  Computable* comp = robot_memory->register_computable(fromjson(
      "{compute:'sum',x:{$exists:true},y:{$exists:true}}"), "robmem.test", &TestComputable::compute_sum, tc);
  QResCursor qres = robot_memory->query(fromjson("{compute:'sum',x:15,y:4}"), "robmem.test");
  ASSERT_TRUE(qres->more());
  ASSERT_TRUE(contains_pairs(qres->next(), fromjson("{sum:19}")));
  robot_memory->remove_computable(comp);
}


TEST_F(RobotMemoryTest, ComputableMultiple)
{
  TestComputable* tc = new TestComputable();
  Computable* comp = robot_memory->register_computable(fromjson(
      "{compute:'multiple'}"), "robmem.test", &TestComputable::compute_multiple, tc);
  QResCursor qres = robot_memory->query(fromjson("{compute:'multiple'}"), "robmem.test");
  std::list<int> values = {3, 2, 1};
  ASSERT_TRUE(qres->more());
  int got = qres->next().getField("count").Int();
  ASSERT_TRUE(std::find(values.begin(), values.end(), got) != values.end());
  values.remove(got);
  ASSERT_TRUE(qres->more());
  got = qres->next().getField("count").Int();
  ASSERT_TRUE(std::find(values.begin(), values.end(), got) != values.end());
  values.remove(got);
  ASSERT_TRUE(qres->more());
  got = qres->next().getField("count").Int();
  ASSERT_TRUE(std::find(values.begin(), values.end(), got) != values.end());
  values.remove(got);
  ASSERT_EQ(0, values.size());
  ASSERT_FALSE(qres->more());
  robot_memory->remove_computable(comp);
}


TEST_F(RobotMemoryTest, BlackboardComputable)
{
  Position3DInterface* if3d = blackboard->open_for_writing<Position3DInterface>("test1");
  if3d->set_frame("test_frame");
  if3d->set_translation(0, 1.1);
  if3d->set_translation(1, 2.2);
  if3d->set_translation(2, 3.3);
  if3d->write();
  QResCursor qres = robot_memory->query(fromjson("{interface:'Position3DInterface',id:'test1'}"), "robmem.blackboard");
  ASSERT_TRUE(qres->more());
  ASSERT_TRUE(contains_pairs(qres->next(), fromjson(
      "{interface:'Position3DInterface',id:'test1',frame:'test_frame',translation:[1.1, 2.2, 3.3]}")));
  blackboard->close(if3d);
}


TEST_F(RobotMemoryTest, BlackboardComputableMultiple)
{
  Position3DInterface* if3d = blackboard->open_for_writing<Position3DInterface>("test");
  if3d->set_frame("test_frame");
  if3d->write();
  Position3DInterface* if3d_2 = blackboard->open_for_writing<Position3DInterface>("test_2");
  if3d_2->set_frame("test_frame");
  if3d_2->write();
  QResCursor qres = robot_memory->query(fromjson("{interface:'Position3DInterface',id:'test'}"), "robmem.blackboard");
  ASSERT_TRUE(qres->more());
  ASSERT_TRUE(contains_pairs(qres->next(), fromjson("{interface:'Position3DInterface'}")));
  blackboard->close(if3d);
  blackboard->close(if3d_2);
}


TEST_F(RobotMemoryTest, TransformComputable)
{
  robot_memory->insert("{name:'test pos', frame:'cam_tag', translation:[0.0, 0.0, 0.0], rotation:[0.0, 0.0, 0.0, 1.0]}", "robmem.test");
  QResCursor qres = robot_memory->query(fromjson("{name:'test pos', frame:'base_link', allow_tf:true}"), "robmem.test");
  ASSERT_TRUE(qres->more());
  BSONObj res = qres->next();
  ASSERT_EQ("base_link", res.getField("frame").String());
  ASSERT_TRUE(fabs(0.1 - res.getField("translation").Array()[0].Double()) < 0.001);
  ASSERT_TRUE(fabs(-0.5 - res.getField("rotation").Array()[0].Double()) < 0.001);
}
