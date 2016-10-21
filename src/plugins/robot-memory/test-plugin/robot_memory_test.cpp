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
#include <list>
#include <algorithm>

//init static variable
RobotMemory* RobotMemoryTestEnvironment::robot_memory = NULL;

using namespace fawkes;
using namespace mongo;

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

TEST_F(RobotMemoryTest, AggregationQuery)
{
  //TODO: implement
}

TEST_F(RobotMemoryTest, MapReduceQuery)
{
  //TODO: implement
}

TEST_F(RobotMemoryTest, JavaScriptQuery)
{
  //TODO: implement
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
  usleep(100000);

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
  usleep(100000);

  ASSERT_EQ(2, rmc->callback_counter);

  robot_memory->remove_trigger(trigger1);
  robot_memory->remove_trigger(trigger2);
}

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


TEST_F(RobotMemoryTest, ComputableRegisterRemove)
{
  TestComputable* tc = new TestComputable();
  Computable* comp = robot_memory->register_computable(fromjson("{somekey:'value'}"), &TestComputable::compute, tc);
  robot_memory->remove_computable(comp);
}
