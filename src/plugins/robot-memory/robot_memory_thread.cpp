
/***************************************************************************
 *  robot_memory_thread.cpp - Robot Memory thread
 *
 *  Created: Sun May 01 13:41:45 2016
 *  Copyright  2016 Frederik Zwilling
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

#include "robot_memory_thread.h"
#include "interfaces/RobotMemoryInterface.h"
#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include <memory>

// from MongoDB
#include <mongo/client/dbclient.h>

using namespace mongo;
using namespace fawkes;

/** @class RobotMemoryThread "robot_memory_thread.h"
 * Thread that provides a robot memory with MongoDB
 * @author Frederik Zwilling
 */

/** Constructor. */
RobotMemoryThread::RobotMemoryThread()
	: Thread("RobotMemoryThread", Thread::OPMODE_WAITFORWAKEUP),
	  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_WORLDSTATE)
{
  __mutex = new Mutex();
}


/** Destructor. */
RobotMemoryThread::~RobotMemoryThread()
{
  delete __mutex;
}


void
RobotMemoryThread::init()
{
	logger->log_info(name(), "Started RobotMemory");
	__collection = "fawkes.msglog";
  try {
    __collection = config->get_string("/plugins/mongodb/test-collection");
  } catch (Exception &e) {}

  __rm_if = blackboard->open_for_writing<RobotMemoryInterface>(config->get_string("/plugins/robot-memory/interface-name").c_str());
  __rm_if->set_error("");
  __rm_if->set_result("");
  
  __rm_if->write();
}


void
RobotMemoryThread::finalize()
{
}


void
RobotMemoryThread::loop()
{
	// process interface messages
  while (! __rm_if->msgq_empty() ) {
    if (__rm_if->msgq_first_is<RobotMemoryInterface::QueryMessage>()) {
	    RobotMemoryInterface::QueryMessage* msg = (RobotMemoryInterface::QueryMessage*) __rm_if->msgq_first();
	    exec_query(msg->query());
    } else if (__rm_if->msgq_first_is<RobotMemoryInterface::InsertMessage>()) {
	    RobotMemoryInterface::InsertMessage* msg = (RobotMemoryInterface::InsertMessage*) __rm_if->msgq_first();
	    exec_insert(msg->insert());
    } else if (__rm_if->msgq_first_is<RobotMemoryInterface::UpdateMessage>()) {
	    RobotMemoryInterface::UpdateMessage* msg = (RobotMemoryInterface::UpdateMessage*) __rm_if->msgq_first();
	    exec_update(msg->query(), msg->update());
    } else if (__rm_if->msgq_first_is<RobotMemoryInterface::RemoveMessage>()) {
	    RobotMemoryInterface::RemoveMessage* msg = (RobotMemoryInterface::RemoveMessage*) __rm_if->msgq_first();
	    exec_remove(msg->query());
    } else {
      logger->log_warn(name(), "Unknown message received");
    }

    __rm_if->msgq_pop();
  }
}

void RobotMemoryThread::exec_query(std::string query_string)
{
	logger->log_info(name(), "Executing Query: %s", query_string.c_str());

	//only one query at a time
	MutexLocker lock(__mutex);

	//get query from string
	Query query;
	try{
	  query = Query(query_string);
	} catch (DBException &e) {
		logger->log_error(name(), "Can't parse query_string '%s'\n Exception: %s",
		                  query_string.c_str(), e.toString().c_str());
		__rm_if->set_error((std::string("Can't parse query_string ") +  query_string
		                    + "\nException: " + e.toString()).c_str());
		__rm_if->write();
		return;
	}

	//introspect query
	log(query, "executing query:");
	
	//actually execute query
	std::unique_ptr<DBClientCursor> cursor;
	try{
	  cursor = mongodb_client->query(dyn_collection, query);
	} catch (DBException &e) {
		logger->log_error(name(), "Error for query %s\n Exception: %s",
		                  query_string.c_str(), e.toString().c_str());
		__rm_if->set_error((std::string("Query error for ") +  query_string
		                    + "\nException: " + e.toString()).c_str());
		__rm_if->write();
		return;
	}

	if(cursor->more()){
		BSONObj res = cursor->next();
		logger->log_info(name(), "Query One result:\n%s", res.toString().c_str());
		__rm_if->set_result(res.toString().c_str());
		__rm_if->write();
	}
	else {
		logger->log_info(name(), "Query result empty");
	}
}

void RobotMemoryThread::exec_insert(std::string insert_string)
{
	logger->log_info(name(), "Executing Query: %s", insert_string.c_str());

	//only one query at a time
	MutexLocker lock(__mutex);

	//get query from string
  BSONObj obj;
	try{
		obj = fromjson(insert_string);
	} catch (DBException &e) {
		logger->log_error(name(), "Can't parse insert_string '%s'\n Exception: %s",
		                  insert_string.c_str(), e.toString().c_str());
		__rm_if->set_error((std::string("Can't parse insert_string ") +  insert_string
		                    + "\nException: " + e.toString()).c_str());
		__rm_if->write();
		return;
	}

	log(obj, "Inserting:");
	
	//actually execute insert
	try{
	  mongodb_client->insert(__collection, obj);
	} catch (DBException &e) {
		logger->log_error(name(), "Error for insert %s\n Exception: %s",
		                  insert_string.c_str(), e.toString().c_str());
		__rm_if->set_error((std::string("Query error for ") +  insert_string
		                    + "\nException: " + e.toString()).c_str());
		__rm_if->write();
		return;
	}

	__rm_if->set_result("insert successful");
	__rm_if->write();
}

void RobotMemoryThread::exec_update(std::string query_string, std::string update_string)
{
	logger->log_info(name(), "Executing Update %s for query %s",
	                 update_string.c_str(), query_string.c_str());

	//only one query at a time
	MutexLocker lock(__mutex);

	//get query from string
	Query query;
	try{
	  query = Query(query_string);
	} catch (DBException &e) {
		logger->log_error(name(), "Can't parse query_string '%s'\n Exception: %s",
		                  query_string.c_str(), e.toString().c_str());
		__rm_if->set_error((std::string("Can't parse query_string ") +  query_string
		                    + "\nException: " + e.toString()).c_str());
		__rm_if->write();
		return;
	}
	BSONObj update;
	try{
		update = fromjson(update_string);
	} catch (DBException &e) {
		logger->log_error(name(), "Can't parse update_string '%s'\n Exception: %s",
		                  update_string.c_str(), e.toString().c_str());
		__rm_if->set_error((std::string("Can't parse update_string ") +  update_string
		                    + "\nException: " + e.toString()).c_str());
		__rm_if->write();
		return;
	}

	log(query, "Updating documents for query:");
	log(update, "Updating with:");
	
	//actually execute update
	try{
		mongodb_client->update(__collection, query, update);
	} catch (DBException &e) {
		logger->log_error(name(), "Error for update %s for query %s\n Exception: %s",
		                  update_string.c_str(), query_string.c_str(), e.toString().c_str());
		__rm_if->set_error((std::string("Query error for ") +  query_string + " and update "
		                    + update_string + "\nException: " + e.toString()).c_str());
		__rm_if->write();
		return;
	}

	__rm_if->set_result("update successful");
	__rm_if->write();
}

void RobotMemoryThread::exec_remove(std::string query_string)
{
	logger->log_info(name(), "Executing Remove: %s", query_string.c_str());

	//only one query at a time
	MutexLocker lock(__mutex);

	//get query from string
	Query query;
	try{
	  query = Query(query_string);
	} catch (DBException &e) {
		logger->log_error(name(), "Can't parse query_string '%s'\n Exception: %s",
		                  query_string.c_str(), e.toString().c_str());
		__rm_if->set_error((std::string("Can't parse query_string ") +  query_string
		                    + "\nException: " + e.toString()).c_str());
		__rm_if->write();
		return;
	}

	//introspect
	log(query, "Removing documents for query:");
	
	//actually execute remove
	try{
	  mongodb_client->remove(__collection, query);
	} catch (DBException &e) {
		logger->log_error(name(), "Error for query %s\n Exception: %s",
		                  query_string.c_str(), e.toString().c_str());
		__rm_if->set_error((std::string("Query error for ") +  query_string
		                    + "\nException: " + e.toString()).c_str());
		__rm_if->write();
		return;
	}

	__rm_if->set_result("remove successful");
	__rm_if->write();
}

void
RobotMemoryThread::log(Query query, std::string what)
{
	std::string output = what
		+ "\nFilter: " + query.getFilter().toString()
		+ "\nModifiers: " + query.getModifiers().toString()
		+ "\nSort: " + query.getSort().toString()
		+ "\nHint: " + query.getHint().toString()
		+ "\nReadPref: " + query.getReadPref().toString();
		
	logger->log_info(name(), "%s", output.c_str());
}

void
RobotMemoryThread::log(BSONObj obj, std::string what)
{
	std::string output = what
		+ "\nObject: " + obj.toString();
		
	logger->log_info(name(), "%s", output.c_str());
}

