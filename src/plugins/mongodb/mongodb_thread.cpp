
/***************************************************************************
 *  mongodb_thread.cpp - MongoDB Thread
 *
 *  Created: Sun Dec 05 23:32:13 2010
 *  Copyright  2006-2010  Tim Niemueller [www.niemueller.de]
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

#include "mongodb_thread.h"

#include <mongo/client/syncclusterconnection.h>

using namespace mongo;
using namespace fawkes;

/** @class MongoDBThread "mongodb_thread.h"
 * MongoDB Thread.
 * This thread maintains an active connection to MongoDB and provides an
 * aspect to access MongoDB to make it convenient for other threads to use
 * MongoDB.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
MongoDBThread::MongoDBThread()
  : Thread("MongoDBThread", Thread::OPMODE_WAITFORWAKEUP)
{
}


/** Destructor. */
MongoDBThread::~MongoDBThread()
{
}


void
MongoDBThread::init()
{
  __mode = CONNECTION;
  logger->log_warn(name(), "INIT");
}


void
MongoDBThread::finalize()
{
  logger->log_warn(name(), "FINALIZE");
}


void
MongoDBThread::loop()
{
  logger->log_debug(name(), "loop");
}


void
MongoDBThread::add_hostport(const char *hostport)
{
  switch (__mode) {
  case REPLICA_SET:
    __replicaset_hostports.push_back(HostAndPort(hostport));
    break;
  case SYNC_CLUSTER:
    __synccluster_hostports.push_back(HostAndPort(hostport));
    break;
  default:
    throw Exception("Connection mode accepts only one host and port");
    break;
  }
}


mongo::DBClientBase *
MongoDBThread::create_client(const char *dbname, const char *user,
			     const char *clearpwd)
{
  mongo::DBClientBase *client;
  std::string errmsg;

  switch (__mode) {
  case REPLICA_SET:
    {
      std::string noname = "";
      DBClientReplicaSet *repset =
	new DBClientReplicaSet(noname, __replicaset_hostports);
      client = repset;
      if (! repset->connect())  throw Exception("Cannot connect to database");
      if (dbname && user && clearpwd) {
	if (! repset->auth(dbname, user, clearpwd, errmsg, false)) {
	  throw Exception("Authenticating for %s as %s failed: %s",
			  dbname, user, errmsg.c_str());
	}
      }
    }
    break;

  case SYNC_CLUSTER:
    {
      SyncClusterConnection *synccluster =
	new SyncClusterConnection(__synccluster_hostports);
      client = synccluster;
      if (! synccluster->prepare(errmsg)) {
	throw Exception("Failed to prepare sync cluster connection: %s",
			errmsg.c_str());
      }
    }
    break;

  default:
    {
      DBClientConnection *clconn = 
	new DBClientConnection(/* auto reconnect */ true);
      client = clconn;
      std::string errmsg;
      if (! clconn->connect(__conn_hostport, errmsg)) {
	throw Exception("Could not connect to MongoDB at %s: %s",
			__conn_hostport.toString().c_str(), errmsg.c_str());
      }
      if (dbname && user && clearpwd) {
	if (! clconn->auth(dbname, user, clearpwd, errmsg, false)) {
	  throw Exception("Authenticating for %s as %s failed: %s",
			  dbname, user, errmsg.c_str());
	}
      }
    }
    break;
  }

  return client;
}

void
MongoDBThread::delete_client(mongo::DBClientBase *client)
{
  delete client;
}
