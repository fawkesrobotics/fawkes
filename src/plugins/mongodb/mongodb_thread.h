
/***************************************************************************
 *  mongodb_thread.h - MongoDB thread
 *
 *  Created: Sun Dec 05 23:25:20 2010
 *  Copyright  2006-2015  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_MONGODB_MONGODB_THREAD_H_
#define __PLUGINS_MONGODB_MONGODB_THREAD_H_

#include <plugins/mongodb/aspect/mongodb_conncreator.h>
#include <plugins/mongodb/aspect/mongodb_inifin.h>
#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/clock.h>
#include <aspect/aspect_provider.h>
#include <aspect/thread_producer.h>

// from MongoDB
#include <mongo/client/dbclient.h>

#include <vector>
#include <list>
#include <string>
#include <memory>

class MongoDBClientConfig;
class MongoDBInstanceConfig;
class MongoDBReplicaSetConfig;

class MongoDBThread
: public fawkes::Thread,
	public fawkes::LoggingAspect,
	public fawkes::ConfigurableAspect,
	public fawkes::ClockAspect,
	public fawkes::AspectProviderAspect,
	public fawkes::ThreadProducerAspect,
	public fawkes::MongoDBConnCreator
{
 public:
	MongoDBThread();
	virtual ~MongoDBThread();

	virtual void init();
	virtual void loop();
	virtual void finalize();

	virtual mongo::DBClientBase *  create_client(const std::string &config_name = "");
	virtual void delete_client(mongo::DBClientBase *client);

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
	void init_client_configs();
	void init_instance_configs();
	void init_replicaset_configs();

 private:
	std::map<std::string, std::shared_ptr<MongoDBClientConfig>> client_configs_;
	std::map<std::string, std::shared_ptr<MongoDBInstanceConfig>> instance_configs_;
	std::map<std::string, std::shared_ptr<MongoDBReplicaSetConfig>> replicaset_configs_;

	fawkes::MongoDBAspectIniFin     mongodb_aspect_inifin_;
};

#endif
