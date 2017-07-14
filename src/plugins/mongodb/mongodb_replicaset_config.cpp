
/***************************************************************************
 *  mongodb_replicaset_config.cpp - MongoDB replica set configuration
 *
 *  Created: Thu Jul 13 10:25:19 2017
 *  Copyright  2006-2017  Tim Niemueller [www.niemueller.de]
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

#include "mongodb_replicaset_config.h"
#include "mongodb_client_config.h"

#include <config/config.h>
#include <logging/logger.h>
#include <utils/time/wait.h>

#include <mongo/client/dbclient.h>

using namespace fawkes;

/** @class MongoDBReplicaSetConfig "mongodb_replicaset_config.h"
 * MongoDB replica set configuration.
 * Configure a replica set. This only takes care of initializing and
 * maintaining the configuration of a running replica set. The
 * mongod instances have to be handled separately, for example using
 * instance configurations.
 *
 * @author Tim Niemueller
 */

/** Constructor.
 * This will read the given configuration.
 * @param config configuration to query
 * @param cfgname configuration name
 * @param prefix configuration path prefix
 * @param bootstrap_client MongoDB client to access bootstrap database
 * @param bootstrap_database database to hold leader election data
 */
MongoDBReplicaSetConfig::MongoDBReplicaSetConfig(Configuration *config,
                                                 std::string cfgname, std::string prefix,
                                                 std::shared_ptr<mongo::DBClientBase> bootstrap_client,
                                                 std::string bootstrap_database)
	: Thread("MongoDBReplicaSet", Thread::OPMODE_CONTINUOUS)
{
	set_name("MongoDBReplicaSet|%s",  cfgname.c_str());
	config_name_ = cfgname;
	is_leader_ = false;
	
	enabled_ = false;
	try {
		enabled_ = config->get_bool(prefix + "enabled");
	} catch (Exception &e) {}
	
	if (enabled_) {

		bootstrap_client_     = bootstrap_client;
		bootstrap_database_   = bootstrap_database;
		bootstrap_ns_         = bootstrap_database + "." + config_name_;

		local_client_ = config->get_string(prefix + "local-client");
		loop_interval_ = 5.0;
		try {
			loop_interval_ = config->get_float(prefix + "loop-interval");
		} catch (Exception &e) {} // ignored, use default

		leader_expiration_ = 10;
		try {
			leader_expiration_ = config->get_int(prefix + "leader-expiration");
		} catch (Exception &e) {} // ignored, use default

		MongoDBClientConfig client_config(config, logger, local_client_,
		                                  "/plugins/mongodb/clients/" + local_client_ + "/");
		local_hostport_ = client_config.hostport();
		hosts_ = config->get_strings(prefix + "hosts");

		bootstrap_client_->createCollection(bootstrap_ns_);
		bootstrap_client_->createIndex(bootstrap_ns_, mongo::IndexSpec().addKey("host"));
		bootstrap_client_->createIndex(bootstrap_ns_, mongo::IndexSpec().addKey("master").unique());
		bootstrap_client_->createIndex(bootstrap_ns_,
		                               mongo::IndexSpec().addKey("last_seen").expireAfterSeconds(leader_expiration_));
	
		mongo::BSONObjBuilder query;
		query.append("host", local_hostport_);
		leader_elec_query_ = query.obj();

		mongo::BSONObjBuilder update;
		update.append("$currentDate", BSON("last_seen" << true));
		mongo::BSONObjBuilder update_set;
		update_set.append("master", true);
		update_set.append("host", local_hostport_);
		update.append("$set", update_set.obj());
		leader_elec_update_ = update.obj();
	}
}


void
MongoDBReplicaSetConfig::leader_elect()
{
	try {
		bootstrap_client_->update(bootstrap_ns_,
		                          leader_elec_query_, leader_elec_update_,
		                          /* upsert */ true, /* multi */ false,
		                          &mongo::WriteConcern::majority);
		if (! is_leader_) {
			is_leader_ = true;
			logger->log_info(name(), "Became replica set leader");
		}
	} catch (mongo::OperationException &e) {
		if (e.obj()["code"].numberInt() != 11000) {
			// 11000: Duplicate key exception, occurs if we do not become leader, all fine
			logger->log_error(name(), "Leader election failed (%i): %s %s", e.getCode(), e.what(), e.obj().jsonString().c_str());
			is_leader_ = false;
		} else if (is_leader_) {
			logger->log_warn(name(), "Lost replica set leadership");
			is_leader_ = false;
		}
	}
}

void
MongoDBReplicaSetConfig::leader_resign()
{
	if (is_leader_) {
		logger->log_info(name(), "Resigning replica set leadership");
		bootstrap_client_->remove(bootstrap_ns_,
		                          leader_elec_query_, /* just one */ true,
		                          &mongo::WriteConcern::majority);
	}
}


void
MongoDBReplicaSetConfig::init()
{
	if (!enabled_) {
		throw Exception("Replica set manager '%s' cannot be started while disabled", name());
	}

	logger->log_info(name(), "Bootstrap Query:  %s", leader_elec_query_.jsonString().c_str());
	logger->log_info(name(), "Bootstrap Update: %s", leader_elec_update_.jsonString().c_str());
	
	timewait_ = new TimeWait(clock, (int)(loop_interval_ * 1000000.));
}

void
MongoDBReplicaSetConfig::finalize()
{
	leader_resign();

	delete timewait_;
}

void
MongoDBReplicaSetConfig::loop()
{
	timewait_->mark_start();
	leader_elect();
	if (is_leader_) {
		// do leader stuff
	}
	timewait_->wait_systime();
}
