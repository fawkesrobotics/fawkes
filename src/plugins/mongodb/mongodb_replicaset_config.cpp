
/***************************************************************************
 *  mongodb_replicaset_config.cpp - MongoDB replica set configuration
 *
 *  Created: Thu Jul 13 10:25:19 2017
 *  Copyright  2006-2018  Tim Niemueller [www.niemueller.de]
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
#include <mongo/bson/bson.h>

#include <chrono>
#include <iterator>
#include <numeric>

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
	last_status_ = ReplicaSetStatus{
		.member_status  = MongoDBManagedReplicaSetInterface::ERROR,
		.primary_status = MongoDBManagedReplicaSetInterface::PRIMARY_UNKNOWN
	};
	
	enabled_ = false;
	try {
		enabled_ = config->get_bool(prefix + "enabled");
	} catch (Exception &e) {}

	if (enabled_) {

		bootstrap_client_     = bootstrap_client;
		bootstrap_database_   = bootstrap_database;
		bootstrap_ns_         = bootstrap_database + "." + config_name_;

		local_client_cfg_ = config->get_string(prefix + "local-client");
		loop_interval_ = 5.0;
		try {
			loop_interval_ = config->get_float(prefix + "loop-interval");
		} catch (Exception &e) {} // ignored, use default

		leader_expiration_ = 10;
		try {
			leader_expiration_ = config->get_int(prefix + "leader-expiration");
		} catch (Exception &e) {} // ignored, use default

		MongoDBClientConfig client_config(config, logger, local_client_cfg_,
		                                  "/plugins/mongodb/clients/" + local_client_cfg_ + "/");
		if (! client_config.is_enabled()) {
			throw Exception("%s: local client configuration '%s' disabled",
			                name(), local_client_cfg_.c_str());
		}
		if (client_config.mode() != MongoDBClientConfig::CONNECTION) {
			throw Exception("%s: local client configuration '%s' mode is not connection",
			                name(), local_client_cfg_.c_str());
		}
		local_hostport_ = client_config.hostport();
		std::vector<std::string> hostv = config->get_strings(prefix + "hosts");
		std::copy(hostv.begin(), hostv.end(), std::inserter(hosts_, hosts_.end()));

		if (std::find(hosts_.begin(), hosts_.end(), local_hostport_) == hosts_.end()) {
			throw Exception("%s host list does not include local client", name());
		}

		local_client_.reset(client_config.create_client());
		bootstrap_client_->createCollection(bootstrap_ns_);
		bootstrap_client_->createIndex(bootstrap_ns_, mongo::IndexSpec().addKey("host"));
		bootstrap_client_->createIndex(bootstrap_ns_, mongo::IndexSpec().addKey("master").unique());
		bootstrap_client_->createIndex(bootstrap_ns_,
		                               mongo::IndexSpec().addKey("last_seen").expireAfterSeconds(leader_expiration_));

		leader_elec_query_ = BSON("host" << local_hostport_ << "master" << false);
		leader_elec_query_force_ = BSON("master" << true);

		mongo::BSONObjBuilder update;
		update.append("$currentDate", BSON("last_seen" << true));
		mongo::BSONObjBuilder update_set;
		update_set.append("master", true);
		update_set.append("host", local_hostport_);
		update.append("$set", update_set.obj());
		leader_elec_update_ = update.obj();
	}
}


bool
MongoDBReplicaSetConfig::leader_elect(bool force)
{
	try {
		bootstrap_client_->update(bootstrap_ns_,
		                          force ? leader_elec_query_force_ : leader_elec_query_,
		                          leader_elec_update_,
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
	return is_leader_;
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

	logger->log_debug(name(), "Bootstrap Query:  %s", leader_elec_query_.jsonString().c_str());
	logger->log_debug(name(), "Bootstrap Update: %s", leader_elec_update_.jsonString().c_str());

	rs_status_if_ = blackboard->open_for_writing<MongoDBManagedReplicaSetInterface>(config_name_.c_str());

	timewait_ = new TimeWait(clock, (int)(loop_interval_ * 1000000.));
}

void
MongoDBReplicaSetConfig::finalize()
{
	leader_resign();
	blackboard->close(rs_status_if_);

	delete timewait_;
}

void
MongoDBReplicaSetConfig::loop()
{
	timewait_->mark_start();
	mongo::BSONObj reply;
	ReplicaSetStatus status = rs_status(reply);

	if (status.primary_status == MongoDBManagedReplicaSetInterface::NO_PRIMARY) {
		logger->log_warn(name(), "No primary, triggering leader election");
		if (leader_elect(/* force leadership */ false)) {
			logger->log_info(name(), "No primary, we became leader, managing");
			rs_monitor(reply);
		}
	}

	switch (status.member_status) {
	case MongoDBManagedReplicaSetInterface::PRIMARY:
		if (last_status_.member_status != status.member_status) {
			logger->log_info(name(), "Became PRIMARY, starting managing");
		}
		leader_elect(/* force leaderhsip */ true);
		rs_monitor(reply);
		break;
	case MongoDBManagedReplicaSetInterface::SECONDARY:
		if (last_status_.member_status != status.member_status) {
			logger->log_info(name(), "Became SECONDARY");
		}
		break;
	case MongoDBManagedReplicaSetInterface::ARBITER:
		//logger->log_info(name(), "Arbiter");
		break;
	case MongoDBManagedReplicaSetInterface::NOT_INITIALIZED:
		if (hosts_.size() == 1 || leader_elect()) {
			// we are alone or leader, initialize replica set
			if (hosts_.size() == 1) {
				logger->log_info(name(), "Now initializing RS (alone)");
			} else {
				logger->log_info(name(), "Now initializing RS (leader)");
			}
			rs_init();
		}
		break;
	case MongoDBManagedReplicaSetInterface::INVALID_CONFIG:
		// we might later want to cover some typical cases
		logger->log_error(name(), "Invalid configuration, hands-on required\n%s",
		                  reply.jsonString().c_str());
		break;
	default:
		break;
	}

	if (last_status_ != status) {
		rs_status_if_->set_member_status(status.member_status);
		rs_status_if_->set_primary_status(status.primary_status);
		rs_status_if_->set_error_msg(status.error_msg.c_str());
		rs_status_if_->write();

		last_status_ = status;
	}

	timewait_->wait_systime();
}


MongoDBReplicaSetConfig::ReplicaSetStatus
MongoDBReplicaSetConfig::rs_status(mongo::BSONObj &reply)
{
	ReplicaSetStatus status = {
		.member_status  = MongoDBManagedReplicaSetInterface::ERROR,
		.primary_status = MongoDBManagedReplicaSetInterface::PRIMARY_UNKNOWN
	};

	mongo::BSONObj cmd(BSON("replSetGetStatus" << 1));
	try {
		bool ok = local_client_->runCommand("admin", cmd, reply);

		if (! ok) {
			if (reply["code"].numberInt() == mongo::ErrorCodes::NotYetInitialized) {
				logger->log_warn(name(), "Instance has not received replica set configuration, yet");
				status.member_status = MongoDBManagedReplicaSetInterface::NOT_INITIALIZED;
				status.error_msg   = "Instance has not received replica set configuration, yet";
			} else if (reply["code"].numberInt() == mongo::ErrorCodes::InvalidReplicaSetConfig) {
				logger->log_error(name(), "Invalid replica set configuration: %s", reply.jsonString().c_str());
				status.member_status = MongoDBManagedReplicaSetInterface::INVALID_CONFIG;
				status.error_msg   = "Invalid replica set configuration: " + reply.jsonString();
			} else {
				status.error_msg   = "Unknown error";
			}
			return status;
		} else {
			//logger->log_warn(name(), "rs status reply: %s", reply.jsonString().c_str());
			try {
				mongo::BSONObjIterator members(reply.getObjectField("members"));
				bool have_primary = false;
				MongoDBManagedReplicaSetInterface::ReplicaSetMemberStatus self_status =
					MongoDBManagedReplicaSetInterface::REMOVED;
				while(members.more()) {
					mongo::BSONObj m = members.next().Obj();
					int state = m["state"].Int();
					if (state == 1)  have_primary = true;

					if (m.hasField("self") && m["self"].boolean()) {
						switch (state) {
						case 1: self_status = MongoDBManagedReplicaSetInterface::PRIMARY;   break;
						case 2:	self_status = MongoDBManagedReplicaSetInterface::SECONDARY; break;
						case 3: // RECOVERING
						case 5: // STARTUP2
						case 9: // ROLLBACK
							self_status = MongoDBManagedReplicaSetInterface::INITIALIZING;    break;
							break;
						case 7: self_status = MongoDBManagedReplicaSetInterface::ARBITER;   break;
						default: self_status = MongoDBManagedReplicaSetInterface::ERROR;    break;
						}
					}
				}
				status.primary_status =
					have_primary ? MongoDBManagedReplicaSetInterface::HAVE_PRIMARY
					             : MongoDBManagedReplicaSetInterface::NO_PRIMARY;
				status.member_status   = self_status;
				return status;
			} catch (mongo::DBException &e) {
				logger->log_warn(name(), "Failed to analyze member info: %s", e.what());
				status.member_status = MongoDBManagedReplicaSetInterface::ERROR;
				status.error_msg   = std::string("Failed to analyze member info: ") + e.what();
				return status;
			}
		}
	} catch (mongo::DBException &e) {
		logger->log_warn(name(), "Failed to get RS status: %s", e.what());
		status.member_status = MongoDBManagedReplicaSetInterface::ERROR;
		status.error_msg   = std::string("Failed to get RS status: ") + e.what();
		return status;
	}
	return status;
}


void
MongoDBReplicaSetConfig::rs_init()
{
	// using default configuration, this will just add ourself
	mongo::BSONObj conf;
	mongo::BSONObj cmd(BSON("replSetInitiate" << conf));

	mongo::BSONObj reply;
	try {
		bool ok = local_client_->runCommand("admin", cmd, reply);
		if (! ok) {
			logger->log_error(name(), "RS initialization failed: %s", reply["errmsg"].toString().c_str());
		} else {
			logger->log_debug(name(), "RS initialized successfully: %s", reply.jsonString().c_str());
		}
	} catch (mongo::DBException &e) {
		logger->log_error(name(), "RS initialization failed: %s", e.what());
	}
}


bool
MongoDBReplicaSetConfig::rs_get_config(mongo::BSONObj &rs_config)
{
	mongo::BSONObj cmd(BSON("replSetGetConfig" << 1));

	try {
		mongo::BSONObj reply;
		bool ok = local_client_->runCommand("admin", cmd, reply);
		if (ok) {
			rs_config = reply["config"].Obj().copy();
			//logger->log_info(name(), "Config: %s", rs_config.jsonString(mongo::Strict, true).c_str());
		} else {
			logger->log_warn(name(), "Failed to get RS config: %s (DB error)", reply["errmsg"].str().c_str());
		}
		return ok;
	} catch (mongo::DBException &e) {
		logger->log_warn(name(), "Failed to get RS config: %s", e.what());
		return false;
	}
}

void
MongoDBReplicaSetConfig::rs_monitor(const mongo::BSONObj &status_reply)
{
	using namespace std::chrono_literals;

	std::set<std::string>  in_rs, unresponsive, new_alive, members;
	int last_member_id = 0;

	mongo::BSONObjIterator members_it(status_reply.getObjectField("members"));
	while(members_it.more()) {
		mongo::BSONObj m = members_it.next().Obj();
		members.insert(m["name"].str());

		last_member_id = std::max(m["_id"].numberInt(), last_member_id);

		// determine members to remove
		if (m.hasField("self") && m["self"].boolean()) {
			in_rs.insert(m["name"].str());
		} else {
			std::chrono::time_point<std::chrono::high_resolution_clock>
				last_heartbeat_rcvd(std::chrono::milliseconds(m["lastHeartbeatRecv"].date()));
			auto now = std::chrono::high_resolution_clock::now();
			auto diff_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_heartbeat_rcvd);
			if ( (m["health"].numberInt() != 1) || (now - last_heartbeat_rcvd) > 15s) {
				//logger->log_info(name(), "Reply: %s", status_reply.jsonString(mongo::Strict, true).c_str());
				unresponsive.insert(m["name"].str());
			} else {
				in_rs.insert(m["name"].str());
			}
		}
	}

	std::set<std::string> not_member;
	std::set_difference(hosts_.begin(), hosts_.end(), in_rs.begin(), in_rs.end(),
	                    std::inserter(not_member, not_member.end()));

	for (const std::string &h : not_member) {
		// Check if this host became alive, and add if it did
		if (check_alive(h)) {
			logger->log_info(name(), "Host %s alive, adding to RS", h.c_str());
			new_alive.insert(h);
		//} else {
			//logger->log_info(name(), "Potential member %s not responding", h.c_str());
		}
	}

	if (! unresponsive.empty() || ! new_alive.empty()) {
		// generate new config
		mongo::BSONObj rs_config;
		if (! rs_get_config(rs_config))  return;

		mongo::BSONObjBuilder new_config;
		std::set<std::string> field_names;
		rs_config.getFieldNames(field_names);
		for (const std::string &fn : field_names) {
			if (fn == "version") {
				new_config.append("version", rs_config["version"].numberInt() + 1);
			} else if (fn == "members") {
				mongo::BSONObjIterator members_it(rs_config.getObjectField("members"));

				mongo::BSONArrayBuilder members_arr(new_config.subarrayStart("members"));

				while(members_it.more()) {
					mongo::BSONObj m = members_it.next().Obj();
					std::string host = m["host"].str();
					if (hosts_.find(host) == hosts_.end()) {
					logger->log_warn(name(), "Removing '%s', "
					  "not part of the replica set configuration", host.c_str());
					} else if (unresponsive.find(host) == unresponsive.end()) {
						// it's not unresponsive, add
						logger->log_warn(name(), "Keeping RS member '%s'", host.c_str());
						members_arr.append(m);
					} else {
						logger->log_warn(name(), "Removing RS member '%s'", host.c_str());
					}
				}
				for (const std::string &h : new_alive) {
					logger->log_info(name(), "Adding new RS member '%s'", h.c_str());
					mongo::BSONObjBuilder membuild;
					membuild.append("_id", ++last_member_id);
					membuild.append("host", h);
					members_arr.append(membuild.obj());
				}
				members_arr.doneFast();
			} else {
				try {
					new_config.append(rs_config[fn]);
				} catch (mongo::MsgAssertionException &e) {
					logger->log_error(name(), "ERROR on RS reconfigure (%s): %s", fn.c_str(), e.what());
					return;
				}
			}
		}

		mongo::BSONObj new_config_obj(new_config.obj());
		//logger->log_info(name(), "Reconfigure: %s", new_config_obj.jsonString(mongo::Strict, true).c_str());

		mongo::BSONObjBuilder cmd;
		cmd.append("replSetReconfig", new_config_obj);
		cmd.append("force", true);

		try {
			mongo::BSONObj reply;
			bool ok = local_client_->runCommand("admin", cmd.obj(), reply);
			if (! ok) {
				logger->log_error(name(), "RS reconfig failed: %s (DB error)", reply["errmsg"].str().c_str());
			}
		} catch (mongo::DBException &e) {
			logger->log_warn(name(), "RS reconfig failed: %s (exception)", e.what());
		}
	}
}


bool
MongoDBReplicaSetConfig::check_alive(const std::string &h)
{
	try {
		std::shared_ptr<mongo::DBClientConnection> client =
			std::make_shared<mongo::DBClientConnection>();
		std::string errmsg;
		mongo::HostAndPort hostport(h);
		if (! client->connect(hostport, errmsg)) {
			return false;
		}
		mongo::BSONObj cmd(BSON("isMaster" << 1));
		mongo::BSONObj reply;
		bool ok = client->runCommand("admin", cmd, reply);
		if (! ok) {
			logger->log_warn(name(), "Failed to connect: %s", reply.jsonString().c_str());
		}
		return ok;
	} catch (mongo::DBException &e) {
		logger->log_warn(name(), "Fail: %s", e.what());
		return false;
	}
}
