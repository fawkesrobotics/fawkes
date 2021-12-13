
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
#include "utils.h"

#include <config/config.h>
#include <logging/logger.h>
#include <utils/time/wait.h>

#include <bsoncxx/builder/basic/document.hpp>
#include <chrono>
#include <iterator>
#include <mongocxx/exception/operation_exception.hpp>
#include <numeric>

using namespace fawkes;
using namespace bsoncxx::builder;

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
 * @param cfgname configuration name
 * @param prefix configuration path prefix
 * @param bootstrap_prefix configuration path prefix for bootstrap configuration
 */
MongoDBReplicaSetConfig::MongoDBReplicaSetConfig(const std::string &cfgname,
                                                 const std::string &prefix,
                                                 const std::string &bootstrap_prefix)
: Thread("MongoDBReplicaSet", Thread::OPMODE_CONTINUOUS),
  leader_elec_query_(bsoncxx::builder::basic::document()),
  leader_elec_query_force_(bsoncxx::builder::basic::document()),
  leader_elec_update_(bsoncxx::builder::basic::document())
{
	set_name("MongoDBReplicaSet|%s", cfgname.c_str());
	config_name_      = cfgname;
	prefix_           = prefix;
	bootstrap_prefix_ = bootstrap_prefix;
	is_leader_        = false;
	last_status_ =
	  ReplicaSetStatus{.member_status  = MongoDBManagedReplicaSetInterface::ERROR,
	                   .primary_status = MongoDBManagedReplicaSetInterface::PRIMARY_UNKNOWN};

	enabled_ = false;
}

void
MongoDBReplicaSetConfig::init()
{
	try {
		enabled_ = config->get_bool(prefix_ + "enabled");
	} catch (Exception &e) {
	}
	if (!enabled_) {
		throw Exception("Replica set manager '%s' cannot be started while disabled", name());
	}

	logger->log_debug(name(),
	                  "Bootstrap Query:  %s",
	                  bsoncxx::to_json(leader_elec_query_.view()).c_str());
	logger->log_debug(name(),
	                  "Bootstrap Update: %s",
	                  bsoncxx::to_json(leader_elec_update_.view()).c_str());

	rs_status_if_ =
	  blackboard->open_for_writing<MongoDBManagedReplicaSetInterface>(config_name_.c_str());

	if (enabled_) {
		bootstrap_database_                      = config->get_string(bootstrap_prefix_ + "database");
		std::string         bootstrap_client_cfg = config->get_string(bootstrap_prefix_ + "client");
		MongoDBClientConfig bootstrap_client_config(config,
		                                            logger,
		                                            bootstrap_client_cfg,
		                                            "/plugins/mongodb/clients/" + bootstrap_client_cfg
		                                              + "/");
		if (!bootstrap_client_config.is_enabled()) {
			throw Exception("%s: bootstrap client configuration '%s' disabled",
			                name(),
			                bootstrap_client_cfg.c_str());
		}
		bootstrap_client_.reset(bootstrap_client_config.create_client());
		bootstrap_ns_ = bootstrap_database_ + "." + config_name_;

		local_client_cfg_ = config->get_string(prefix_ + "local-client");
		loop_interval_    = 5.0;
		try {
			loop_interval_ = config->get_float(prefix_ + "loop-interval");
		} catch (Exception &e) {
		} // ignored, use default

		timewait_ = new TimeWait(clock, (int)(loop_interval_ * 1000000.));

		leader_expiration_ = 10;
		try {
			leader_expiration_ = config->get_int(prefix_ + "leader-expiration");
		} catch (Exception &e) {
		} // ignored, use default

		MongoDBClientConfig client_config(config,
		                                  logger,
		                                  local_client_cfg_,
		                                  "/plugins/mongodb/clients/" + local_client_cfg_ + "/");
		if (!client_config.is_enabled()) {
			throw Exception("%s: local client configuration '%s' disabled",
			                name(),
			                local_client_cfg_.c_str());
		}
		if (client_config.mode() != MongoDBClientConfig::CONNECTION) {
			throw Exception("%s: local client configuration '%s' mode is not connection",
			                name(),
			                local_client_cfg_.c_str());
		}
		local_hostport_                = client_config.hostport();
		std::vector<std::string> hostv = config->get_strings(prefix_ + "hosts");
		std::copy(hostv.begin(), hostv.end(), std::inserter(hosts_, hosts_.end()));

		if (std::find(hosts_.begin(), hosts_.end(), local_hostport_) == hosts_.end()) {
			throw Exception("%s host list does not include local client", name());
		}

		using namespace bsoncxx::builder::basic;

		auto leader_elec_query_builder = basic::document{};
		leader_elec_query_builder.append(basic::kvp("host", local_hostport_));
		leader_elec_query_ = leader_elec_query_builder.extract();

		auto leader_elec_query_force_builder = basic::document{};
		leader_elec_query_builder.append(basic::kvp("master", true));
		leader_elec_query_force_ = leader_elec_query_force_builder.extract();

		auto leader_elec_update_builder = basic::document{};
		leader_elec_update_builder.append(basic::kvp("$currentDate", [](basic::sub_document subdoc) {
			subdoc.append(basic::kvp("last_seen", true));
		}));
		leader_elec_update_builder.append(basic::kvp("$set", [this](basic::sub_document subdoc) {
			subdoc.append(basic::kvp("master", true));
			subdoc.append(basic::kvp("host", local_hostport_));
		}));
		leader_elec_update_ = leader_elec_update_builder.extract();

		local_client_.reset(client_config.create_client());
	}
	bootstrap();
}

/** Setup replicaset bootstrap client.
 * @param bootstrap_client MongoDB client to access bootstrap database
 */
void
MongoDBReplicaSetConfig::bootstrap()
{
	if (enabled_) {
		try {
			auto database   = bootstrap_client_->database(bootstrap_database_);
			auto collection = database[bootstrap_ns_];

			collection.create_index(basic::make_document(basic::kvp("host", 1)));
			collection.create_index(basic::make_document(basic::kvp("master", 1)),
			                        basic::make_document(basic::kvp("unique", true)));
			collection.create_index(basic::make_document(basic::kvp("last_seen", 1)),
			                        basic::make_document(
			                          basic::kvp("expireAfterSeconds", leader_expiration_)));
		} catch (mongocxx::operation_exception &e) {
			logger->log_error(name(), "Failed to initialize bootstrap client: %s", e.what());
			throw;
		}
	}
}

bool
MongoDBReplicaSetConfig::leader_elect(bool force)
{
	try {
		auto write_concern = mongocxx::write_concern();
		write_concern.majority(std::chrono::milliseconds(0));
		auto result = bootstrap_client_->database(bootstrap_database_)[bootstrap_ns_].update_one(
		  force ? leader_elec_query_force_.view() : leader_elec_query_.view(),
		  leader_elec_update_.view(),
		  mongocxx::options::update().upsert(true).write_concern(write_concern));
		if (result) {
			if (!is_leader_) {
				is_leader_ = true;
				logger->log_info(name(), "Became replica set leader");
			}
		} else {
			if (is_leader_) {
				is_leader_ = false;
				logger->log_warn(name(), "Lost replica set leadership");
			}
		}
	} catch (mongocxx::operation_exception &e) {
		if (boost::optional<bsoncxx::document::value> error = e.raw_server_error()) {
			bsoncxx::array::view writeErrors = error->view()["writeErrors"].get_array();
			// Do not use writeErrors.length(), which is not the number of errors!
			auto num_errors = std::distance(writeErrors.begin(), writeErrors.end());
			int  error_code = -1;
			if (num_errors > 0) {
				error_code = error->view()["writeErrors"][0]["code"].get_int32();
			}
			if (num_errors > 1 || error_code != 11000) {
				// 11000: Duplicate key exception, occurs if we do not become leader, all fine
				logger->log_error(name(),
				                  "Leader election failed (%i): %s %s",
				                  error_code,
				                  e.what(),
				                  bsoncxx::to_json(error->view()).c_str());
				is_leader_ = false;
			} else if (is_leader_) {
				logger->log_warn(name(), "Lost replica set leadership");
				is_leader_ = false;
			}
		} else {
			logger->log_error(name(), "Leader election failed; failed to fetch error code: %s", e.what());
		}
	}
	return is_leader_;
}

void
MongoDBReplicaSetConfig::leader_resign()
{
	if (is_leader_) {
		logger->log_info(name(), "Resigning replica set leadership");
		auto write_concern = mongocxx::write_concern();
		write_concern.majority(std::chrono::milliseconds(0));
		bootstrap_client_->database(bootstrap_database_)[bootstrap_ns_].delete_one(
		  leader_elec_query_.view(), mongocxx::options::delete_options().write_concern(write_concern));
	}
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
	bsoncxx::document::value reply{bsoncxx::builder::basic::document()};
	ReplicaSetStatus         status = rs_status(reply);

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
		logger->log_error(name(),
		                  "Invalid configuration, hands-on required\n%s",
		                  bsoncxx::to_json(reply.view()).c_str());
		break;
	default: break;
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
MongoDBReplicaSetConfig::rs_status(bsoncxx::document::value &reply)
{
	ReplicaSetStatus status = {.member_status  = MongoDBManagedReplicaSetInterface::ERROR,
	                           .primary_status = MongoDBManagedReplicaSetInterface::PRIMARY_UNKNOWN};

	auto cmd = basic::make_document(basic::kvp("replSetGetStatus", 1));
	try {
		reply = local_client_->database("admin").run_command(std::move(cmd));
	} catch (mongocxx::operation_exception &e) {
		int  error_code         = -1;
		auto error_code_element = e.raw_server_error()->view()["code"];
		if (error_code_element && error_code_element.type() == bsoncxx::type::k_int32) {
			error_code = e.raw_server_error()->view()["code"].get_int32();
		}
		if (error_code == 94 /* NotYetInitialized */) {
			logger->log_warn(name(), "Instance has not received replica set configuration, yet");
			status.member_status = MongoDBManagedReplicaSetInterface::NOT_INITIALIZED;
			status.error_msg     = "Instance has not received replica set configuration, yet";
		} else if (error_code == 93 /* InvalidReplicaSetConfig */) {
			logger->log_error(name(),
			                  "Invalid replica set configuration: %s",
			                  bsoncxx::to_json(reply.view()).c_str());
			status.member_status = MongoDBManagedReplicaSetInterface::INVALID_CONFIG;
			status.error_msg     = "Invalid replica set configuration: " + bsoncxx::to_json(reply.view());
		} else {
			status.error_msg = "Unknown error";
		}
		return status;
	}
	//logger->log_warn(name(), "rs status reply: %s", bsoncxx::to_json(reply.view()).c_str());
	try {
		MongoDBManagedReplicaSetInterface::ReplicaSetMemberStatus self_status =
		  MongoDBManagedReplicaSetInterface::REMOVED;
		auto members = reply.view()["members"];
		if (members && members.type() == bsoncxx::type::k_array) {
			bsoncxx::array::view members_view{members.get_array().value};
			bool                 have_primary = false;
			for (bsoncxx::array::element member : members_view) {
				int state = member["state"].get_int32();
				if (state == 1) {
					have_primary = true;
				}
				if (member["self"] && member["self"].get_bool()) {
					switch (state) {
					case 1: self_status = MongoDBManagedReplicaSetInterface::PRIMARY; break;
					case 2: self_status = MongoDBManagedReplicaSetInterface::SECONDARY; break;
					case 3: // RECOVERING
					case 5: // STARTUP2
					case 9: // ROLLBACK
						self_status = MongoDBManagedReplicaSetInterface::INITIALIZING;
						break;
					case 7: self_status = MongoDBManagedReplicaSetInterface::ARBITER; break;
					default: self_status = MongoDBManagedReplicaSetInterface::ERROR; break;
					}
				}
			}
			status.primary_status = have_primary ? MongoDBManagedReplicaSetInterface::HAVE_PRIMARY
			                                     : MongoDBManagedReplicaSetInterface::NO_PRIMARY;
			status.member_status  = self_status;
			return status;
		} else {
			logger->log_error(name(),
			                  "Received replica set status reply without members, unknown status");
			self_status = MongoDBManagedReplicaSetInterface::ERROR;
		}
	} catch (mongocxx::operation_exception &e) {
		logger->log_warn(name(), "Failed to analyze member info: %s", e.what());
		status.member_status = MongoDBManagedReplicaSetInterface::ERROR;
		status.error_msg     = std::string("Failed to analyze member info: ") + e.what();
		return status;
	}
	return status;
}

void
MongoDBReplicaSetConfig::rs_init()
{
	// using default configuration, this will just add ourself
	auto cmd = basic::make_document(basic::kvp("replSetInitiate", basic::document{}));
	bsoncxx::document::value reply{bsoncxx::builder::basic::document()};
	try {
		reply   = local_client_->database("admin").run_command(std::move(cmd));
		bool ok = check_mongodb_ok(reply.view());
		if (!ok) {
			logger->log_error(name(),
			                  "RS initialization failed: %s",
			                  reply.view()["errmsg"].get_utf8().value.to_string().c_str());
		} else {
			logger->log_debug(name(),
			                  "RS initialized successfully: %s",
			                  bsoncxx::to_json(reply.view()).c_str());
		}
	} catch (mongocxx::operation_exception &e) {
		logger->log_error(name(), "RS initialization failed: %s", e.what());
	}
}

bool
MongoDBReplicaSetConfig::rs_get_config(bsoncxx::document::value &rs_config)
{
	auto cmd = basic::make_document(basic::kvp("replSetGetConfig", 1));

	try {
		bsoncxx::document::value reply{bsoncxx::builder::basic::document()};
		reply   = local_client_->database("admin").run_command(std::move(cmd));
		bool ok = check_mongodb_ok(reply.view());
		if (ok) {
			rs_config = reply;
			//logger->log_info(name(), "Config: %s", bsoncxx::to_json(rs_config.view()["config"]).c_str());
		} else {
			logger->log_warn(name(),
			                 "Failed to get RS config: %s (DB error)",
			                 bsoncxx::to_json(reply.view()).c_str());
		}
		return ok;
	} catch (mongocxx::operation_exception &e) {
		logger->log_warn(name(), "Failed to get RS config: %s", e.what());
		return false;
	}
}

void
MongoDBReplicaSetConfig::rs_monitor(const bsoncxx::document::view &status_reply)
{
	using namespace std::chrono_literals;

	std::set<std::string> in_rs, unresponsive, new_alive, members;
	int                   last_member_id{0};
	bsoncxx::array::view  members_view{status_reply["members"].get_array().value};
	for (bsoncxx::array::element member : members_view) {
		std::string member_name = member["name"].get_utf8().value.to_string();
		members.insert(member_name);
		last_member_id = std::max(int(member["_id"].get_int32()), last_member_id);
		if (member["self"] && member["self"].get_bool()) {
			in_rs.insert(member_name);
		} else {
			std::chrono::time_point<std::chrono::high_resolution_clock> last_heartbeat_rcvd(
			  std::chrono::milliseconds(member["lastHeartbeatRecv"].get_date()));
			auto now = std::chrono::high_resolution_clock::now();
			if ((int(member["health"].get_double()) != 1) || (now - last_heartbeat_rcvd) > 15s) {
				unresponsive.insert(member_name);
			} else {
				in_rs.insert(member_name);
			}
		}
	}
	std::set<std::string> not_member;
	std::set_difference(hosts_.begin(),
	                    hosts_.end(),
	                    in_rs.begin(),
	                    in_rs.end(),
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

	if (!unresponsive.empty() || !new_alive.empty()) {
		// generate new config
		bsoncxx::document::value reply{bsoncxx::builder::basic::document()};
		if (!rs_get_config(reply)) {
			return;
		}
		auto config = reply.view()["config"].get_document().view();
		using namespace bsoncxx::builder::basic;
		logger->log_info(name(), "Creating new config");
		auto new_config = basic::document{};
		for (auto &&key_it = config.begin(); key_it != config.end(); key_it++) {
			if (key_it->key() == "version") {
				new_config.append(basic::kvp("version", config["version"].get_int32() + 1));
				//new_config = new_config << "version" << config["version"].get_int32() + 1;
			} else if (key_it->key() == "members") {
				bsoncxx::array::view members_view{config["members"].get_array().value};
				new_config.append(basic::kvp("members", [&](basic::sub_array array) {
					for (bsoncxx::array::element member : members_view) {
						std::string host = member["host"].get_utf8().value.to_string();
						if (hosts_.find(host) == hosts_.end()) {
							logger->log_warn(name(),
							                 "Removing '%s', "
							                 "not part of the replica set configuration",
							                 host.c_str());
						} else if (unresponsive.find(host) == unresponsive.end()) {
							// it's not unresponsive, add
							logger->log_warn(name(), "Keeping RS member '%s'", host.c_str());
							array.append(basic::make_document(basic::kvp("host", host),
							                                  basic::kvp("_id", member["_id"].get_value())));
						} else {
							logger->log_warn(name(), "Removing RS member '%s'", host.c_str());
						}
					}
					for (const std::string &h : new_alive) {
						logger->log_info(name(), "Adding new RS member '%s'", h.c_str());
						array.append(
						  basic::make_document(basic::kvp("host", h), basic::kvp("_id", ++last_member_id)));
					}
				}));
			} else {
				new_config.append(basic::kvp(key_it->key(), key_it->get_value()));
			}
		}

		//mongo::BSONObj new_config_obj(new_config.obj());
		//logger->log_info(name(), "Reconfigure: %s", new_config_obj.jsonString(mongo::Strict, true).c_str());

		auto cmd = basic::document{};
		cmd.append(basic::kvp("replSetReconfig", new_config));
		cmd.append(basic::kvp("force", true));
		try {
			logger->log_info(name(), "Running command");
			auto reply = local_client_->database("admin").run_command(cmd.view());
			logger->log_info(name(), "done");
			bool ok = check_mongodb_ok(reply.view());
			if (!ok) {
				logger->log_error(name(),
				                  "RS reconfig failed: %s (DB error)",
				                  reply.view()["errmsg"].get_utf8().value.to_string().c_str());
			}
		} catch (mongocxx::operation_exception &e) {
			logger->log_warn(name(), "RS reconfig failed: %s (exception)", e.what());
		}
	}
}

bool
MongoDBReplicaSetConfig::check_alive(const std::string &h)
{
	using namespace bsoncxx::builder::basic;
	try {
		mongocxx::client client{mongocxx::uri{"mongodb://" + h}};
		auto             cmd = basic::document{};
		cmd.append(basic::kvp("isMaster", 1));
		auto reply = client.database("admin").run_command(cmd.view());
		bool ok    = check_mongodb_ok(reply.view());
		if (!ok) {
			logger->log_warn(name(), "Failed to connect: %s", bsoncxx::to_json(reply.view()).c_str());
		}
		return ok;
	} catch (mongocxx::operation_exception &e) {
		logger->log_warn(name(), "Fail: %s", e.what());
		return false;
	}
}
