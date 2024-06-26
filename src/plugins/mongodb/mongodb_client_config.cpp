
/***************************************************************************
 *  mongodb_client_config.cpp - MongoDB client configuration
 *
 *  Created: Wed Jul 12 13:45:03 2017
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

#include "mongodb_client_config.h"

#include <config/config.h>
#include <logging/logger.h>

#include <chrono>
#include <thread>

using namespace fawkes;

/** @class MongoDBClientConfig "mongodb_client_config.h"
 * MongoDB Client Configuration.
 * Instances of this class represent a single MongoDB client configuration
 * used to initiate connections.
 *
 * @author Tim Niemueller
 */

/** Constructor.
 * This will read the given configuration.
 * @param config configuration to query
 * @param logger logger for info messages
 * @param cfgname configuration name
 * @param prefix configuration path prefix
 */
MongoDBClientConfig::MongoDBClientConfig(Configuration *config,
                                         Logger        *logger,
                                         std::string    cfgname,
                                         std::string    prefix)
{
	logcomp_ = "MongoDBClient|" + cfgname;

	enabled_ = false;
	try {
		enabled_ = config->get_bool((prefix + "enabled").c_str());
	} catch (Exception &e) {
	}

	std::string mode = "connection";
	try {
		mode = config->get_string((prefix + "mode").c_str());
	} catch (Exception &e) {
		logger->log_info(logcomp_.c_str(),
		                 "MongoDB config '%s' specifies no client "
		                 "mode, assuming 'connection'.",
		                 cfgname.c_str());
	}

	startup_grace_period_ = 10;
	try {
		startup_grace_period_ = config->get_uint(prefix + "startup_grace_period");
	} catch (Exception &e) {
		logger->log_info(logcomp_.c_str(),
		                 "MongoDB config '%s' specifies no startup grace period "
		                 ", assuming 10 seconds.",
		                 cfgname.c_str());
	}

	auth_string_ = "";
	read_authinfo(config, logger, cfgname, prefix);

	if (mode == "replica_set" || mode == "replicaset") {
		mode_            = REPLICA_SET;
		replicaset_name_ = config->get_string((prefix + "name").c_str());

		std::vector<std::string> hosts = config->get_strings(prefix + "hosts");
		std::string              uri{"mongodb://"};
		for (auto it = hosts.begin(); it != hosts.end(); it++) {
			uri += *it;
			if (std::next(it) != hosts.end()) {
				uri += ",";
			}
		}
		uri += "/" + auth_dbname;
		uri += "?replicaSet=" + replicaset_name_;
		try {
			uri += "&readPreference=" + config->get_string((prefix + "read-preference").c_str());
		} catch (Exception &e) {
			// use default read preference
		}
		try {
			uri += "&readPreferenceTags=" + config->get_string((prefix + "read-preference-tags").c_str());
		} catch (Exception &e) {
		}
		conn_uri_ = mongocxx::uri{uri};

	} else if (mode == "sync_cluster" || mode == "synccluster") {
		throw Exception("sync_cluster connections are no longer supported");

	} else {
		mode_     = CONNECTION;
		conn_uri_ = mongocxx::uri{"mongodb://" + auth_string_ + config->get_string(prefix + "hostport")
		                          + "/" + auth_dbname};
	}
}

/** Read authentication info for given configuration.
 * This will try to read the fields auth_dbname, auth_username, and
 * auth_password.
 * @param config configuration to query
 * @param logger logger for info messages
 * @param cfgname configuration name
 * @param prefix configuration path prefix
 */
void
MongoDBClientConfig::read_authinfo(Configuration *config,
                                   Logger        *logger,
                                   std::string    cfgname,
                                   std::string    prefix)
{
	try {
		auth_dbname          = config->get_string((prefix + "auth_dbname").c_str());
		std::string username = config->get_string((prefix + "auth_username").c_str());
		std::string password = config->get_string((prefix + "auth_password").c_str());
		if (!username.empty() && !password.empty()) {
			auth_string_ = username + ":" + password + "@";
		}
	} catch (Exception &e) {
		logger->log_info(logcomp_.c_str(),
		                 "No default authentication info for "
		                 "MongoDB client '%s'",
		                 cfgname.c_str());
	}
}

/** Create MongoDB client for this configuration.
 * @return MongoDB client
 */
mongocxx::client *
MongoDBClientConfig::create_client()
{
	for (unsigned int startup_tries = 0; startup_tries < startup_grace_period_ * 2; ++startup_tries) {
		try {
			return new mongocxx::client(conn_uri_);
		} catch (fawkes::Exception &) {
			using namespace std::chrono_literals;
			std::this_thread::sleep_for(500ms);
		}
	}
	throw Exception("Failed to create client with uri: '%s'", conn_uri_.to_string().c_str());
}

/** Write client configuration information to log.
 * @param logger logger to write to
 * @param component component to pass to logger
 * @param indent indentation to put before each string
 */
void
MongoDBClientConfig::log(Logger *logger, const char *component, const char *indent)
{
	switch (mode_) {
	case REPLICA_SET: {
		logger->log_info(component, "%smode:   replica set", indent);
	} break;
	default: {
		logger->log_info(component, "%smode:   connection", indent);
	} break;
	}
	std::string uri_string = conn_uri_.to_string();
	if (!auth_string_.empty()) {
		if (uri_string.find(auth_string_) != std::string::npos) {
			uri_string.replace(uri_string.find(auth_string_), auth_string_.length(), "****@****");
		}
	}
	logger->log_info(component, "%suri:    %s", indent, uri_string.c_str());
}

/** Get host and port of configuration.
 * @return string of the form "host:port"
 */
std::string
MongoDBClientConfig::hostport() const
{
	auto hosts = conn_uri_.hosts();
	if (hosts.empty()) {
		return "";
	} else {
		// If multiple hosts are specified, only use the first host.
		return hosts[0].name + ":" + std::to_string(hosts[0].port);
	}
}

/** Get client configuration mode.
 * @return mode, connection or replica set
 */
MongoDBClientConfig::ConnectionMode
MongoDBClientConfig::mode() const
{
	return mode_;
}
