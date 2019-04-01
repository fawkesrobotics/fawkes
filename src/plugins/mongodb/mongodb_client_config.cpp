
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
                                         Logger *       logger,
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

	std::string auth_string = "";
	if (!username.empty() && !password.empty()) {
		auth_string = username + ":" + password + "@";
	}

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
		conn_uri_ = mongocxx::uri{uri};

	} else if (mode == "sync_cluster" || mode == "synccluster") {
		throw Exception("sync_cluster connections are no longer supported");

	} else {
		mode_     = CONNECTION;
		conn_uri_ = mongocxx::uri{"mongodb://" + auth_string + config->get_string(prefix + "hostport")
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
                                   Logger *       logger,
                                   std::string    cfgname,
                                   std::string    prefix)
{
	try {
		auth_dbname = config->get_string((prefix + "auth_dbname").c_str());
		username    = config->get_string((prefix + "auth_username").c_str());
		password    = config->get_string((prefix + "auth_password").c_str());
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
	return new mongocxx::client(conn_uri_);
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
	}
	default: {
		logger->log_info(component, "%smode:   connection", indent);
	} break;
	}
	logger->log_info(component, "%suri:    %s", indent, conn_uri_.to_string().c_str());
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
