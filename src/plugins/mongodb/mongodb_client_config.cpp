
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
MongoDBClientConfig::MongoDBClientConfig(Configuration *config, Logger *logger,
                                         std::string cfgname, std::string prefix)
{
	logcomp_ = "MongoDBClient|" + cfgname;

	enabled_ = false;
	try {
		enabled_ = config->get_bool((prefix + "enabled").c_str());
	} catch (Exception &e) {}

	std::string mode = "connection";
	try {
		mode = config->get_string((prefix + "mode").c_str());
	} catch (Exception &e) {
		logger->log_info(logcomp_.c_str(), "MongoDB config '%s' specifies no client "
		                 "mode, assuming 'connection'.", cfgname.c_str());
	}

	if (mode == "replica_set" || mode == "replicaset") {
		mode_ = REPLICA_SET;
		replicaset_name_ = config->get_string((prefix + "name").c_str());

		std::vector<std::string> hosts = config->get_strings(prefix + "hosts");
		std::transform(hosts.begin(), hosts.end(),
		               std::back_inserter(replicaset_hostports_),
		               [](const std::string &s) -> mongo::HostAndPort
		               { return mongo::HostAndPort(s); });

	} else if (mode == "sync_cluster" || mode == "synccluster") {
		throw Exception("sync_cluster connections are no longer supported");

	} else {
		mode_ = CONNECTION;

		conn_hostport_ =
			mongo::HostAndPort(config->get_string(prefix + "hostport"));
	}
}

/** Read authentication info for given configuration.
 * This will first try to read the fields auth_dbname, auth_username, and
 * auth_password. If that fails, the auth/ subdirectory is crawled for subtrees
 * that contain the just named entries.
 * @param config configuration to query
 * @param logger logger for info messages
 * @param cfgname configuration name
 * @param prefix configuration path prefix
 */
void
MongoDBClientConfig::read_authinfo(Configuration *config, Logger *logger,
                                         std::string cfgname, std::string prefix)
{
	std::set<std::string> authinfos;

	try {
		std::string dbname   = config->get_string((prefix + "auth_dbname").c_str());
		std::string username = config->get_string((prefix + "auth_username").c_str());
		std::string password = config->get_string((prefix + "auth_password").c_str());
		auth_infos_.push_back(AuthInfo(dbname, username, password));
	} catch (Exception &e) {
		logger->log_info(logcomp_.c_str(), "No default authentication info for "
		                 "MongoDB client '%s'", cfgname.c_str());
	}

	std::unique_ptr<Configuration::ValueIterator>
		i(config->search((prefix + "auth/").c_str()));
	while (i->next()) {
		std::string auth_name = std::string(i->path()).substr(prefix.length());
		auth_name = auth_name.substr(0, auth_name.find("/"));

		if (authinfos.find(auth_name) == authinfos.end()) {
			try {
				std::string ap = prefix + auth_name + "/";
				std::string dbname   = config->get_string((ap + "auth_dbname").c_str());
				std::string username = config->get_string((ap + "auth_username").c_str());
				std::string password = config->get_string((ap + "auth_password").c_str());
				auth_infos_.push_back(AuthInfo(dbname, username, password));
			} catch (Exception &e) {
				logger->log_info(logcomp_.c_str(), "Incomplete extended auth info '%s' "
				                 "for MongoDB client '%s'",
				                 auth_name.c_str(), cfgname.c_str());
			}
		}
	}
}
					 
/** Create MongoDB client for this configuration.
 * @return MongoDB client
 */
mongo::DBClientBase *
MongoDBClientConfig::create_client()
{
	mongo::DBClientBase *client;
	std::string errmsg;

	switch (mode_) {
	case REPLICA_SET:
		{
			mongo::DBClientReplicaSet *repset =
				new mongo::DBClientReplicaSet(replicaset_name_, replicaset_hostports_);
			client = repset;
			if (! repset->connect())  throw Exception("Cannot connect to database");
			std::list<AuthInfo>::iterator ai;
			for (ai = auth_infos_.begin(); ai != auth_infos_.end(); ++ai) {
				if (!repset->auth(ai->dbname, ai->username, ai->clearpwd, errmsg, false)) {
					throw Exception("Authenticating for %s as %s failed: %s",
					                ai->dbname.c_str(), ai->username.c_str(),
					                errmsg.c_str());
				}
			}
		}
		break;

	default:
		{
			mongo::DBClientConnection *clconn = 
				new mongo::DBClientConnection(/* auto reconnect */ true);
			client = clconn;
			std::string errmsg;
			if (! clconn->connect(conn_hostport_, errmsg)) {
				throw Exception("Could not connect to MongoDB at %s: %s\n"
				                "You probably forgot to start/enable the mongod service",
				                conn_hostport_.toString().c_str(), errmsg.c_str());
			}
			std::list<AuthInfo>::iterator ai;
			for (ai = auth_infos_.begin(); ai != auth_infos_.end(); ++ai) {
				if (!clconn->auth(ai->dbname, ai->username, ai->clearpwd, errmsg, false)) {
					throw Exception("Authenticating for %s as %s failed: %s",
					                ai->dbname.c_str(), ai->username.c_str(),
					                errmsg.c_str());
				}
			}
		}
		break;
	}

	return client;
}


/** Write client configuration information to log.
 * @param logger logger to write to
 * @param component component to pass to logger
 * @param indent indentation to put before each string
 */
void
MongoDBClientConfig::log(Logger *logger, const char *component,
                         const char *indent)
{
	switch (mode_) {
	case REPLICA_SET:
		{
			logger->log_info(component, "%smode:   replica set", indent);
			logger->log_info(component, "%shosts:", indent);
			std::vector<mongo::HostAndPort>::iterator i;
			for (i = replicaset_hostports_.begin();
			     i != replicaset_hostports_.end();
			     ++i)
			{
				logger->log_info(component, "%s  - %s:", indent, i->toString().c_str());
			}

			if (! auth_infos_.empty()) {
				logger->log_info(component, "%sauth infos:", indent);
				std::list<AuthInfo>::iterator a;
				for (a = auth_infos_.begin(); a != auth_infos_.end(); ++a) {
					logger->log_info(component, "%s  - %s @ %s", indent, a->username.c_str(),
					                 a->dbname.c_str());
				}
			}
		}
		break;

	default:
		{
			logger->log_info(component, "%smode:   connection", indent);
			logger->log_info(component, "%shost:   %s", indent,
			                 conn_hostport_.toString().c_str());
			if (! auth_infos_.empty()) {
				logger->log_info(component, "%sauth infos:", indent);
				std::list<AuthInfo>::iterator a;
				for (a = auth_infos_.begin(); a != auth_infos_.end(); ++a) {
					logger->log_info(component, "%s  - %s @ %s", indent, a->username.c_str(),
					                 a->dbname.c_str());
				}
			}
		}
		break;
	}
}


/** Get host and port of configuration.
 * @return string of the form "host:port"
 */
std::string
MongoDBClientConfig::hostport() const
{
	return conn_hostport_.toString();
}


/** Get client configuration mode.
 * @return mode, connection or replica set
 */
MongoDBClientConfig::ConnectionMode
MongoDBClientConfig::mode() const
{
	return mode_;
}
