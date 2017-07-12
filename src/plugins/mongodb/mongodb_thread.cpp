
/***************************************************************************
 *  mongodb_thread.cpp - MongoDB Thread
 *
 *  Created: Sun Dec 05 23:32:13 2010
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

#include "mongodb_thread.h"

#ifdef HAVE_MONGODB_VERSION_H
#  include <mongo/client/init.h>
#endif

using namespace mongo;
using namespace fawkes;

/** Client configuration. */
class MongoDBThread::ClientConf
{
public:
	/** Connection mode enumeration. */
	typedef enum {
		CONNECTION,		/**< connect to single node */
		REPLICA_SET,	/**< connect to replica set */
	} ConnectionMode;

	ClientConf(fawkes::Configuration *config, fawkes::Logger *logger,
	           std::string cfgname, std::string prefix);
	mongo::DBClientBase * create_client();

	/** Check if configuration is active.
	 * @return true if configuration is active, false otherwise
	 */
	bool is_active() const { return active_; }

	void log(Logger *logger, const char *component, const char *indent);

private:
	void read_authinfo(Configuration *config, Logger *logger,
	                   std::string cfgname, std::string prefix);

private:
	std::string                     logcomp_;
	bool                            active_;
	ConnectionMode                  mode_;
	mongo::HostAndPort              conn_hostport_;
	std::vector<mongo::HostAndPort> replicaset_hostports_;
	std::string                     replicaset_name_;

	/// @cond INTERNALS
	typedef struct _AuthInfo {
		_AuthInfo(std::string dbname, std::string username, std::string clearpwd)
		{ this->dbname = dbname; this->username = username;
			this->clearpwd = clearpwd; }
		std::string dbname;
		std::string username;
		std::string clearpwd;
	} AuthInfo;    
	/// @endcond

	std::list<AuthInfo> auth_infos_;
};


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
	: Thread("MongoDBThread", Thread::OPMODE_WAITFORWAKEUP),
	  AspectProviderAspect(&mongodb_aspect_inifin_),
	  mongodb_aspect_inifin_(this)
{
}


/** Destructor. */
MongoDBThread::~MongoDBThread()
{
}


void
MongoDBThread::init()
{
#ifdef HAVE_MONGODB_VERSION_H
	mongo::client::initialize();
#endif

	std::set<std::string> ignored_configs;

	std::string prefix = "/plugins/mongodb/clients/";

	std::unique_ptr<Configuration::ValueIterator> i(config->search(prefix.c_str()));
	while (i->next()) {
		std::string cfg_name = std::string(i->path()).substr(prefix.length());
		cfg_name = cfg_name.substr(0, cfg_name.find("/"));

		if ( (configs_.find(cfg_name) == configs_.end()) &&
		     (ignored_configs.find(cfg_name) == ignored_configs.end()) ) {

			std::string cfg_prefix = prefix + cfg_name + "/";

			try {
				ClientConf *conf = new ClientConf(config, logger, cfg_name, cfg_prefix);
				if (conf->is_active()) {
					configs_[cfg_name] = conf;
					logger->log_info(name(), "Added MongoDB client configuration %s",
					                 cfg_name.c_str());
					conf->log(logger, name(), "  ");
				} else {
					logger->log_info(name(), "Ignoring disabled MongoDB client "
					                 "configuration %s", cfg_name.c_str());
					delete conf;
					ignored_configs.insert(cfg_name);
				}
			} catch (Exception &e) {
				logger->log_warn(name(), "Invalid MongoDB client config %s, ignoring, "
				                 "exception follows.", cfg_name.c_str());
				ignored_configs.insert(cfg_name);
			}
		}
	}

	if (configs_.empty()) {
		throw Exception("No active MongoDB configurations found");
	}
}


void
MongoDBThread::finalize()
{
	std::map<std::string, ClientConf *>::iterator i;
	for (i = configs_.begin(); i != configs_.end(); ++i) {
		delete i->second;
	}
	configs_.clear();
}


void
MongoDBThread::loop()
{
}

mongo::DBClientBase *
MongoDBThread::create_client(const std::string &config_name)
{
	const std::string cname{config_name.empty() ? "default" : config_name};

	if (configs_.find(cname) != configs_.end()) {
		if (! configs_[cname]->is_active()) {
			throw Exception("MongoDB config '%s' is not marked active", cname);
		}
		return configs_[cname]->create_client();
	} else {
		throw Exception("No MongoDB config named '%s' exists", cname);
	}
}

void
MongoDBThread::delete_client(mongo::DBClientBase *client)
{
	delete client;
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
MongoDBThread::ClientConf::read_authinfo(Configuration *config, Logger *logger,
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
					 
/** Constructor.
 * This will read the given configuration.
 * @param config configuration to query
 * @param logger logger for info messages
 * @param cfgname configuration name
 * @param prefix configuration path prefix
 */
MongoDBThread::ClientConf::ClientConf(Configuration *config, Logger *logger,
                                      std::string cfgname, std::string prefix)
{
	logcomp_ = "MongoDB ClientConf " + cfgname;

	active_ = false;
	try {
		active_ = config->get_bool((prefix + "active").c_str());
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

		std::unique_ptr<Configuration::ValueIterator>
			i(config->search((prefix + "hosts/").c_str()));
		while (i->next()) {
			if (i->is_string()) {
				replicaset_hostports_.push_back(HostAndPort(i->get_string()));
			}
		}

	} else if (mode == "sync_cluster" || mode == "synccluster") {
		throw Exception("sync_cluster connections are no longer supported");

	} else {
		mode_ = CONNECTION;

		conn_hostport_ =
			HostAndPort(config->get_string((prefix + "hostport").c_str()));
	}
}

/** Create MongoDB client for this configuration.
 * @return MongoDB client
 */
mongo::DBClientBase *
MongoDBThread::ClientConf::create_client()
{
	mongo::DBClientBase *client;
	std::string errmsg;

	switch (mode_) {
	case REPLICA_SET:
		{
			DBClientReplicaSet *repset =
				new DBClientReplicaSet(replicaset_name_, replicaset_hostports_);
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
			DBClientConnection *clconn = 
				new DBClientConnection(/* auto reconnect */ true);
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
MongoDBThread::ClientConf::log(Logger *logger, const char *component,
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
