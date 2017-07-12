
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
#include "mongodb_client_config.h"

#ifdef HAVE_MONGODB_VERSION_H
#  include <mongo/client/init.h>
#endif

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

	init_client_configs();
	
	if (client_configs_.empty()) {
		throw Exception("No enabled MongoDB configurations found");
	}
}


void
MongoDBThread::init_client_configs()
{
	std::set<std::string> ignored_configs;
	std::string prefix = "/plugins/mongodb/clients/";

	std::unique_ptr<Configuration::ValueIterator> i(config->search(prefix.c_str()));
	while (i->next()) {
		std::string cfg_name = std::string(i->path()).substr(prefix.length());
		cfg_name = cfg_name.substr(0, cfg_name.find("/"));

		if ( (client_configs_.find(cfg_name) == client_configs_.end()) &&
		     (ignored_configs.find(cfg_name) == ignored_configs.end()) ) {

			std::string cfg_prefix = prefix + cfg_name + "/";

			try {
				MongoDBClientConfig *conf = new MongoDBClientConfig(config, logger, cfg_name, cfg_prefix);
				if (conf->is_enabled()) {
					client_configs_[cfg_name] = conf;
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
}

void
MongoDBThread::finalize()
{
	for (auto c : client_configs_) {
		delete c.second;
	}
	client_configs_.clear();
}


void
MongoDBThread::loop()
{
}

mongo::DBClientBase *
MongoDBThread::create_client(const std::string &config_name)
{
	const std::string cname{config_name.empty() ? "default" : config_name};

	if (client_configs_.find(cname) != client_configs_.end()) {
		if (! client_configs_[cname]->is_enabled()) {
			throw Exception("MongoDB config '%s' is not marked enabled", cname);
		}
		return client_configs_[cname]->create_client();
	} else {
		throw Exception("No MongoDB config named '%s' exists", cname);
	}
}

void
MongoDBThread::delete_client(mongo::DBClientBase *client)
{
	delete client;
}
