
/***************************************************************************
 *  mongodb_client_config.h - MongoDB configuration for a single client
 *
 *  Created: Wed Jul 12 13:43:35 2017
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

#ifndef __PLUGINS_MONGODB_MONGODB_CLIENT_CONFIG_H_
#define __PLUGINS_MONGODB_MONGODB_CLIENT_CONFIG_H_

#include <mongo/client/dbclient.h>

#include <string>
#include <vector>

namespace fawkes {
	class Configuration;
	class Logger;
}

/** Client configuration. */
class MongoDBClientConfig
{
 public:
	/** Connection mode enumeration. */
	typedef enum {
		CONNECTION,		/**< connect to single node */
		REPLICA_SET,	/**< connect to replica set */
	} ConnectionMode;

	MongoDBClientConfig(fawkes::Configuration *config, fawkes::Logger *logger,
	                    std::string cfgname, std::string prefix);
	mongo::DBClientBase * create_client();

	/** Check if configuration is enabled.
	 * @return true if configuration is enabled, false otherwise
	 */
	bool is_enabled() const { return enabled_; }

	std::string hostport() const;
	
	void log(fawkes::Logger *logger, const char *component, const char *indent);

	ConnectionMode mode() const;
	
 private:
	void read_authinfo(fawkes::Configuration *config, fawkes::Logger *logger,
	                   std::string cfgname, std::string prefix);

 private:
	std::string                     logcomp_;
	bool                            enabled_;
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

#endif
