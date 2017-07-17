
/***************************************************************************
 *  mongodb_instance_config.h - MongoDB instance configuration
 *
 *  Created: Wed Jul 12 14:31:10 2017
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

#ifndef __PLUGINS_MONGODB_MONGODB_INSTANCE_CONFIG_H_
#define __PLUGINS_MONGODB_MONGODB_INSTANCE_CONFIG_H_

#include <string>
#include <vector>
#include <memory>

namespace fawkes {
	class Configuration;
	class Logger;
	class SubProcess;
}

/** Client configuration. */
class MongoDBInstanceConfig
{
 public:
	MongoDBInstanceConfig(fawkes::Configuration *config, fawkes::Logger *logger,
	                      std::string cfgname, std::string prefix);

	/** Check if configuration is enabled.
	 * @return true if configuration is enabled, false otherwise
	 */
	bool is_enabled() const { return enabled_; }

	void log(fawkes::Logger *logger, const char *component, const char *indent);

	void start_mongod();
	void kill_mongod();

	std::string command_line() const;
	unsigned int termination_grace_period() const;

	
 private:
	std::string  logcomp_;
	bool check_alive();

 private:
	bool         enabled_;

	std::string  config_name_;
	unsigned int startup_grace_period_;
	unsigned int termination_grace_period_;
	bool         clear_data_on_termination_;
	unsigned int port_;
	std::string  data_path_;
	std::string  log_path_;
	bool         log_append_;
	std::string  replica_set_;
	unsigned int oplog_size_;

	fawkes::Logger *logger_;

	bool running_;
	std::vector<std::string> argv_;
	std::shared_ptr<fawkes::SubProcess> proc_;
	std::string command_line_;
};

#endif
