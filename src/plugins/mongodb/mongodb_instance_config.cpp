
/***************************************************************************
 *  mongodb_instance_config.cpp - MongoDB instance configuration
 *
 *  Created: Wed Jul 12 14:33:02 2017
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

#include "mongodb_instance_config.h"

#include <config/config.h>
#include <logging/logger.h>
#include <utils/sub_process/proc.h>

#include <boost/filesystem.hpp>
#include <chrono>
#include <numeric>

using namespace fawkes;
using namespace std::chrono_literals;

/** @class MongoDBInstanceConfig "mongodb_client_config.h"
 * MongoDB Instances Configuration.
 * Configure a single mongod instance that can be run as a sub-process.
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
MongoDBInstanceConfig::MongoDBInstanceConfig(Configuration *config, Logger *logger,
                                             std::string cfgname, std::string prefix)
{
	logcomp_ = "MongoDBInstance|" + cfgname;
	config_name_ = cfgname;

	running_ = false;
	
	enabled_ = false;
	try {
		enabled_ = config->get_bool(prefix + "enabled");
	} catch (Exception &e) {}

	if (enabled_) {
		termination_grace_period_ = config->get_uint(prefix + "termination-grace-period");
		clear_data_on_termination_ = config->get_bool(prefix + "clear-data-on-termination");
		port_ = config->get_uint(prefix + "port");
		data_path_ = config->get_string(prefix + "data-path");
		log_path_ = config->get_string(prefix + "log/path");
		log_append_ = config->get_bool(prefix + "log/append");
		try {
			replica_set_ = config->get_string(prefix + "replica-set");;
		} catch (Exception &e) {} // ignored, no replica set
		if (! replica_set_.empty()) {
			oplog_size_ = 0;
			try {
				oplog_size_ = config->get_uint(prefix + "oplog-size");
			} catch (Exception &e) {} // ignored, use default
		}
	}

	argv_ =	{ "mongod", "--port", std::to_string(port_), "--dbpath", data_path_ };

	if (! log_path_.empty()) {
		if (log_append_) {
			argv_.push_back("--logappend");
		}
		argv_.push_back("--logpath");
		argv_.push_back(log_path_);
	}

	if (! replica_set_.empty()) {
		argv_.push_back("--replSet");
		argv_.push_back(replica_set_);
		if (oplog_size_ > 0) {
			argv_.push_back("--oplogSize");
			argv_.push_back(std::to_string(oplog_size_));
		}
	}

	command_line_ =
		std::accumulate(std::next(argv_.begin()), argv_.end(), argv_.front(),
		                [](std::string &s, const std::string &a) { return s + " " + a; });

	logger_ = logger;
}


/** Write instance configuration information to log.
 * @param logger logger to write to
 * @param component component to pass to logger
 * @param indent indentation to put before each string
 */
void
MongoDBInstanceConfig::log(Logger *logger, const char *component,
                           const char *indent)
{
	if (enabled_) {
		logger->log_info(logcomp_.c_str(), "enabled: true");
		logger->log_info(logcomp_.c_str(), "TCP port: %u", port_);
		logger->log_info(logcomp_.c_str(), "Termination grace period: %u", termination_grace_period_);
		logger->log_info(logcomp_.c_str(), "clear data on termination: %s", clear_data_on_termination_ ? "yes" : "no");
		logger->log_info(logcomp_.c_str(), "data path: %s", data_path_.c_str());
		logger->log_info(logcomp_.c_str(), "log path: %s", log_path_.c_str());
		logger->log_info(logcomp_.c_str(), "log append: %s", log_append_ ? "yes" : "no");
		logger->log_info(logcomp_.c_str(), "replica set: %s",
		                 replica_set_.empty() ? "DISABLED" : replica_set_.c_str());
		if (! replica_set_.empty()) {
			logger->log_info(logcomp_.c_str(), "Op Log Size: %u MB", oplog_size_);
		}
	} else {
		logger->log_info(logcomp_.c_str(), "enabled: DISABLED");
	}
}


/** Get command line used to execute program.
 * @return command line to run mongod
 */
std::string
MongoDBInstanceConfig::command_line() const
{
	return command_line_;
}

/** Get termination grace period.
 * @return termination grace period
 */
unsigned int
MongoDBInstanceConfig::termination_grace_period() const
{
	return termination_grace_period_;
}


/** Start mongod. */
void
MongoDBInstanceConfig::start_mongod()
{
	if (running_)  return;
	
	try {
		boost::filesystem::create_directories(data_path_);
	} catch (boost::filesystem::filesystem_error &e) {
		throw Exception("Failed to create data path '%s' for mongod(%s): %s",
		                data_path_.c_str(), config_name_.c_str(), e.what());
	}

	if (! log_path_.empty()) {
		boost::filesystem::path p(log_path_);
		try {
			boost::filesystem::create_directories(p.parent_path());
		} catch (boost::filesystem::filesystem_error &e) {
			throw Exception("Failed to create log path '%s' for mongod(%s): %s",
			                p.parent_path().string().c_str(), config_name_.c_str(), e.what());
		}
	}
	
	std::string progname = "mongod(" + config_name_ + ")";
	proc_ = std::make_shared<SubProcess>(progname, "mongod", argv_, std::vector<std::string>{}, logger_);

	running_ = true;
}

/** Stop mongod.
 * This send a SIGINT and then wait for the configured grace period
 * before sending the TERM signal.
 */
void
MongoDBInstanceConfig::kill_mongod()
{
	if (proc_) {
		proc_->kill(SIGINT);
		for (unsigned i = 0; i < termination_grace_period_; ++i) {
			if (! proc_->alive())  break;
			std::this_thread::sleep_for(1s);
		}
		// This will send the term signal
		proc_.reset();
		running_ = false;
		if (clear_data_on_termination_) {
			try {
				boost::filesystem::remove_all(data_path_);
			} catch (boost::filesystem::filesystem_error &e) {
				throw Exception("Failed to create data path '%s' for mongod(%s): %s",
				                data_path_.c_str(), config_name_.c_str(), e.what());
			}			
		}
	}
}
