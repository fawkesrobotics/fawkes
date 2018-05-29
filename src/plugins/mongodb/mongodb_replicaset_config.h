
/***************************************************************************
 *  mongodb_replicaset_config.h - MongoDB replica set configuration
 *
 *  Created: Thu Jul 13 10:23:33 2017
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

#ifndef __PLUGINS_MONGODB_MONGODB_REPLICASET_CONFIG_H_
#define __PLUGINS_MONGODB_MONGODB_REPLICASET_CONFIG_H_

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/clock.h>
#include <aspect/blackboard.h>

#include <interfaces/MongoDBManagedReplicaSetInterface.h>

#include <string>
#include <vector>
#include <memory>

#include <mongo/bson/bson.h>
#include <mongo/client/dbclient.h>

namespace fawkes {
	class Configuration;
	class TimeWait;
}

namespace mongo {
	class DBClientBase;
}

class MongoDBReplicaSetConfig
: public fawkes::Thread,
	public fawkes::LoggingAspect,
	public fawkes::ClockAspect,
	public fawkes::BlackBoardAspect
{
 public:
	MongoDBReplicaSetConfig(fawkes::Configuration *config,
	                        std::string cfgname, std::string prefix,
	                        std::shared_ptr<mongo::DBClientBase> bootstrap_client,
	                        std::string bootstrap_database);

	/** Check if configuration is enabled.
	 * @return true if configuration is enabled, false otherwise
	 */
	bool is_enabled() const { return enabled_; }

	virtual void init();
	virtual void loop();
	virtual void finalize();

 private:
	bool leader_elect(bool force = false);
	void leader_resign();

	struct ReplicaSetStatus {
		fawkes::MongoDBManagedReplicaSetInterface::ReplicaSetMemberStatus   member_status;
		fawkes::MongoDBManagedReplicaSetInterface::ReplicaSetPrimaryStatus  primary_status;
		std::string                                                         error_msg;

		bool operator!=(const ReplicaSetStatus& other) const {
			return member_status != other.member_status ||
				primary_status != other.primary_status ||
				error_msg != other.error_msg;
		}
	};

	ReplicaSetStatus rs_status(mongo::BSONObj &reply);
	void rs_init();
	void rs_monitor(const mongo::BSONObj &reply);
	bool check_alive(const std::string &h);
	bool rs_get_config(mongo::BSONObj &rs_config);

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
	bool         enabled_;

	std::string  config_name_;

	std::shared_ptr<mongo::DBClientBase> bootstrap_client_;
	mongo::BSONObj leader_elec_query_;
	mongo::BSONObj leader_elec_query_force_;
	mongo::BSONObj leader_elec_update_;
	std::string bootstrap_database_;
	std::string bootstrap_collection_;
	std::string bootstrap_ns_;

	std::string local_client_cfg_;
	std::shared_ptr<mongo::DBClientBase> local_client_;
	std::string local_hostport_;
	std::set<std::string> hosts_;

	bool is_leader_;
	float loop_interval_;
	int leader_expiration_;
  fawkes::TimeWait *timewait_;

  ReplicaSetStatus last_status_;

  fawkes::MongoDBManagedReplicaSetInterface*  rs_status_if_;
};

#endif
