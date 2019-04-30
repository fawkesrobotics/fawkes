
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

#ifndef _PLUGINS_MONGODB_MONGODB_REPLICASET_CONFIG_H_
#define _PLUGINS_MONGODB_MONGODB_REPLICASET_CONFIG_H_

#include <aspect/blackboard.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <core/threading/thread.h>
#include <interfaces/MongoDBManagedReplicaSetInterface.h>

#include <bsoncxx/json.hpp>
#include <memory>
#include <mongocxx/client.hpp>
#include <set>
#include <string>
#include <vector>

namespace fawkes {
class TimeWait;
} // namespace fawkes

namespace mongo {
class DBClientBase;
}

class MongoDBReplicaSetConfig : public fawkes::Thread,
                                public fawkes::LoggingAspect,
                                public fawkes::ConfigurableAspect,
                                public fawkes::ClockAspect,
                                public fawkes::BlackBoardAspect
{
public:
	MongoDBReplicaSetConfig(const std::string &cfgname,
	                        const std::string &prefix,
	                        const std::string &bootstrap_prefix);

	/** Check if configuration is enabled.
	 * @return true if configuration is enabled, false otherwise
	 */
	bool
	is_enabled() const
	{
		return enabled_;
	}

	virtual void init();
	virtual void loop();
	virtual void finalize();

private:
	void bootstrap();
	bool leader_elect(bool force = false);
	void leader_resign();

	struct ReplicaSetStatus
	{
		fawkes::MongoDBManagedReplicaSetInterface::ReplicaSetMemberStatus  member_status;
		fawkes::MongoDBManagedReplicaSetInterface::ReplicaSetPrimaryStatus primary_status;
		std::string                                                        error_msg;

		bool
		operator!=(const ReplicaSetStatus &other) const
		{
			return member_status != other.member_status || primary_status != other.primary_status
			       || error_msg != other.error_msg;
		}
	};

	// TODO: update signature
	ReplicaSetStatus rs_status(bsoncxx::document::value &reply);
	void             rs_init();
	void             rs_monitor(const bsoncxx::document::view &reply);
	bool             check_alive(const std::string &h);
	bool             rs_get_config(bsoncxx::document::value &rs_config);

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
protected:
	virtual void
	run()
	{
		Thread::run();
	}

private:
	bool enabled_;

	std::string config_name_;

	std::shared_ptr<mongocxx::client> bootstrap_client_;
	bsoncxx::document::value          leader_elec_query_;
	bsoncxx::document::value          leader_elec_query_force_;
	bsoncxx::document::value          leader_elec_update_;
	std::string                       prefix_;
	std::string                       bootstrap_prefix_;
	std::string                       bootstrap_database_;
	std::string                       bootstrap_ns_;

	std::string                       local_client_cfg_;
	std::shared_ptr<mongocxx::client> local_client_;
	std::string                       local_hostport_;
	std::set<std::string>             hosts_;

	bool              is_leader_;
	float             loop_interval_;
	int               leader_expiration_;
	fawkes::TimeWait *timewait_;

	ReplicaSetStatus last_status_;

	fawkes::MongoDBManagedReplicaSetInterface *rs_status_if_;
};

#endif
