/***************************************************************************
 *  lookup_estimator.cpp - Estimate skill exec times by random db lookups
 *
 *  Created: Tue 24 Jan 2020 11:25:31 CET 16:35
 *  Copyright  2020  Tarik Viehmann <viehmann@kbsg.rwth-aachen.de>
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

#include "lookup_estimator.h"

#include <aspect/logging.h>
#include <config/yaml.h>
#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>

#include <bsoncxx/builder/basic/document.hpp>
#include <bsoncxx/builder/basic/kvp.hpp>
#include <bsoncxx/exception/exception.hpp>
#include <chrono>
#include <mongocxx/client.hpp>
#include <mongocxx/exception/operation_exception.hpp>
#include <thread>

namespace fawkes {

/** @class LookupEstimator
 * Estimate the execution time of skills by drawing a random sample from a
 * set of possible values stored in a mongodb database.
 */

/** Constructor.
 * @param mongo_connection_manager The mongodb manager to connect to a lookup collection
 * @param config The config to retrieve database related info and the skills to estimates
 * @param cfg_prefix The config prefix under which the estimator-specific configurations are found
 * @param logger The logger to inform about client connection status
 */
LookupEstimator::LookupEstimator(MongoDBConnCreator *mongo_connection_manager,
                                 Configuration *     config,
                                 const std::string & cfg_prefix,
                                 Logger *            logger)
: ExecutionTimeEstimator(config, cfg_prefix),
  mongo_connection_manager_(mongo_connection_manager),
  logger_(logger),
  fully_match_args_(config_, cfg_prefix_, "fully-match-args", true),
  include_failures_(config, cfg_prefix_, "include-failures", false),
  instance_(
    config_->get_string_or_default((std::string(cfg_prefix_) + "/instance").c_str(), "default")),
  database_(
    config_->get_string_or_default((std::string(cfg_prefix_) + "/database").c_str(), "skills")),
  collection_(config_->get_string_or_default((std::string(cfg_prefix_) + "/collection").c_str(),
                                             "exec_times"))
{
	logger_->log_info(logger_name_,
	                  "Using instance %s, database %s, collection %s",
	                  instance_.c_str(),
	                  database_.c_str(),
	                  collection_.c_str());
}

void
LookupEstimator::init()
{
	std::string  mongo_cfg_prefix = "/plugins/mongodb/instances/" + instance_ + "/";
	unsigned int startup_grace_period =
	  config_->get_uint_or_default((mongo_cfg_prefix + "startup-grace-period").c_str(), 10);
	logger_->log_info(logger_name_, "Connect to mongodb %s instance", instance_.c_str());
	unsigned int startup_tries = 0;
	for (; startup_tries < startup_grace_period * 2; ++startup_tries) {
		try {
			mongodb_client_lookup_ = mongo_connection_manager_->create_client(instance_);
			logger_->log_info(logger_name_, "Successfully connected to %s instance", instance_.c_str());
			return;
		} catch (fawkes::Exception &) {
			using namespace std::chrono_literals;
			logger_->log_info(logger_name_, "Waiting for mongodb %s instance", instance_.c_str());
			std::this_thread::sleep_for(500ms);
		}
	}
	logger_->log_error(logger_name_, "Failed to connect to mongodb %s instance", instance_.c_str());
}

bool
LookupEstimator::can_provide_exec_time(const Skill &skill) const
{
	// lock as mongocxx::client is not thread-safe
	MutexLocker lock(mutex_);
	// if all skills should be looked up by default, then the skills_ contain
	// those skills that should not be estimated via lookup
	try {
		using bsoncxx::builder::basic::document;
		using bsoncxx::builder::basic::kvp;

		document query = get_skill_query(skill);
		query.append(kvp("outcome", static_cast<int>(SkillerInterface::SkillStatusEnum::S_FINAL)));
		bsoncxx::stdx::optional<bsoncxx::document::value> found_entry =
		  mongodb_client_lookup_->database(database_)[collection_].find_one(query.view());
		return found_entry.has_value();
	} catch (mongocxx::operation_exception &e) {
		std::string error =
		  std::string("Error trying to lookup " + skill.skill_name + "\n Exception: " + e.what());
		logger_->log_error(logger_name_, "%s", error.c_str());
		return false;
	}
}

float
LookupEstimator::get_execution_time(const Skill &skill)
{
	using bsoncxx::builder::basic::document;
	using bsoncxx::builder::basic::kvp;
	// pipeline to pick a random sample out of all documents with matching name
	// field
	document query = get_skill_query(skill);
	if (!get_property(include_failures_)) {
		query.append(kvp("outcome", static_cast<int>(SkillerInterface::SkillStatusEnum::S_FINAL)));
	}
	mongocxx::pipeline p{};
	p.match(query.view());
	p.sample(1);

	// default values in case lookup fails
	error_   = "";
	outcome_ = SkillerInterface::SkillStatusEnum::S_FINAL;

	// lock as mongocxx::client is not thread-safe
	MutexLocker lock(mutex_);
	try {
		if (get_property(include_failures_)) {
			query.append(kvp("outcome", (int)SkillerInterface::SkillStatusEnum::S_FINAL));
		}
		mongocxx::cursor sample_cursor =
		  mongodb_client_lookup_->database(database_)[collection_].aggregate(p);
		auto  doc = *(sample_cursor.begin());
		float res = 0.f;
		switch (doc[duration_field_].get_value().type()) {
		case bsoncxx::type::k_double:
			res = static_cast<float>(doc[duration_field_].get_double().value);
			break;
		case bsoncxx::type::k_int32:
			res = static_cast<float>(doc[duration_field_].get_int32().value);
			break;
		default:
			throw fawkes::Exception(("Unexpected type "
			                         + bsoncxx::to_string(doc[duration_field_].get_value().type())
			                         + " when looking up skill exec duration.")
			                          .c_str());
		}
		error_   = doc["error"].get_utf8().value.to_string();
		outcome_ = SkillerInterface::SkillStatusEnum(doc["outcome"].get_int32().value);
		return res / speed_;
	} catch (mongocxx::operation_exception &e) {
		std::string error =
		  std::string("Error for lookup of " + skill.skill_name + "\n Exception: " + e.what());
		logger_->log_error(logger_name_, "%s", error.c_str());
		throw;
	}
}

std::pair<SkillerInterface::SkillStatusEnum, std::string>
LookupEstimator::execute(const Skill &skill)
{
	return make_pair(outcome_, error_);
}

bsoncxx::builder::basic::document
LookupEstimator::get_skill_query(const Skill &skill) const
{
	using bsoncxx::builder::basic::document;
	using bsoncxx::builder::basic::kvp;
	// pipeline to pick a random sample out of all documents with matching name
	// field
	document query = document();
	query.append(kvp(skill_name_field_, skill.skill_name));
	if (get_property(fully_match_args_)) {
		for (const auto &skill_arg : skill.skill_args) {
			query.append(kvp("args." + skill_arg.first, skill_arg.second));
		}
	} else if (active_whitelist_entry_ != whitelist_.end()) {
		for (const auto &skill_arg : active_whitelist_entry_->second.skill_args) {
			query.append(kvp("args." + skill_arg.first, skill_arg.second));
		}
	}
	return query;
}

} // namespace fawkes
