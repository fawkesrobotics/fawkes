
/***************************************************************************
 *  clips_robot_memory_thread.cpp - CLIPS feature to access the robot memory
 *
 *  Created: Mon Aug 29 15:41:47 2016
 *  Copyright  2016       Frederik Zwilling
 *             2013-2018  Tim Niemueller [www.niemueller.de]
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

#include "clips_robot_memory_thread.h"

#include <core/threading/mutex_locker.h>
#include <plugins/mongodb/utils.h>

#include <bsoncxx/document/element.hpp>
#include <bsoncxx/exception/exception.hpp>
#include <bsoncxx/types.hpp>
#include <mongocxx/exception/exception.hpp>

using namespace fawkes;

/** @class ClipsRobotMemoryThread 'clips_robot_memory_thread.h' 
 * CLIPS feature to access the robot memory.
 * MongoDB access through CLIPS first appeared in the RCLL referee box.
 * @author Tim Niemueller
 * @author Frederik Zwilling
 */

ClipsRobotMemoryThread::ClipsRobotMemoryThread()
: Thread("ClipsRobotMemoryThread", Thread::OPMODE_WAITFORWAKEUP),
  CLIPSFeature("robot_memory"),
  CLIPSFeatureAspect(this)
{
}

void
ClipsRobotMemoryThread::init()
{
}

void
ClipsRobotMemoryThread::loop()
{
}

void
ClipsRobotMemoryThread::finalize()
{
	envs_.clear();
	for (ClipsRmTrigger *trigger : clips_triggers_) {
		delete trigger;
	}
}

void
ClipsRobotMemoryThread::clips_context_init(const std::string &          env_name,
                                           LockPtr<CLIPS::Environment> &clips)
{
	envs_[env_name] = clips;
	logger->log_debug(name(), "Called to initialize environment %s", env_name.c_str());

	clips.lock();

	clips->add_function("bson-create",
	                    sigc::slot<CLIPS::Value>(
	                      sigc::mem_fun(*this, &ClipsRobotMemoryThread::clips_bson_create)));
	clips->add_function("bson-parse",
	                    sigc::slot<CLIPS::Value, std::string>(
	                      sigc::mem_fun(*this, &ClipsRobotMemoryThread::clips_bson_parse)));
	clips->add_function("bson-destroy",
	                    sigc::slot<void, void *>(
	                      sigc::mem_fun(*this, &ClipsRobotMemoryThread::clips_bson_destroy)));
	clips->add_function("bson-append",
	                    sigc::slot<void, void *, std::string, CLIPS::Value>(
	                      sigc::mem_fun(*this, &ClipsRobotMemoryThread::clips_bson_append)));
	clips->add_function("bson-append-regex",
	                    sigc::slot<void, void *, std::string, CLIPS::Value>(
	                      sigc::mem_fun(*this, &ClipsRobotMemoryThread::clips_bson_append_regex)));
	clips->add_function("bson-append-array",
	                    sigc::slot<void, void *, std::string, CLIPS::Values>(
	                      sigc::mem_fun(*this, &ClipsRobotMemoryThread::clips_bson_append_array)));
	clips->add_function("bson-array-start",
	                    sigc::slot<CLIPS::Value, void *, std::string>(
	                      sigc::mem_fun(*this, &ClipsRobotMemoryThread::clips_bson_array_start)));
	clips->add_function("bson-array-finish",
	                    sigc::slot<void, void *>(
	                      sigc::mem_fun(*this, &ClipsRobotMemoryThread::clips_bson_array_finish)));
	clips->add_function("bson-array-append",
	                    sigc::slot<void, void *, CLIPS::Value>(
	                      sigc::mem_fun(*this, &ClipsRobotMemoryThread::clips_bson_array_append)));

	clips->add_function("bson-append-time",
	                    sigc::slot<void, void *, std::string, CLIPS::Values>(
	                      sigc::mem_fun(*this, &ClipsRobotMemoryThread::clips_bson_append_time)));
	clips->add_function("bson-tostring",
	                    sigc::slot<std::string, void *>(
	                      sigc::mem_fun(*this, &ClipsRobotMemoryThread::clips_bson_tostring)));
	clips->add_function("robmem-insert",
	                    sigc::slot<void, std::string, void *>(
	                      sigc::mem_fun(*this, &ClipsRobotMemoryThread::clips_robotmemory_insert)));
	clips->add_function("robmem-upsert",
	                    sigc::slot<void, std::string, void *, CLIPS::Value>(
	                      sigc::mem_fun(*this, &ClipsRobotMemoryThread::clips_robotmemory_upsert)));
	clips->add_function("robmem-update",
	                    sigc::slot<void, std::string, void *, CLIPS::Value>(
	                      sigc::mem_fun(*this, &ClipsRobotMemoryThread::clips_robotmemory_update)));
	clips->add_function("robmem-replace",
	                    sigc::slot<void, std::string, void *, CLIPS::Value>(
	                      sigc::mem_fun(*this, &ClipsRobotMemoryThread::clips_robotmemory_replace)));
	clips->add_function("robmem-query",
	                    sigc::slot<CLIPS::Value, std::string, void *>(
	                      sigc::mem_fun(*this, &ClipsRobotMemoryThread::clips_robotmemory_query)));
	clips->add_function("robmem-remove",
	                    sigc::slot<void, std::string, void *>(
	                      sigc::mem_fun(*this, &ClipsRobotMemoryThread::clips_robotmemory_remove)));
	clips->add_function("robmem-query-sort",
	                    sigc::slot<CLIPS::Value, std::string, void *, void *>(
	                      sigc::mem_fun(*this,
	                                    &ClipsRobotMemoryThread::clips_robotmemory_query_sort)));
	clips->add_function("robmem-dump-collection",
	                    sigc::slot<CLIPS::Value, std::string, std::string>(
	                      sigc::mem_fun(*this,
	                                    &ClipsRobotMemoryThread::clips_robotmemory_dump_collection)));
	clips->add_function("robmem-restore-collection",
	                    sigc::slot<CLIPS::Value, std::string, std::string, std::string>(sigc::mem_fun(
	                      *this, &ClipsRobotMemoryThread::clips_robotmemory_restore_collection)));
	clips->add_function("robmem-cursor-destroy",
	                    sigc::slot<void, void *>(
	                      sigc::mem_fun(*this,
	                                    &ClipsRobotMemoryThread::clips_robotmemory_cursor_destroy)));
	clips->add_function("robmem-cursor-more",
	                    sigc::slot<CLIPS::Value, void *>(
	                      sigc::mem_fun(*this,
	                                    &ClipsRobotMemoryThread::clips_robotmemory_cursor_more)));
	clips->add_function("robmem-cursor-next",
	                    sigc::slot<CLIPS::Value, void *>(
	                      sigc::mem_fun(*this,
	                                    &ClipsRobotMemoryThread::clips_robotmemory_cursor_next)));
	clips->add_function("robmem-trigger-register",
	                    sigc::slot<CLIPS::Value, std::string, void *, std::string>(sigc::bind<0>(
	                      sigc::mem_fun(*this,
	                                    &ClipsRobotMemoryThread::clips_robotmemory_register_trigger),
	                      env_name)));
	clips->add_function("robmem-trigger-destroy",
	                    sigc::slot<void, void *>(
	                      sigc::mem_fun(*this,
	                                    &ClipsRobotMemoryThread::clips_robotmemory_destroy_trigger)));
	clips->add_function("bson-field-names",
	                    sigc::slot<CLIPS::Values, void *>(
	                      sigc::mem_fun(*this, &ClipsRobotMemoryThread::clips_bson_field_names)));
	clips->add_function("bson-has-field",
	                    sigc::slot<CLIPS::Value, void *, std::string>(
	                      sigc::mem_fun(*this, &ClipsRobotMemoryThread::clips_bson_has_field)));
	clips->add_function("bson-get",
	                    sigc::slot<CLIPS::Value, void *, std::string>(
	                      sigc::mem_fun(*this, &ClipsRobotMemoryThread::clips_bson_get)));
	clips->add_function("bson-get-array",
	                    sigc::slot<CLIPS::Values, void *, std::string>(
	                      sigc::mem_fun(*this, &ClipsRobotMemoryThread::clips_bson_get_array)));
	clips->add_function("bson-get-time",
	                    sigc::slot<CLIPS::Values, void *, std::string>(
	                      sigc::mem_fun(*this, &ClipsRobotMemoryThread::clips_bson_get_time)));
	clips->add_function("robmem-create-index",
	                    sigc::slot<void, std::string, void *>(
	                      sigc::mem_fun(*this,
	                                    &ClipsRobotMemoryThread::clips_robotmemory_create_index)));
	clips->add_function("robmem-create-unique-index",
	                    sigc::slot<void, std::string, void *>(sigc::mem_fun(
	                      *this, &ClipsRobotMemoryThread::clips_robotmemory_create_unique_index)));

	clips->add_function("robmem-mutex-create",
	                    sigc::slot<CLIPS::Value, std::string>(
	                      sigc::mem_fun(*this,
	                                    &ClipsRobotMemoryThread::clips_robotmemory_mutex_create)));
	clips->add_function("robmem-mutex-destroy",
	                    sigc::slot<CLIPS::Value, std::string>(
	                      sigc::mem_fun(*this,
	                                    &ClipsRobotMemoryThread::clips_robotmemory_mutex_destroy)));
	clips->add_function("robmem-mutex-try-lock",
	                    sigc::slot<CLIPS::Value, std::string, std::string>(
	                      sigc::mem_fun(*this,
	                                    &ClipsRobotMemoryThread::clips_robotmemory_mutex_try_lock)));
	clips->add_function("robmem-mutex-renew-lock",
	                    sigc::slot<CLIPS::Value, std::string, std::string>(sigc::mem_fun(
	                      *this, &ClipsRobotMemoryThread::clips_robotmemory_mutex_renew_lock)));
	clips->add_function("robmem-mutex-force-lock",
	                    sigc::slot<CLIPS::Value, std::string, std::string>(sigc::mem_fun(
	                      *this, &ClipsRobotMemoryThread::clips_robotmemory_mutex_force_lock)));
	clips->add_function("robmem-mutex-unlock",
	                    sigc::slot<CLIPS::Value, std::string, std::string>(
	                      sigc::mem_fun(*this,
	                                    &ClipsRobotMemoryThread::clips_robotmemory_mutex_unlock)));
	clips->add_function("robmem-mutex-setup-ttl",
	                    sigc::slot<CLIPS::Value, float>(
	                      sigc::mem_fun(*this,
	                                    &ClipsRobotMemoryThread::clips_robotmemory_mutex_setup_ttl)));
	clips->add_function("robmem-mutex-expire-locks",
	                    sigc::slot<CLIPS::Value, float>(sigc::mem_fun(
	                      *this, &ClipsRobotMemoryThread::clips_robotmemory_mutex_expire_locks)));

	clips->add_function("robmem-mutex-create-async",
	                    sigc::slot<CLIPS::Values, std::string>(sigc::mem_fun(
	                      *this, &ClipsRobotMemoryThread::clips_robotmemory_mutex_create_async)));
	clips->add_function("robmem-mutex-destroy-async",
	                    sigc::slot<CLIPS::Values, std::string>(sigc::mem_fun(
	                      *this, &ClipsRobotMemoryThread::clips_robotmemory_mutex_destroy_async)));
	clips->add_function(
	  "robmem-mutex-try-lock-async",
	  sigc::slot<CLIPS::Values, std::string, std::string>(sigc::bind<0>(
	    sigc::mem_fun(*this, &ClipsRobotMemoryThread::clips_robotmemory_mutex_try_lock_async),
	    env_name)));
	clips->add_function(
	  "robmem-mutex-renew-lock-async",
	  sigc::slot<CLIPS::Values, std::string, std::string>(sigc::bind<0>(
	    sigc::mem_fun(*this, &ClipsRobotMemoryThread::clips_robotmemory_mutex_renew_lock_async),
	    env_name)));
	clips->add_function("robmem-mutex-force-lock-async",
	                    sigc::slot<CLIPS::Values, std::string, std::string>(sigc::mem_fun(
	                      *this, &ClipsRobotMemoryThread::clips_robotmemory_mutex_force_lock_async)));
	clips->add_function("robmem-mutex-unlock-async",
	                    sigc::slot<CLIPS::Values, std::string, std::string>(sigc::mem_fun(
	                      *this, &ClipsRobotMemoryThread::clips_robotmemory_mutex_unlock_async)));
	clips->add_function(
	  "robmem-mutex-expire-locks-async",
	  sigc::slot<CLIPS::Value, float>(sigc::bind<0>(
	    sigc::mem_fun(*this, &ClipsRobotMemoryThread::clips_robotmemory_mutex_expire_locks_async),
	    env_name)));

	clips->build("(deffacts have-feature-mongodb (have-feature MongoDB))");

	//load helper functions written in CLIPS
	clips->batch_evaluate(SRCDIR "/robot-memory.clp");

	clips.unlock();
}

void
ClipsRobotMemoryThread::clips_context_destroyed(const std::string &env_name)
{
	envs_.erase(env_name);
	logger->log_debug(name(), "Removing environment %s", env_name.c_str());
}

CLIPS::Value
ClipsRobotMemoryThread::clips_bson_create()
{
	return CLIPS::Value(new bsoncxx::builder::basic::document());
}

CLIPS::Value
ClipsRobotMemoryThread::clips_bson_parse(std::string document)
{
	auto b = new bsoncxx::builder::basic::document();
	try {
		b->append(bsoncxx::builder::concatenate(bsoncxx::from_json(document)));
	} catch (bsoncxx::exception &e) {
		logger->log_error("MongoDB", "Parsing JSON doc failed: %s\n%s", e.what(), document.c_str());
	}
	return CLIPS::Value(b);
}

void
ClipsRobotMemoryThread::clips_bson_destroy(void *bson)
{
	auto b = static_cast<bsoncxx::builder::basic::document *>(bson);
	delete b;
}

std::string
ClipsRobotMemoryThread::clips_bson_tostring(void *bson)
{
	auto b = static_cast<bsoncxx::builder::basic::document *>(bson);
	return bsoncxx::to_json(b->view());
}

void
ClipsRobotMemoryThread::clips_bson_append(void *bson, std::string field_name, CLIPS::Value value)
{
	using namespace bsoncxx::builder;
	try {
		auto b = static_cast<basic::document *>(bson);
		switch (value.type()) {
		case CLIPS::TYPE_FLOAT: b->append(basic::kvp(field_name, value.as_float())); break;

		case CLIPS::TYPE_INTEGER:
			b->append(basic::kvp(field_name, static_cast<int64_t>(value.as_integer())));
			break;

		case CLIPS::TYPE_SYMBOL:
		case CLIPS::TYPE_INSTANCE_NAME:
		case CLIPS::TYPE_STRING: b->append(basic::kvp(field_name, value.as_string())); break;
		case CLIPS::TYPE_EXTERNAL_ADDRESS: {
			auto subb = static_cast<basic::document *>(value.as_address());
			b->append(basic::kvp(field_name, subb->view()));
		} break;

		default:
			logger->log_warn("RefBox", "Tried to add unknown type to BSON field %s", field_name.c_str());
			break;
		}
	} catch (bsoncxx::exception &e) {
		logger->log_error("MongoDB",
		                  "Failed to append array value to field %s: %s",
		                  field_name.c_str(),
		                  e.what());
	}
}

void
ClipsRobotMemoryThread::clips_bson_append_regex(void *       bson,
                                                std::string  field_name,
                                                CLIPS::Value regex_string)
{
	using namespace bsoncxx::builder;
	if (regex_string.type() != CLIPS::TYPE_STRING) {
		logger->log_error("MongoDB", "Regex string has to be of type string");
		return;
	}
	try {
		auto b = static_cast<basic::document *>(bson);
		b->append(basic::kvp(field_name, bsoncxx::types::b_regex{regex_string.as_string()}));
	} catch (bsoncxx::exception &e) {
		logger->log_error("MongoDB",
		                  "Failed to append regex to field %s: %s",
		                  field_name.c_str(),
		                  e.what());
	}
}

void
ClipsRobotMemoryThread::clips_bson_append_array(void *        bson,
                                                std::string   field_name,
                                                CLIPS::Values values)
{
	using namespace bsoncxx::builder;
	try {
		auto b = static_cast<basic::document *>(bson);

		b->append(basic::kvp(field_name, [&](basic::sub_array array) {
			for (auto value : values) {
				switch (value.type()) {
				case CLIPS::TYPE_FLOAT: array.append(value.as_float()); break;

				case CLIPS::TYPE_INTEGER: array.append(static_cast<int64_t>(value.as_integer())); break;

				case CLIPS::TYPE_SYMBOL:
				case CLIPS::TYPE_STRING:
				case CLIPS::TYPE_INSTANCE_NAME: array.append(value.as_string()); break;

				case CLIPS::TYPE_EXTERNAL_ADDRESS: {
					auto subb = static_cast<bsoncxx::builder::basic::document *>(value.as_address());
					array.append(subb->view());
				} break;

				default:
					logger->log_warn("MongoDB",
					                 "Tried to add unknown type to BSON array field %s",
					                 field_name.c_str());
					break;
				}
			}
		}));
	} catch (bsoncxx::exception &e) {
		logger->log_error("MongoDB",
		                  "Failed to append array value to field %s: %s",
		                  field_name.c_str(),
		                  e.what());
	}
}

CLIPS::Value
ClipsRobotMemoryThread::clips_bson_array_start(void *bson, std::string field_name)
{
	// With the new libmongocxx, we can no longer create an open array as
	// sub-field of another document.
	throw Exception("Not implemented");
	/*
	mongo::BSONObjBuilder *  b    = static_cast<mongo::BSONObjBuilder *>(bson);
	mongo::BufBuilder &      bb   = b->subarrayStart(field_name);
	mongo::BSONArrayBuilder *arrb = new mongo::BSONArrayBuilder(bb);
	return CLIPS::Value(arrb);
  */
}

void
ClipsRobotMemoryThread::clips_bson_array_finish(void *barr)
{
	throw Exception("Not implemented");
	/*
	mongo::BSONArrayBuilder *ab = static_cast<mongo::BSONArrayBuilder *>(barr);
	delete ab;
  */
}

void
ClipsRobotMemoryThread::clips_bson_array_append(void *barr, CLIPS::Value value)
{
	throw Exception("Not implemented");
	/*
	try {
		auto *ab = static_cast<mongo::BSONArrayBuilder *>(barr);
		switch (value.type()) {
		case CLIPS::TYPE_FLOAT: ab->append(value.as_float()); break;

		case CLIPS::TYPE_INTEGER: ab->append(value.as_integer()); break;

		case CLIPS::TYPE_SYMBOL:
		case CLIPS::TYPE_STRING:
		case CLIPS::TYPE_INSTANCE_NAME: ab->append(value.as_string()); break;

		case CLIPS::TYPE_EXTERNAL_ADDRESS: {
			mongo::BSONObjBuilder *subb = static_cast<mongo::BSONObjBuilder *>(value.as_address());
			ab->append(subb->asTempObj());
		} break;

		default: logger->log_warn("RefBox", "Tried to add unknown type to BSON array"); break;
		}
#ifdef HAVE_MONGODB_VERSION_H
	} catch (mongo::MsgAssertionException &e) {
#else
	} catch (mongo::AssertionException &e) {
#endif
		logger->log_error("MongoDB", "Failed to append to array: %s", e.what());
	}
  */
}

void
ClipsRobotMemoryThread::clips_bson_append_time(void *        bson,
                                               std::string   field_name,
                                               CLIPS::Values time)
{
	if (time.size() != 2) {
		logger->log_warn("MongoDB", "Invalid time, %zu instead of 2 entries", time.size());
		return;
	}
	if (time[0].type() != CLIPS::TYPE_INTEGER || time[1].type() != CLIPS::TYPE_INTEGER) {
		logger->log_warn("MongoDB", "Invalid time, type mismatch");
		return;
	}

	try {
		auto                   b   = static_cast<bsoncxx::builder::basic::document *>(bson);
		struct timeval         now = {time[0].as_integer(), time[1].as_integer()};
		bsoncxx::types::b_date nowd{
		  std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds>{
		    std::chrono::milliseconds{now.tv_sec * 1000 + now.tv_usec / 1000}}};
		b->append(bsoncxx::builder::basic::kvp(field_name, nowd));
	} catch (bsoncxx::exception &e) {
		logger->log_error("MongoDB",
		                  "Failed to append time value to field %s: %s",
		                  field_name.c_str(),
		                  e.what());
	}
}

void
ClipsRobotMemoryThread::clips_robotmemory_insert(std::string collection, void *bson)
{
	auto b = static_cast<bsoncxx::builder::basic::document *>(bson);

	try {
		robot_memory->insert(b->view(), collection);
	} catch (mongocxx::exception &e) {
		logger->log_warn("MongoDB", "Insert failed: %s", e.what());
	}
}

void
ClipsRobotMemoryThread::clips_robotmemory_create_index(std::string collection, void *bson)
{
	auto b = static_cast<bsoncxx::builder::basic::document *>(bson);

	try {
		robot_memory->create_index(b->view(), collection, /* unique */ false);
	} catch (mongocxx::exception &e) {
		logger->log_warn("MongoDB", "Creating index failed: %s", e.what());
	}
}

void
ClipsRobotMemoryThread::clips_robotmemory_create_unique_index(std::string collection, void *bson)
{
	auto b = static_cast<bsoncxx::builder::basic::document *>(bson);

	try {
		robot_memory->create_index(b->view(), collection, /* unique */ true);
	} catch (mongocxx::exception &e) {
		logger->log_warn("MongoDB", "Creating unique index failed: %s", e.what());
	}
}

void
ClipsRobotMemoryThread::robotmemory_update(std::string &                  collection,
                                           const bsoncxx::document::view &update,
                                           CLIPS::Value &                 query,
                                           bool                           upsert)
{
	try {
		if (query.type() == CLIPS::TYPE_STRING) {
			robot_memory->update(bsoncxx::from_json(query.as_string()), update, collection, upsert);
		} else if (query.type() == CLIPS::TYPE_EXTERNAL_ADDRESS) {
			bsoncxx::builder::basic::document *qb =
			  static_cast<bsoncxx::builder::basic::document *>(query.as_address());
			robot_memory->update(qb->view(), update, collection, upsert);
		} else {
			logger->log_warn("MongoDB", "Invalid query, must be string or BSON document");
			return;
		}

	} catch (bsoncxx::exception &e) {
		logger->log_warn("MongoDB", "Compiling query failed: %s", e.what());
	} catch (mongocxx::exception &e) {
		logger->log_warn("MongoDB", "Insert failed: %s", e.what());
	}
}

void
ClipsRobotMemoryThread::clips_robotmemory_upsert(std::string  collection,
                                                 void *       bson,
                                                 CLIPS::Value query)
{
	auto b = static_cast<bsoncxx::builder::basic::document *>(bson);
	if (!b) {
		logger->log_warn("MongoDB", "Invalid BSON Builder passed");
		return;
	}
	robotmemory_update(collection, b->view(), query, true);
}

void
ClipsRobotMemoryThread::clips_robotmemory_update(std::string  collection,
                                                 void *       bson,
                                                 CLIPS::Value query)
{
	auto b = static_cast<bsoncxx::builder::basic::document *>(bson);
	if (!b) {
		logger->log_warn("MongoDB", "Invalid BSON Builder passed");
		return;
	}
	robotmemory_update(collection, b->view(), query, false);
}

void
ClipsRobotMemoryThread::clips_robotmemory_replace(std::string  collection,
                                                  void *       bson,
                                                  CLIPS::Value query)
{
	auto b = static_cast<bsoncxx::builder::basic::document *>(bson);
	if (!b)
		logger->log_warn("MongoDB", "Invalid BSON Builder passed");
	robotmemory_update(collection, b->view(), query, false);
}

CLIPS::Value
ClipsRobotMemoryThread::clips_robotmemory_query_sort(std::string collection,
                                                     void *      bson,
                                                     void *      bson_sort)
{
	auto b = static_cast<bsoncxx::builder::basic::document *>(bson);

	try {
		mongocxx::options::find find_opts{};
		if (bson_sort) {
			auto *bs = static_cast<bsoncxx::builder::basic::document *>(bson_sort);
			find_opts.sort(bs->view());
		}

		auto cursor = robot_memory->query(b->view(), collection, find_opts);
		//std::unique_ptr<mongocxx::cursor> c = new mongocxx::cursor(std::move(cursor));
		return CLIPS::Value(new std::unique_ptr<mongocxx::cursor>(
		                      new mongocxx::cursor(std::move(cursor))),
		                    CLIPS::TYPE_EXTERNAL_ADDRESS);
	} catch (std::system_error &e) {
		logger->log_warn("MongoDB", "Query failed: %s", e.what());
		return CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL);
	}
}

CLIPS::Value
ClipsRobotMemoryThread::clips_robotmemory_dump_collection(std::string collection,
                                                          std::string directory)
{
	try {
		int succ = robot_memory->dump_collection(collection, directory);
		if (succ == 1) {
			return CLIPS::Value("TRUE", CLIPS::TYPE_SYMBOL);
		} else {
			return CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL);
		}
	} catch (mongocxx::exception &e) {
		logger->log_error("MongoDB",
		                  "Dumping collection %s to %s failed: \n %s",
		                  collection.c_str(),
		                  directory.c_str(),
		                  e.what());
		return CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL);
	}
}

CLIPS::Value
ClipsRobotMemoryThread::clips_robotmemory_restore_collection(std::string collection,
                                                             std::string directory,
                                                             std::string target_collection)
{
	try {
		int succ = robot_memory->restore_collection(collection, directory, target_collection);
		if (succ == 1) {
			return CLIPS::Value("TRUE", CLIPS::TYPE_SYMBOL);
		} else {
			return CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL);
		}
	} catch (mongocxx::exception &e) {
		logger->log_error("MongoDB",
		                  "Restoring collection %s to %s failed: \n %s",
		                  collection.c_str(),
		                  directory.c_str(),
		                  e.what());
		return CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL);
	}
}

void
ClipsRobotMemoryThread::clips_robotmemory_remove(std::string collection, void *bson)
{
	auto b = static_cast<bsoncxx::builder::basic::document *>(bson);
	try {
		robot_memory->remove(b->view(), collection);
	} catch (std::system_error &e) {
		logger->log_warn("MongoDB", "Remove failed: %s", e.what());
	}
}

CLIPS::Value
ClipsRobotMemoryThread::clips_robotmemory_query(const std::string &collection, void *bson)
{
	return clips_robotmemory_query_sort(collection, bson, NULL);
}

void
ClipsRobotMemoryThread::clips_robotmemory_cursor_destroy(void *cursor)
{
	auto c = static_cast<std::unique_ptr<mongocxx::cursor> *>(cursor);
	if (!c || !c->get()) {
		logger->log_error("MongoDB", "mongodb-cursor-destroy: got invalid cursor");
		return;
	}

	delete c;
}

CLIPS::Value
ClipsRobotMemoryThread::clips_robotmemory_cursor_more(void *cursor)
{
	throw Exception("The function cursor-more is no longer supported. Call cursor-next and check the "
	                "return value for FALSE instead.");
}

CLIPS::Value
ClipsRobotMemoryThread::clips_robotmemory_cursor_next(void *cursor)
{
	auto c = static_cast<std::unique_ptr<mongocxx::cursor> *>(cursor);

	if (!c || !c->get()) {
		logger->log_error("MongoDB", "mongodb-cursor-next: got invalid cursor");
		return CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL);
	}

	try {
		auto it = (*c)->begin();
		if (it == (*c)->end()) {
			return CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL);
		} else {
			auto b = new bsoncxx::builder::basic::document();
			b->append(bsoncxx::builder::concatenate(*it));
			it++;
			return CLIPS::Value(b);
		}
	} catch (std::system_error &e) {
		logger->log_error("MongoDB", "mongodb-cursor-next: got invalid query: %s", e.what());
		return CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL);
	}
}

CLIPS::Values
ClipsRobotMemoryThread::clips_bson_field_names(void *bson)
{
	auto b = static_cast<bsoncxx::builder::basic::document *>(bson);

	if (!b) {
		logger->log_error("MongoDB", "mongodb-bson-field-names: invalid object");
		CLIPS::Values rv;
		rv.push_back(CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL));
		return rv;
	}

	CLIPS::Values rv;
	for (auto element : b->view()) {
		rv.push_back(CLIPS::Value(std::string(element.key())));
	}
	return rv;
}

CLIPS::Value
ClipsRobotMemoryThread::clips_bson_get(void *bson, std::string field_name)
{
	auto b = static_cast<bsoncxx::builder::basic::document *>(bson);

	if (!b) {
		logger->log_error("MongoDB", "mongodb-bson-get: invalid object");
		return CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL);
	}

	try {
		auto el = get_dotted_field(b->view(), field_name);
		if (!el) {
			logger->log_warn(name(),
			                 "mongodb-bson-get: failed to get '%s', no such element in doc: %s",
			                 field_name.c_str(),
			                 bsoncxx::to_json(b->view()).c_str());
			return CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL);
		}
		switch (el.type()) {
		case bsoncxx::type::k_double: return CLIPS::Value(el.get_double());
		case bsoncxx::type::k_utf8: return CLIPS::Value(el.get_utf8().value.to_string());
		case bsoncxx::type::k_bool:
			return CLIPS::Value(el.get_bool() ? "TRUE" : "FALSE", CLIPS::TYPE_SYMBOL);
		case bsoncxx::type::k_int32: return CLIPS::Value(el.get_int32());
		case bsoncxx::type::k_int64: return CLIPS::Value(el.get_int64());
		case bsoncxx::type::k_document: {
			auto b = new bsoncxx::builder::basic::document();
			b->append(bsoncxx::builder::concatenate(el.get_document().view()));
			return CLIPS::Value(b);
		}
		case bsoncxx::type::k_oid: //ObjectId
			return CLIPS::Value(el.get_oid().value.to_string());

		default: return CLIPS::Value("INVALID_VALUE_TYPE", CLIPS::TYPE_SYMBOL);
		}
	} catch (std::system_error &e) {
		logger->log_warn(name(),
		                 "mongodb-bson-get: failed to get '%s': %s",
		                 field_name.c_str(),
		                 e.what());
		return CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL);
	}
}

CLIPS::Value
ClipsRobotMemoryThread::clips_bson_has_field(void *bson, std::string field_name)
{
	auto b = static_cast<bsoncxx::builder::basic::document *>(bson);

	if (!b) {
		logger->log_error("MongoDB", "mongodb-bson-get: invalid object");
		return CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL);
	}

	try {
		auto el = get_dotted_field(b->view(), field_name);
		if (!el.key().empty()) {
			return CLIPS::Value("TRUE", CLIPS::TYPE_SYMBOL);
		} else {
			return CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL);
		}
	} catch (bsoncxx::exception &e) {
		return CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL);
	}
}

CLIPS::Values
ClipsRobotMemoryThread::clips_bson_get_array(void *bson, std::string field_name)
{
	auto b = static_cast<bsoncxx::builder::basic::document *>(bson);

	CLIPS::Values rv;

	if (!b) {
		logger->log_error("MongoDB", "mongodb-bson-get-array: invalid object");
		rv.push_back(CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL));
		return rv;
	}

	try {
		auto el = get_dotted_field(b->view(), field_name);

		if (el.type() != bsoncxx::type::k_array) {
			logger->log_error("MongoDB",
			                  "mongodb-bson-get-array: field %s is not an array",
			                  field_name.c_str());
			rv.push_back(CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL));
			return rv;
		}

		bsoncxx::array::view array_view{el.get_array()};
		for (const auto e : array_view) {
			switch (e.type()) {
			case bsoncxx::type::k_double: rv.push_back(CLIPS::Value(e.get_double())); break;
			case bsoncxx::type::k_utf8: rv.push_back(CLIPS::Value(e.get_utf8().value.to_string())); break;
			case bsoncxx::type::k_bool:
				rv.push_back(CLIPS::Value(e.get_bool() ? "TRUE" : "FALSE", CLIPS::TYPE_SYMBOL));
				break;
			case bsoncxx::type::k_int32: rv.push_back(CLIPS::Value(e.get_int32())); break;
			case bsoncxx::type::k_int64: rv.push_back(CLIPS::Value(e.get_int64())); break;
			case bsoncxx::type::k_document: {
				auto b = new bsoncxx::builder::basic::document();
				b->append(bsoncxx::builder::concatenate(e.get_document().view()));
				rv.push_back(CLIPS::Value(b));
			} break;
			default:
				rv.clear();
				rv.push_back(CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL));
				return rv;
			}
		}
		return rv;
	} catch (bsoncxx::exception &e) {
		logger->log_warn(name(),
		                 "mongodb-bson-get: failed to get '%s': %s",
		                 field_name.c_str(),
		                 e.what());
		rv.clear();
		rv.push_back(CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL));
		return rv;
	}
}

CLIPS::Values
ClipsRobotMemoryThread::clips_bson_get_time(void *bson, std::string field_name)
{
	auto b = static_cast<bsoncxx::builder::basic::document *>(bson);

	CLIPS::Values rv;

	if (!b) {
		logger->log_error("MongoDB", "mongodb-bson-get-time: invalid object");
		rv.push_back(CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL));
		return rv;
	}

	try {
		auto    el = get_dotted_field(b->view(), field_name);
		int64_t ts = 0;
		if (el.type() == bsoncxx::type::k_date) {
			bsoncxx::types::b_date d = el.get_date();
			ts                       = d.to_int64();
		} else if (el.type() == bsoncxx::type::k_timestamp) {
			bsoncxx::types::b_timestamp t = el.get_timestamp();
			ts                            = (int64_t)t.timestamp * 1000;
		} else {
			logger->log_error("MongoDB",
			                  "mongodb-bson-get-time: field %s is not a time",
			                  field_name.c_str());
			rv.push_back(CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL));
			return rv;
		}

		rv.resize(2);
		rv[0] = CLIPS::Value((long long int)(ts / 1000));
		rv[1] = CLIPS::Value((ts - (rv[0].as_integer() * 1000)) * 1000);
		return rv;
	} catch (bsoncxx::exception &e) {
		rv.resize(2, CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL));
		return rv;
	}
}

CLIPS::Value
ClipsRobotMemoryThread::clips_robotmemory_register_trigger(std::string env_name,
                                                           std::string collection,
                                                           void *      query,
                                                           std::string assert_name)
{
	bsoncxx::document::value b{static_cast<bsoncxx::builder::basic::document *>(query)->view()};
	std::string              future_name = "register_trigger_" + assert_name;
	if (!mutex_future_ready(future_name)) {
		MutexLocker clips_lock(envs_[env_name].objmutex_ptr());
		envs_[env_name]->assert_fact_f("(mutex-trigger-register-feedback FAIL \"%s\")",
		                               assert_name.c_str());
		return CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL);
	}
	auto fut = std::async(std::launch::async, [this, b, env_name, collection, assert_name] {
		try {
			ClipsRmTrigger *clips_trigger =
			  new ClipsRmTrigger(assert_name, robot_memory, envs_[env_name], logger);
			clips_trigger->set_trigger(robot_memory->register_trigger(
			  b.view(), collection, &ClipsRmTrigger::callback, clips_trigger));
			MutexLocker triggers_lock(&clips_triggers_mutex_);
			clips_triggers_.push_back(clips_trigger);
			MutexLocker clips_lock(envs_[env_name].objmutex_ptr());
			envs_[env_name]->assert_fact_f("(mutex-trigger-register-feedback SUCCESS \"%s\" %p)",
			                               assert_name.c_str(),
			                               CLIPS::Value(clips_trigger).as_address());
			return true;
		} catch (std::system_error &e) {
			logger->log_warn(name(), "Error while registering trigger: %s", e.what());
			MutexLocker clips_lock(envs_[env_name].objmutex_ptr());
			envs_[env_name]->assert_fact_f("(mutex-trigger-register-feedback FAIL \"%s\")",
			                               assert_name.c_str());
			return false;
		}
	});

	mutex_futures_[future_name] = std::move(fut);
	return CLIPS::Value("TRUE", CLIPS::TYPE_SYMBOL);
}

void
ClipsRobotMemoryThread::clips_robotmemory_destroy_trigger(void *trigger)
{
	ClipsRmTrigger *clips_trigger = static_cast<ClipsRmTrigger *>(trigger);
	MutexLocker     triggers_lock(&clips_triggers_mutex_);
	clips_triggers_.remove(clips_trigger);
	delete clips_trigger; //the triger unregisteres itself at the robot memory
}

CLIPS::Value
ClipsRobotMemoryThread::clips_robotmemory_mutex_create(std::string name)
{
	bool rv = robot_memory->mutex_create(name);
	return CLIPS::Value(rv ? "TRUE" : "FALSE", CLIPS::TYPE_SYMBOL);
}

CLIPS::Value
ClipsRobotMemoryThread::clips_robotmemory_mutex_destroy(std::string name)
{
	bool rv = robot_memory->mutex_destroy(name);
	return CLIPS::Value(rv ? "TRUE" : "FALSE", CLIPS::TYPE_SYMBOL);
}

CLIPS::Value
ClipsRobotMemoryThread::clips_robotmemory_mutex_try_lock(std::string name, std::string identity)
{
	bool rv = robot_memory->mutex_try_lock(name, identity);
	return CLIPS::Value(rv ? "TRUE" : "FALSE", CLIPS::TYPE_SYMBOL);
}

CLIPS::Value
ClipsRobotMemoryThread::clips_robotmemory_mutex_renew_lock(std::string name, std::string identity)
{
	bool rv = robot_memory->mutex_renew_lock(name, identity);
	return CLIPS::Value(rv ? "TRUE" : "FALSE", CLIPS::TYPE_SYMBOL);
}

CLIPS::Value
ClipsRobotMemoryThread::clips_robotmemory_mutex_force_lock(std::string name, std::string identity)
{
	bool rv = robot_memory->mutex_try_lock(name, identity, /* force */ true);
	return CLIPS::Value(rv ? "TRUE" : "FALSE", CLIPS::TYPE_SYMBOL);
}

CLIPS::Value
ClipsRobotMemoryThread::clips_robotmemory_mutex_unlock(std::string name, std::string identity)
{
	bool rv = robot_memory->mutex_unlock(name, identity);
	return CLIPS::Value(rv ? "TRUE" : "FALSE", CLIPS::TYPE_SYMBOL);
}

CLIPS::Value
ClipsRobotMemoryThread::clips_robotmemory_mutex_setup_ttl(float max_age_sec)
{
	bool rv = robot_memory->mutex_setup_ttl(max_age_sec);
	return CLIPS::Value(rv ? "TRUE" : "FALSE", CLIPS::TYPE_SYMBOL);
}

CLIPS::Value
ClipsRobotMemoryThread::clips_robotmemory_mutex_expire_locks(float max_age_sec)
{
	bool rv = robot_memory->mutex_expire_locks(max_age_sec);
	return CLIPS::Value(rv ? "TRUE" : "FALSE", CLIPS::TYPE_SYMBOL);
}

bool
ClipsRobotMemoryThread::mutex_future_ready(const std::string &name)
{
	auto mf_it = mutex_futures_.find(name);
	if (mf_it != mutex_futures_.end()) {
		auto fut_status = mutex_futures_[name].wait_for(std::chrono::milliseconds(0));
		if (fut_status != std::future_status::ready) {
			return false;
		} else {
			mutex_futures_.erase(mf_it);
		}
	}
	return true;
}

CLIPS::Values
ClipsRobotMemoryThread::clips_robotmemory_mutex_create_async(std::string name)
{
	CLIPS::Values rv;
	if (!mutex_future_ready(name)) {
		rv.push_back(CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL));
		rv.push_back(CLIPS::Value("Task already running for " + name + " (create failed)"));
		return rv;
	}

	auto fut =
	  std::async(std::launch::async, [this, name] { return robot_memory->mutex_create(name); });

	mutex_futures_[name] = std::move(fut);

	rv.push_back(CLIPS::Value("TRUE", CLIPS::TYPE_SYMBOL));
	return rv;
}

CLIPS::Values
ClipsRobotMemoryThread::clips_robotmemory_mutex_destroy_async(std::string name)
{
	CLIPS::Values rv;
	if (!mutex_future_ready(name)) {
		rv.push_back(CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL));
		rv.push_back(CLIPS::Value("Task already running for " + name + " (destroy failed)"));
		return rv;
	}

	auto fut =
	  std::async(std::launch::async, [this, name] { return robot_memory->mutex_destroy(name); });

	mutex_futures_[name] = std::move(fut);

	rv.push_back(CLIPS::Value("TRUE", CLIPS::TYPE_SYMBOL));
	return rv;
}

CLIPS::Values
ClipsRobotMemoryThread::clips_robotmemory_mutex_try_lock_async(std::string env_name,
                                                               std::string name,
                                                               std::string identity)
{
	CLIPS::Values rv;
	if (!mutex_future_ready(name)) {
		rv.push_back(CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL));
		rv.push_back(CLIPS::Value("Task already running for " + name + " (try-lock failed)"));
		envs_[env_name]->assert_fact_f("(mutex-op-feedback try-lock-async FAIL %s)", name.c_str());
		return rv;
	}

	auto fut = std::async(std::launch::async, [this, env_name, name, identity] {
		bool ok = robot_memory->mutex_try_lock(name, identity);
		if (!ok) {
			MutexLocker lock(envs_[env_name].objmutex_ptr());
			envs_[env_name]->assert_fact_f("(mutex-op-feedback try-lock-async FAIL %s)", name.c_str());
		}
		return ok;
	});

	mutex_futures_[name] = std::move(fut);

	rv.push_back(CLIPS::Value("TRUE", CLIPS::TYPE_SYMBOL));
	return rv;
}

CLIPS::Values
ClipsRobotMemoryThread::clips_robotmemory_mutex_renew_lock_async(std::string env_name,
                                                                 std::string name,
                                                                 std::string identity)
{
	CLIPS::Values rv;
	if (!mutex_future_ready(name)) {
		rv.push_back(CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL));
		rv.push_back(CLIPS::Value("Task already running for " + name + " (try-lock failed)"));
		MutexLocker lock(envs_[env_name].objmutex_ptr());
		envs_[env_name]->assert_fact_f("(mutex-op-feedback renew-lock-async FAIL %s)", name.c_str());
		return rv;
	}

	auto fut = std::async(std::launch::async, [this, env_name, name, identity] {
		bool        ok = robot_memory->mutex_renew_lock(name, identity);
		MutexLocker lock(envs_[env_name].objmutex_ptr());
		envs_[env_name]->assert_fact_f("(mutex-op-feedback renew-lock-async %s %s)",
		                               ok ? "OK" : "FAIL",
		                               name.c_str());
		return ok;
	});

	mutex_futures_[name] = std::move(fut);

	rv.push_back(CLIPS::Value("TRUE", CLIPS::TYPE_SYMBOL));
	return rv;
}

CLIPS::Values
ClipsRobotMemoryThread::clips_robotmemory_mutex_force_lock_async(std::string name,
                                                                 std::string identity)
{
	CLIPS::Values rv;
	if (!mutex_future_ready(name)) {
		rv.push_back(CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL));
		rv.push_back(CLIPS::Value("Task already running for " + name + " (force-lock failed)"));
		return rv;
	}

	auto fut = std::async(std::launch::async, [this, name, identity] {
		return robot_memory->mutex_try_lock(name, identity, /* force */ true);
	});

	mutex_futures_[name] = std::move(fut);

	rv.push_back(CLIPS::Value("TRUE", CLIPS::TYPE_SYMBOL));
	return rv;
}

CLIPS::Values
ClipsRobotMemoryThread::clips_robotmemory_mutex_unlock_async(std::string name, std::string identity)
{
	CLIPS::Values rv;
	if (!mutex_future_ready(name)) {
		rv.push_back(CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL));
		rv.push_back(CLIPS::Value("Task already running for " + name + " (unlock failed)"));
		return rv;
	}

	auto fut = std::async(std::launch::async, [this, name, identity] {
		return robot_memory->mutex_unlock(name, identity);
	});

	mutex_futures_[name] = std::move(fut);

	rv.push_back(CLIPS::Value("TRUE", CLIPS::TYPE_SYMBOL));
	return rv;
}

CLIPS::Value
ClipsRobotMemoryThread::clips_robotmemory_mutex_expire_locks_async(std::string env_name,
                                                                   float       max_age_sec)
{
	CLIPS::Values rv;
	if (mutex_expire_future_.valid()) {
		// have shared state, expire was or is running
		auto fut_status = mutex_expire_future_.wait_for(std::chrono::milliseconds(0));
		if (fut_status != std::future_status::ready) {
			MutexLocker lock(envs_[env_name].objmutex_ptr());
			envs_[env_name]->assert_fact_f("(mutex-op-feedback expire-locks-async FAIL)");
			return CLIPS::Value("FALSE", CLIPS::TYPE_SYMBOL);
		}
	}

	auto fut = std::async(std::launch::async, [this, env_name, max_age_sec] {
		bool        ok = robot_memory->mutex_expire_locks(max_age_sec);
		MutexLocker lock(envs_[env_name].objmutex_ptr());
		envs_[env_name]->assert_fact_f("(mutex-op-feedback expire-locks-async %s)", ok ? "OK" : "FAIL");
		return ok;
	});

	mutex_expire_future_ = std::move(fut);

	return CLIPS::Value("TRUE", CLIPS::TYPE_SYMBOL);
}
