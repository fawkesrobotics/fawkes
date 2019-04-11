
/***************************************************************************
 *  mongodb_log_tf_thread.cpp - MongoDB transforms logging Thread
 *
 *  Created: Tue Dec 11 14:58:00 2012 (Freiburg)
 *  Copyright  2010-2017  Tim Niemueller [www.niemueller.de]
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

#include "mongodb_log_tf_thread.h"

#include <core/threading/mutex_locker.h>
#include <plugins/mongodb/aspect/mongodb_conncreator.h>
#include <tf/time_cache.h>
#include <utils/time/wait.h>

#include <bsoncxx/builder/basic/document.hpp>
#include <cstdlib>
#include <mongocxx/client.hpp>
#include <mongocxx/exception/operation_exception.hpp>

using namespace mongocxx;
using namespace fawkes;

/** @class MongoLogTransformsThread "mongodb_log_tf_thread.h"
 * MongoDB transforms logging thread.
 * This thread periodically queries the transformer for recent transforms
 * and stores them to the database.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
MongoLogTransformsThread::MongoLogTransformsThread()
: Thread("MongoLogTransformsThread", Thread::OPMODE_CONTINUOUS),
  MongoDBAspect("default"),
  TransformAspect(TransformAspect::ONLY_LISTENER)
{
	set_prepfin_conc_loop(true);
}

/** Destructor. */
MongoLogTransformsThread::~MongoLogTransformsThread()
{
}

void
MongoLogTransformsThread::init()
{
	database_   = "fflog";
	collection_ = "tf";
	try {
		database_ = config->get_string("/plugins/mongodb-log/database");
	} catch (Exception &e) {
		logger->log_info(name(), "No database configured, writing to %s", database_.c_str());
	}

	try {
		collection_ = config->get_string("/plugins/mongodb-log/transforms/collection");
	} catch (Exception &e) {
		logger->log_info(name(), "No transforms collection configured, using %s", collection_.c_str());
	}
	collection_ = database_ + "." + collection_;

	cfg_storage_interval_ = config->get_float("/plugins/mongodb-log/transforms/storage-interval");

	if (cfg_storage_interval_ <= 0.) {
		// 50% of the cache time, assume 50% is enough to store the data
		cfg_storage_interval_ = tf_listener->get_cache_time() * 0.5;
	}

	wait_  = new TimeWait(clock, cfg_storage_interval_ * 1000000.);
	mutex_ = new Mutex();
}

bool
MongoLogTransformsThread::prepare_finalize_user()
{
	mutex_->lock();
	return true;
}

void
MongoLogTransformsThread::finalize()
{
	delete wait_;
	delete mutex_;
}

void
MongoLogTransformsThread::loop()
{
	mutex_->lock();
	fawkes::Time loop_start(clock);
	wait_->mark_start();
	std::vector<fawkes::Time> tf_range_start;
	std::vector<fawkes::Time> tf_range_end;
	;

	tf_listener->lock();
	std::vector<tf::TimeCacheInterfacePtr> caches = tf_listener->get_frame_caches();
	std::vector<tf::TimeCacheInterfacePtr> copies(caches.size(), tf::TimeCacheInterfacePtr());

	const size_t n_caches = caches.size();
	tf_range_start.resize(n_caches, fawkes::Time(0, 0));
	tf_range_end.resize(n_caches, fawkes::Time(0, 0));
	if (last_tf_range_end_.size() != n_caches) {
		last_tf_range_end_.resize(n_caches, fawkes::Time(0, 0));
	}

	unsigned int num_transforms = 0;
	unsigned int num_upd_caches = 0;

	for (size_t i = 0; i < n_caches; ++i) {
		if (caches[i]) {
			tf_range_end[i] = caches[i]->get_latest_timestamp();
			if (last_tf_range_end_[i] != tf_range_end[i]) {
				// we have new data
				if (!tf_range_end[i].is_zero()) {
					tf_range_start[i] = tf_range_end[i] - cfg_storage_interval_;
					if (last_tf_range_end_[i] > tf_range_start[i]) {
						tf_range_start[i] = last_tf_range_end_[i];
					}
				}
				copies[i]             = caches[i]->clone(tf_range_start[i]);
				last_tf_range_end_[i] = tf_range_end[i];
				num_upd_caches += 1;
				num_transforms += copies[i]->get_list_length();
			}
		}
	}
	tf_listener->unlock();

	store(copies, tf_range_start, tf_range_end);

	mutex_->unlock();
	// -1 to subtract "NO PARENT" pseudo cache
	fawkes::Time loop_end(clock);
	logger->log_debug(name(),
	                  "%u transforms for %u updated frames stored in %.1f ms",
	                  num_transforms,
	                  num_upd_caches,
	                  (loop_end - &loop_start) * 1000.);
	wait_->wait();
}

void
MongoLogTransformsThread::store(std::vector<tf::TimeCacheInterfacePtr> &caches,
                                std::vector<fawkes::Time> &             from,
                                std::vector<fawkes::Time> &             to)
{
	std::vector<std::string> frame_map = tf_listener->get_frame_id_mappings();

	for (size_t i = 0; i < caches.size(); ++i) {
		tf::TimeCacheInterfacePtr tc = caches[i];
		if (!tc)
			continue;

		using namespace bsoncxx::builder;
		basic::document document;

		document.append(basic::kvp("timestamp", static_cast<int64_t>(from[i].in_msec())));
		document.append(basic::kvp("timestamp_from", static_cast<int64_t>(from[i].in_msec())));
		document.append(basic::kvp("timestamp_to", static_cast<int64_t>(to[i].in_msec())));
		const tf::TimeCache::L_TransformStorage &storage = tc->get_storage();

		if (storage.empty()) {
			/*
      fawkes::Time now(clock);
      logger->log_warn(name(), "Empty storage for %s (start: %lu  end: %lu  now: %lu",
			frame_map[i].c_str(), from[i].in_msec(), to[i].in_msec(),
			now.in_msec());
      */
			continue;
		}

		document.append(basic::kvp("frame", frame_map[storage.front().frame_id]));
		document.append(basic::kvp("child_frame", frame_map[storage.front().child_frame_id]));

		/*
    logger->log_debug(name(), "Writing %zu transforms for child frame %s",
		      storage.size(), frame_map[i].c_str());
    */

		document.append(basic::kvp("transforms", [storage, frame_map](basic::sub_array array) {
			for (auto s = storage.begin(); s != storage.end(); ++s) {
				/*
	      "frame" : "/bl_caster_rotation_link",
	      "child_frame" : "/bl_caster_l_wheel_link",
	      "translation" : [
	      	0,
	      	0.049,
	      	0
	      ],
	      "rotation" : [
	      	0,
	      	0.607301261865804,
	      	0,
	      	0.7944716340664417
	      ]
        */
				basic::document tf_doc;
				tf_doc.append(basic::kvp("timestamp", static_cast<int64_t>(s->stamp.in_msec())));
				tf_doc.append(basic::kvp("frame", frame_map[s->frame_id]));
				tf_doc.append(basic::kvp("child_frame", frame_map[s->child_frame_id]));
				tf_doc.append(basic::kvp("rotation", [s](basic::sub_array rot_array) {
					rot_array.append(s->rotation.x());
					rot_array.append(s->rotation.y());
					rot_array.append(s->rotation.z());
					rot_array.append(s->rotation.w());
				}));
				tf_doc.append(basic::kvp("translation", [s](basic::sub_array trans_array) {
					trans_array.append(s->translation.x());
					trans_array.append(s->translation.y());
					trans_array.append(s->translation.z());
				}));
				array.append(tf_doc);
			}
		}));

		try {
			mongodb_client->database(database_)[collection_].insert_one(document.view());
		} catch (operation_exception &e) {
			logger->log_warn(name(), "Inserting TF failed: %s", e.what());

		} catch (std::exception &e) {
			// for old and broken compilers
			logger->log_warn(name(), "Inserting TF failed: %s (*)", e.what());
		}
	}
}
