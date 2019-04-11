
/***************************************************************************
 *  mongodb_log_pcl_thread.cpp - Thread to log point clouds to MongoDB
 *
 *  Created: Mon Nov 07 02:58:40 2011
 *  Copyright  2011-2017  Tim Niemueller [www.niemueller.de]
 *             2012       Bastian Klingen
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

#include "mongodb_log_pcl_thread.h"

// Fawkes
#include <core/threading/mutex_locker.h>
#include <utils/time/wait.h>

// from MongoDB
#include <fnmatch.h>
#include <mongocxx/client.hpp>
#include <mongocxx/exception/operation_exception.hpp>
#include <mongocxx/gridfs/uploader.hpp>
#include <unistd.h>

using namespace fawkes;
using namespace mongocxx;

/** @class MongoLogPointCloudThread "mongodb_log_pcl_thread.h"
 * Thread to store point clouds to MongoDB.
 * @author Tim Niemueller
 * @author Bastian Klingen
 */

/** Constructor. */
MongoLogPointCloudThread::MongoLogPointCloudThread()
: Thread("MongoLogPointCloudThread", Thread::OPMODE_CONTINUOUS), MongoDBAspect("default")
{
	set_prepfin_conc_loop(true);
}

/** Destructor. */
MongoLogPointCloudThread::~MongoLogPointCloudThread()
{
}

void
MongoLogPointCloudThread::init()
{
	database_ = "fflog";
	try {
		database_ = config->get_string("/plugins/mongodb-log/database");
	} catch (Exception &e) {
		logger->log_info(name(), "No database configured, writing to %s", database_.c_str());
	}

	cfg_storage_interval_ = config->get_float("/plugins/mongodb-log/pointclouds/storage-interval");

	cfg_chunk_size_ = 2097152; // 2 MB
	try {
		cfg_chunk_size_ = config->get_uint("/plugins/mongodb-log/pointclouds/chunk-size");
	} catch (Exception &e) {
	} // ignored, use default
	logger->log_info(name(), "Chunk size: %u", cfg_chunk_size_);

	cfg_flush_after_write_ = false;
	try {
		cfg_flush_after_write_ = config->get_uint("/plugins/mongodb-log/pointclouds/flush-after-write");
	} catch (Exception &e) {
	} // ignored, use default

	std::vector<std::string> includes;
	try {
		includes = config->get_strings("/plugins/mongodb-log/pointclouds/includes");
	} catch (Exception &e) {
	} // ignored, no include rules
	std::vector<std::string> excludes;

	try {
		excludes = config->get_strings("/plugins/mongodb-log/pointclouds/excludes");
	} catch (Exception &e) {
	} // ignored, no include rules

	mongodb_ = mongodb_client;
	gridfs_  = mongodb_->database(database_).gridfs_bucket();
	//gridfs_->setChunkSize(cfg_chunk_size_);

	adapter_ = new PointCloudAdapter(pcl_manager, logger);

	std::vector<std::string> pcls = pcl_manager->get_pointcloud_list();

	std::vector<std::string>::iterator p;
	std::vector<std::string>::iterator f;
	for (p = pcls.begin(); p != pcls.end(); ++p) {
		bool include = includes.empty();
		if (!include) {
			for (f = includes.begin(); f != includes.end(); ++f) {
				if (fnmatch(f->c_str(), p->c_str(), 0) != FNM_NOMATCH) {
					logger->log_debug(name(), "Include match %s to %s", f->c_str(), p->c_str());
					include = true;
					break;
				}
			}
		}
		if (include) {
			for (f = excludes.begin(); f != excludes.end(); ++f) {
				if (fnmatch(f->c_str(), p->c_str(), 0) != FNM_NOMATCH) {
					logger->log_debug(name(), "Exclude match %s to %s", f->c_str(), p->c_str());
					include = false;
					break;
				}
			}
		}
		if (!include) {
			logger->log_info(name(), "Excluding point cloud %s", p->c_str());
			continue;
		}

		PointCloudInfo pi;

		std::string topic_name = std::string("PointClouds.") + *p;
		size_t      pos        = 0;
		while ((pos = topic_name.find_first_of(" -", pos)) != std::string::npos) {
			topic_name.replace(pos, 1, "_");
			pos = pos + 1;
		}

		pi.topic_name = topic_name;

		logger->log_info(name(), "MongoLog of point cloud %s", p->c_str());

		std::string                         frame_id;
		unsigned int                        width, height;
		bool                                is_dense;
		PointCloudAdapter::V_PointFieldInfo fieldinfo;
		adapter_->get_info(*p, width, height, frame_id, is_dense, fieldinfo);
		pi.msg.header.frame_id = frame_id;
		pi.msg.width           = width;
		pi.msg.height          = height;
		pi.msg.is_dense        = is_dense;
		pi.msg.fields.clear();
		pi.msg.fields.resize(fieldinfo.size());
		for (unsigned int i = 0; i < fieldinfo.size(); ++i) {
			pi.msg.fields[i].name     = fieldinfo[i].name;
			pi.msg.fields[i].offset   = fieldinfo[i].offset;
			pi.msg.fields[i].datatype = fieldinfo[i].datatype;
			pi.msg.fields[i].count    = fieldinfo[i].count;
		}

		pcls_[*p] = pi;
	}

	wait_  = new TimeWait(clock, cfg_storage_interval_ * 1000000.);
	mutex_ = new Mutex();
}

bool
MongoLogPointCloudThread::prepare_finalize_user()
{
	mutex_->lock();
	return true;
}

void
MongoLogPointCloudThread::finalize()
{
	delete adapter_;
	delete wait_;
	delete mutex_;
}

void
MongoLogPointCloudThread::loop()
{
	MutexLocker  lock(mutex_);
	fawkes::Time loop_start(clock);
	wait_->mark_start();
	std::map<std::string, PointCloudInfo>::iterator p;
	unsigned int                                    num_stored = 0;
	for (p = pcls_.begin(); p != pcls_.end(); ++p) {
		PointCloudInfo &pi = p->second;
		std::string     frame_id;
		unsigned int    width, height;
		void *          point_data;
		size_t          point_size, num_points;
		fawkes::Time    time;
		adapter_->get_data(
		  p->first, frame_id, width, height, time, &point_data, point_size, num_points);
		size_t data_size = point_size / sizeof(uint8_t) * num_points;

		if (pi.last_sent != time) {
			pi.last_sent = time;

			fawkes::Time start(clock);

			using namespace bsoncxx::builder;
			basic::document   document;
			std::stringstream name;
			name << pi.topic_name << "_" << time.in_msec();
			auto uploader = gridfs_.open_upload_stream(name.str());
			uploader.write((uint8_t *)point_data, data_size);
			auto result = uploader.close();
			document.append(basic::kvp("timestamp", static_cast<int64_t>(time.in_msec())));
			document.append(basic::kvp("pointcloud", [&](basic::sub_document subdoc) {
				subdoc.append(basic::kvp("frame_id", pi.msg.header.frame_id));
				subdoc.append(basic::kvp("is_dense", pi.msg.is_dense));
				subdoc.append(basic::kvp("width", static_cast<int32_t>(width)));
				subdoc.append(basic::kvp("height", static_cast<int32_t>(height)));
				subdoc.append(basic::kvp("point_size", static_cast<int32_t>(point_size)));
				subdoc.append(basic::kvp("num_points", static_cast<int32_t>(num_points)));
				// TODO: We store the ID, is this correct?
				subdoc.append(basic::kvp("data", result.id()));
				subdoc.append(basic::kvp("field_info", [pi](basic::sub_array array) {
					for (uint i = 0; i < pi.msg.fields.size(); i++) {
						basic::document field_info_doc;
						field_info_doc.append(basic::kvp("name", pi.msg.fields[i].name));
						field_info_doc.append(
						  basic::kvp("offset", static_cast<int32_t>(pi.msg.fields[i].offset)));
						field_info_doc.append(
						  basic::kvp("datatype", static_cast<int32_t>(pi.msg.fields[i].datatype)));
						field_info_doc.append(
						  basic::kvp("count", static_cast<int32_t>(pi.msg.fields[i].count)));
						array.append(field_info_doc);
					}
				}));
			}));

			try {
				mongodb_->database(database_)[pi.topic_name].insert_one(document.view());
				++num_stored;
			} catch (operation_exception &e) {
				logger->log_warn(this->name(),
				                 "Failed to insert into %s: %s",
				                 collection_.c_str(),
				                 e.what());
			}

			fawkes::Time end(clock);
			float        diff = (end - &start) * 1000.;
			logger->log_debug(this->name(),
			                  "Stored point cloud %s (time %li) in %.1f ms",
			                  p->first.c_str(),
			                  time.in_msec(),
			                  diff);

		} else {
			logger->log_debug(this->name(), "Point cloud %s did not change", p->first.c_str());
			//adapter_->close(p->first);
		}
	}
	mutex_->unlock();
	// -1 to subtract "NO PARENT" pseudo cache
	fawkes::Time loop_end(clock);
	logger->log_debug(name(),
	                  "Stored %u of %zu point clouds in %.1f ms",
	                  num_stored,
	                  pcls_.size(),
	                  (loop_end - &loop_start) * 1000.);

	if (cfg_flush_after_write_) {
		// flush database
		using namespace bsoncxx::builder;
		basic::document flush_cmd;
		flush_cmd.append(basic::kvp("fsync", 1));
		flush_cmd.append(basic::kvp("async", 1));
		auto reply = mongodb_client->database("admin").run_command(flush_cmd.view());
		if (reply.view()["ok"].get_double() != 1) {
			logger->log_warn(name(),
			                 "fsync error: %s",
			                 reply.view()["errmsg"].get_utf8().value.to_string().c_str());
		}
	}
	wait_->wait();
}
