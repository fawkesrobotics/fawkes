/***************************************************************************
 *  mongodb_tf_transformer.cpp - Read and provide TFs from MongoDB
 *
 *  Created: Thu Nov 29 22:59:49 2012
 *  Copyright  2012  Tim Niemueller [www.niemueller.de]
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#include "mongodb_tf_transformer.h"

#include <bsoncxx/builder/basic/document.hpp>
#include <list>

#ifdef HAVE_MONGODB_VERSION_H
// we are using mongo-cxx-driver which renamed QUERY to MONGO_QUERY
#	define QUERY MONGO_QUERY
#endif

using namespace mongocxx;

namespace fawkes {
namespace tf {

/** @class MongoDBTransformer "mongodb_tf_transformer.h"
 * Read transforms from MongoDB and answer queries.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param mongodb_client MongoDB database client
 * @param database_name name of database to restore transforms from
 * @param ensure_index if true ensures that the required index on timestamps exists
 */
MongoDBTransformer::MongoDBTransformer(mongocxx::client * mongodb_client,
                                       const std::string &database_name,
                                       bool               ensure_index)
: mongodb_client_(mongodb_client), database_(database_name)
{
	if (ensure_index) {
		using namespace bsoncxx::builder;
		mongodb_client_->database(database_)["tf"].create_index(
		  basic::make_document(basic::kvp("timestamp", 1)));
		mongodb_client_->database(database_)["TransformInterface"].create_index(
		  basic::make_document(basic::kvp("timestamp", 1)));
	}
}

/** Destructor. */
MongoDBTransformer::~MongoDBTransformer()
{
}

/** Restore transforms from database.
 * @param start start time of range to restore
 * @param end end time of range to restore
 * @param new_start the new start time to which the transform times
 * will be reset, i.e. from the transforms time stamp the @p start
 * time is subtracted and @p new_start is added.
 */
void
MongoDBTransformer::restore(fawkes::Time &start, fawkes::Time &end, fawkes::Time &new_start)
{
	restore(start.in_msec(), end.in_msec(), new_start.in_msec());
}

void
MongoDBTransformer::restore_tf_doc(const bsoncxx::document::view &doc,
                                   long long                      start_msec,
                                   long long                      new_start_msec)
{
	bsoncxx::array::view trans = doc["translation"].get_array();
	bsoncxx::array::view rot   = doc["rotation"].get_array();
	double               rx, ry, rz, rw, tx, ty, tz;
	std::string          frame, child_frame;
	long                 timestamp = new_start_msec + (doc["timestamp"].get_int64() - start_msec);
	Time                 time(timestamp);
	rx          = rot[0].get_double();
	ry          = rot[1].get_double();
	rz          = rot[2].get_double();
	rw          = rot[3].get_double();
	tx          = trans[0].get_double();
	ty          = trans[1].get_double();
	tz          = trans[2].get_double();
	frame       = doc["frame"].get_utf8().value.to_string();
	child_frame = doc["child_frame"].get_utf8().value.to_string();

	tf::Quaternion q(rx, ry, rz, rw);
	tf::assert_quaternion_valid(q);
	tf::Transform        t(q, tf::Vector3(tx, ty, tz));
	tf::StampedTransform transform(t, time, frame, child_frame);
	set_transform(transform, "MongoDBTransformer");
}

/** Restore transforms from database.
 * @param start_msec start time of range to restore since the epoch in msec
 * @param end_msec end time of range to restore since the epoch in msec
 * @param new_start_msec the new start time since the epoch in msec to which the
 * transform times will be reset, i.e. from the transforms time stamp the
 * @p start time is subtracted and @p new_start is added.
 */
void
MongoDBTransformer::restore(long start_msec, long end_msec, long new_start_msec)
{
	cache_time_ = (double)(end_msec - start_msec) / 1000.;

	if (new_start_msec == 0) {
		new_start_msec = start_msec;
	}

	// requires mongo-cxx-driver 3.4.0
	//std::list<std::string> collections = mongodb_client_->database(database_).list_collection_names();
	std::list<std::string> collections;
	for (auto c : mongodb_client_->database(database_).list_collections()) {
		collections.push_back(c["name"].get_utf8().value.to_string());
	}

	std::list<std::string>::iterator c;
	for (c = collections.begin(); c != collections.end(); ++c) {
		if ((c->find(database_ + ".TransformInterface.") != 0) && (c->find(database_ + ".tf") != 0)) {
			continue;
		}

		auto collection = mongodb_client_->database(database_)[*c];
		using namespace bsoncxx::builder;
		auto result = collection.find(
		  basic::make_document(
		    basic::kvp("timestamp",
		               [start_msec, end_msec](basic::sub_document subdoc) {
			               subdoc.append(basic::kvp("$gt", static_cast<int64_t>(start_msec)));
			               subdoc.append(basic::kvp("$lt", static_cast<int64_t>(end_msec)));
		               })),
		  mongocxx::options::find().sort(basic::make_document(basic::kvp("timestamp", 1))));

		for (auto doc : result) {
			if (doc.find("transforms") != doc.end()) {
				bsoncxx::array::view transforms = doc["transforms"].get_array();
				for (auto el : transforms) {
					restore_tf_doc(el.get_document().view(), start_msec, new_start_msec);
				}
			} else {
				restore_tf_doc(doc, start_msec, new_start_msec);
			}
		}
	}
}

} // end namespace tf
} // end namespace fawkes
