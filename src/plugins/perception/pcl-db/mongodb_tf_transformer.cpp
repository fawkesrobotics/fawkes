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

#include <list>

#ifdef HAVE_MONGODB_VERSION_H
// we are using mongo-cxx-driver which renamed QUERY to MONGO_QUERY
#  define QUERY MONGO_QUERY
#endif

using namespace mongo;

namespace fawkes {
  namespace tf {
#if 0 /* just to make Emacs auto-indent happy */
  }
}
#endif

/** @class MongoDBTransformer "mongodb_tf_transformer.h"
 * Read transforms from MongoDB and answer queries.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param mongodb_client MongoDB database client
 * @param database_name name of database to restore transforms from
 * @param ensure_index if true ensures that the required index on timestamps exists
 */
MongoDBTransformer::MongoDBTransformer(mongo::DBClientBase *mongodb_client,
				       std::string database_name, bool ensure_index)
  : mongodb_client_(mongodb_client), database_(database_name)
{
  if (ensure_index) {
#ifdef HAVE_MONGODB_VERSION_H
    // mongodb-cxx-driver dropped ensureIndex and names it createIndex
    mongodb_client_->createIndex(database_ + ".tf", mongo::fromjson("{timestamp:1}"));
    mongodb_client_->createIndex(database_ + ".TransformInterface",
				 mongo::fromjson("{timestamp:1}"));
#else
    mongodb_client_->ensureIndex(database_ + ".tf", mongo::fromjson("{timestamp:1}"));
    mongodb_client_->ensureIndex(database_ + ".TransformInterface",
				 mongo::fromjson("{timestamp:1}"));
#endif
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
MongoDBTransformer::restore_tf_doc(BSONObj &doc,
				   long long start_msec, long long new_start_msec)
{
  std::vector<BSONElement> trans = doc["translation"].Array();
  std::vector<BSONElement> rot = doc["rotation"].Array();
  double rx, ry, rz, rw, tx, ty, tz;
  std::string frame, child_frame;
  long timestamp = new_start_msec + (doc["timestamp"].Long() - start_msec);
  Time time(timestamp);
  rx = rot[0].Double();
  ry = rot[1].Double();
  rz = rot[2].Double();
  rw = rot[3].Double();
  tx = trans[0].Double();
  ty = trans[1].Double();
  tz = trans[2].Double();
  frame = doc["frame"].String();
  child_frame = doc["child_frame"].String();

  tf::Quaternion q(rx, ry, rz, rw);
  tf::assert_quaternion_valid(q);
  tf::Transform t(q, tf::Vector3(tx, ty, tz));
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
MongoDBTransformer::restore(long long start_msec, long long end_msec, long long new_start_msec)
{
  cache_time_ = (double)(end_msec - start_msec) / 1000.;

  if (new_start_msec == 0) {
    new_start_msec = start_msec;
  }

  std::list<std::string> collections =
    mongodb_client_->getCollectionNames(database_);
  
#if __cplusplus >= 201103L
  std::unique_ptr<DBClientCursor> cursor;
#else
  std::auto_ptr<DBClientCursor> cursor;
#endif
  BSONObj doc;
  std::list<std::string>::iterator c;
  for (c = collections.begin(); c != collections.end(); ++c) {
    if ((c->find(database_ + ".TransformInterface.") != 0 ) &&
	(c->find(database_ + ".tf") != 0) )
    {
      continue;
    }

    cursor = mongodb_client_->query(*c,
	     QUERY("timestamp" << GTE << start_msec << LT << end_msec).sort("timestamp"));

    while (cursor->more()) {
      doc = cursor->next();
      if (doc.hasField("transforms")) {
	// multi transforms document
	BSONObj::iterator i = doc.getObjectField("transforms").begin();
	while (i.more()) {
	  BSONElement e = i.next();
	  BSONObj o = e.Obj();
	  restore_tf_doc(o, start_msec, new_start_msec);
	}
     } else {
	restore_tf_doc(doc, start_msec, new_start_msec);
      }
    }
  }
}

} // end namespace tf
} // end namespace fawkes
