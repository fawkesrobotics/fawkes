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
 */
MongoDBTransformer::MongoDBTransformer(mongo::DBClientBase *mongodb_client,
				       std::string database_name)
  : mongodb_client_(mongodb_client), database_(database_name)
{
}


/** Destructor. */
MongoDBTransformer::~MongoDBTransformer()
{
}

void
MongoDBTransformer::restore(fawkes::Time &start, fawkes::Time &end, fawkes::Time &new_start)
{
  restore(start.in_msec(), end.in_msec(), new_start.in_msec());
}


void
MongoDBTransformer::restore(long long start_msec, long long end_msec, long long new_start_msec)
{
  if (new_start_msec == 0) {
    new_start_msec = start_msec;
  }

  std::list<std::string> collections =
    mongodb_client_->getCollectionNames(database_);
  
  std::auto_ptr<DBClientCursor> cursor;
  BSONObj doc;
  //long long buffered = start_msec;
  std::list<std::string>::iterator c;
  for(c = collections.begin(); c != collections.end(); ++c) {
    if (c->find(database_ + ".TransformInterface.") != 0)  continue;

    cursor = mongodb_client_->query(*c,
	     QUERY("timestamp" << GTE << start_msec << LT << end_msec).sort("timestamp"));

    while (cursor->more()) {
      doc = cursor->next();
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
      set_transform(transform);
 
      //if(doc["timestamp"].Number() > buffered)
      //  buffered = doc["timestamp"].Number();
    }
  }
}

} // end namespace tf
} // end namespace fawkes
