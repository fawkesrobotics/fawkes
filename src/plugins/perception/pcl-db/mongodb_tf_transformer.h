/***************************************************************************
 *  mongodb_tf_transformer.h - Read and provide TFs from MongoDB
 *
 *  Created: Thu Nov 29 22:55:41 2012
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

#ifndef __PLUGINS_PERCEPTION_PCL_DB_MERGE_MONGODB_TF_TRANSFORMER_H_
#define __PLUGINS_PERCEPTION_PCL_DB_MERGE_MONGODB_TF_TRANSFORMER_H_

#include <tf/types.h>
#include <tf/transformer.h>

#include <mongo/client/dbclient.h>

#include <string>

namespace fawkes {
  namespace tf {
#if 0 /* just to make Emacs auto-indent happy */
  }
}
#endif

class MongoDBTransformer
: public Transformer
{
 public:
  MongoDBTransformer(mongo::DBClientBase *mongodb_client,
		     std::string database_name, bool ensure_index = true);
  virtual ~MongoDBTransformer();

  /** Restore transforms from database.
   * @param start start time of range to restore
   * @param end end time of range to restore
   */
  void restore(fawkes::Time &start, fawkes::Time &end)
  { fawkes::Time no_new_start(0,0); restore(start, end, no_new_start); }

  void restore(fawkes::Time &start, fawkes::Time &end, fawkes::Time &new_start);
  void restore(long long start_msec, long long end_msec,
	       long long new_start_msec = 0);

 private:
  void restore_tf_doc(mongo::BSONObj &doc,
		      long long start_msec, long long new_start_msec);

 private:
  mongo::DBClientBase *mongodb_client_;
  std::string database_;
};

} // end namespace tf
} // end namespace fawkes

#endif
