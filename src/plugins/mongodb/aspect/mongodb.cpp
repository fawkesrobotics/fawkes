
/***************************************************************************
 *  mongodb.h - MongoDB aspect for Fawkes
 *
 *  Created: Mon Dec 06 00:28:55 2010
 *  Copyright  2006-2010  Tim Niemueller [www.niemueller.de]
 *
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

#include <plugins/mongodb/aspect/mongodb.h>

#include <cstring>
#include <cstdlib>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class MongoDBAspect <plugins/mongodb/aspect/mongodb.h>
 * Thread aspect to access MongoDB.
 * Give this aspect to your thread to gain access to MongoDB. This will
 * setup the mongodb_client member with an active, auto-recovering connection
 * to MongoDB (can be any kind of connection, single server, replicat set,
 * or sync cluster).
 *
 * @ingroup Aspects
 * @author Tim Niemueller
 */


/** @var mongo::DBClientBase *  MongoDBAspect::mongodb_client
 * MongoDB client to use to interact with the database. If database name, user
 * and password were given to constructor, authentication has been executed
 * (and only on success the aspect is considered to be successfully
 * initialized).
 */

/** Constructor. */
MongoDBAspect::MongoDBAspect()
{
  add_aspect("MongoDBAspect");
  __mongodb_name = __mongodb_user = __mongodb_pass = 0;
}

/** Constructor with authentication.
 * @param dbname database name to authenticate
 * @param user username
 * @param clearpwd password in clear text
 */
MongoDBAspect::MongoDBAspect(const char *dbname,
			     const char *user, const char *clearpwd)
{
  add_aspect("MongoDBAspect");
  __mongodb_name = strdup(dbname);
  __mongodb_user = strdup(user);
  __mongodb_pass = strdup(clearpwd);
}


/** Virtual empty destructor. */
MongoDBAspect::~MongoDBAspect()
{
  if (__mongodb_name)  free(__mongodb_name);
  if (__mongodb_user)  free(__mongodb_user);
  if (__mongodb_pass)  free(__mongodb_pass);
}


/** Init MongoDB aspect.
 * This set the MongoDB client to access MongoDB.
 * It is guaranteed that this is called for a MongoDBThread before start
 * is called (when running regularly inside Fawkes).
 * @param bb MongoDB to use
 */
void
MongoDBAspect::init_MongoDBAspect(mongo::DBClientBase *mongodb_client)
{
  this->mongodb_client = mongodb_client;
}

} // end namespace fawkes
