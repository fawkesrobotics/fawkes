
/***************************************************************************
 *  mongodb.h - MongoDB aspect for Fawkes
 *
 *  Created: Mon Dec 06 00:28:55 2010
 *  Copyright  2006-2017  Tim Niemueller [www.niemueller.de]
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
#include <plugins/mongodb/aspect/mongodb_conncreator.h>

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

/** @fn const std::string & MongoDBAspect::mongodb_config_name() const
 * Get MongoDB configuration name.
 * @return MongoDB path name for the configuration settings from the
 * global configuration. Note that this may return 0 if the default
 * configuration should be used.
 */

/** @var mongo::DBClientBase *  MongoDBAspect::mongodb_client
 * MongoDB client to use to interact with the database. If database name, user
 * and password were given to constructor, authentication has been executed
 * (and only on success the aspect is considered to be successfully
 * initialized).
 */

/** @var fawkes::MongoDBConnCreator *  MongoDBAspect::mongodb_connmgr
 * Connection manager to retrieve more client connections from if necessary.
 */

/** Constructor.
 * @param config_name optional configuration name from which the
 * configuration for the database is read from the global configuration.
 */
MongoDBAspect::MongoDBAspect(const char *config_name)
{
	add_aspect("MongoDBAspect");
	mongodb_config_name_ = config_name;
}

/** Constructor.
 * Using this constructor will leave the mongodb_client member uninitialized.
 * The mongodb_connmgr can be used to create connections at a later point in time.
 */
MongoDBAspect::MongoDBAspect()
{
	add_aspect("MongoDBAspect");
}


/** Virtual empty destructor. */
MongoDBAspect::~MongoDBAspect()
{
	mongodb_config_name_.clear();
}


/** Init MongoDB aspect.
 * This set the MongoDB client to access MongoDB.
 * It is guaranteed that this is called for a MongoDBThread before start
 * is called (when running regularly inside Fawkes).
 * @param mongodb_client MongoDB connection
 * @param mongodb_connmgr MongoDB connection manager
 */
void
MongoDBAspect::init_MongoDBAspect(mongo::DBClientBase *mongodb_client,
                                  MongoDBConnCreator *mongodb_connmgr)
{
	this->mongodb_client  = mongodb_client;
	this->mongodb_connmgr = mongodb_connmgr;
}

} // end namespace fawkes
