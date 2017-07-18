
/***************************************************************************
 *  mongodb_conncreator.h - Fawkes MongoDB connection creator
 *
 *  Created: Mon Dec 06 21:20:16 2010
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

#ifndef __PLUGINS_MONGODB_ASPECT_MONGODB_CONNCREATOR_H_
#define __PLUGINS_MONGODB_ASPECT_MONGODB_CONNCREATOR_H_

#include <string>

namespace mongo {
	class DBClientBase;
}

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class MongoDBConnCreator <plugins/mongodb/aspect/mongodb_conncreator.h>
 * Interface for a MongoDB connection creator.
 * @author Tim Niemueller
 */
class MongoDBConnCreator
{
 public:
	/** Create a new MongoDB client.
	 * @param config_name MongoDB client configuration name for the desired
	 * connection. May be empty in which case the default configuration is used.
	 * @return MongoDB client for the given (or the default) configuration.
	 * @exception thrown if the initialization fails or the configuration for
	 * the given name does not exist.
	 */
	virtual mongo::DBClientBase *  create_client(const std::string &config_name = "") = 0;

	/** Delete a client.
	 * @param client client to delete
	 */
	virtual void delete_client(mongo::DBClientBase *client) = 0;
};

} // end namespace fawkes

#endif
