
/***************************************************************************
 *  mongodb.h - MongoDB aspect for Fawkes
 *
 *  Created: Mon Dec 06 00:24:43 2010
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

#ifndef __PLUGINS_MONGODB_ASPECT_MONGODB_H_
#define __PLUGINS_MONGODB_ASPECT_MONGODB_H_

#include <aspect/aspect.h>
#include <string>

namespace mongo {
	class DBClientBase;
}

namespace fawkes {
	class MongoDBConnCreator;

#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class MongoDBAspectIniFin;

class MongoDBAspect : public virtual Aspect
{
	friend MongoDBAspectIniFin;

 public:
	MongoDBAspect(const char *config_prefix);
	MongoDBAspect();
	virtual ~MongoDBAspect();

	const std::string & mongodb_config_name() const
	{ return mongodb_config_name_; }

 protected:
	mongo::DBClientBase *mongodb_client;
	MongoDBConnCreator  *mongodb_connmgr;

 private:
	void init_MongoDBAspect(mongo::DBClientBase *mongodb_client,
	                        MongoDBConnCreator  *mongodb_connmgr);

 private:
	std::string mongodb_config_name_;
};

} // end namespace fawkes

#endif
