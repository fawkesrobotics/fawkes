
/***************************************************************************
 *  mongodb_inifin.cpp - Fawkes MongoDBAspect initializer/finalizer
 *
 *  Created: Mon Dec 06 22:33:03 2010
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

#include <plugins/mongodb/aspect/mongodb_inifin.h>
#include <plugins/mongodb/aspect/mongodb.h>
#include <plugins/mongodb/aspect/mongodb_conncreator.h>
#include <core/threading/thread_finalizer.h>
#include <cstddef>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class MongoDBAspectIniFin <plugins/mongodb/aspect/mongodb_inifin.h>
 * MongoDBAspect initializer/finalizer.
 * This initializer/finalizer will use the MongoDBConnCreator instance to
 * create client instances as requested by a thread.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param conn_creator connection creator to use for initializing threads
 */
MongoDBAspectIniFin::MongoDBAspectIniFin(MongoDBConnCreator *conn_creator)
	: AspectIniFin("MongoDBAspect")
{
	conn_creator_ = conn_creator;
}

void
MongoDBAspectIniFin::init(Thread *thread)
{
	MongoDBAspect *mongodb_thread;
	mongodb_thread = dynamic_cast<MongoDBAspect *>(thread);
	if (mongodb_thread == NULL) {
		throw CannotInitializeThreadException("Thread '%s' claims to have the "
		                                      "MongoDBAspect, but RTTI says it "
		                                      "has not. ", thread->name());
	}

	mongo::DBClientBase *client = nullptr;

	if (! mongodb_thread->mongodb_config_name().empty()) {
		conn_creator_->create_client(mongodb_thread->mongodb_config_name());
	}

	mongodb_thread->init_MongoDBAspect(client, conn_creator_);
}

void
MongoDBAspectIniFin::finalize(Thread *thread)
{
	MongoDBAspect *mongodb_thread;
	mongodb_thread = dynamic_cast<MongoDBAspect *>(thread);
	if (mongodb_thread == NULL) {
		throw CannotFinalizeThreadException("Thread '%s' claims to have the "
		                                    "MongoDBAspect, but RTTI says it "
		                                    "has not. ", thread->name());
	}

	conn_creator_->delete_client(mongodb_thread->mongodb_client);
}

} // end namespace fawkes
