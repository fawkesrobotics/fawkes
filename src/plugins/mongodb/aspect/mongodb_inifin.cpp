
/***************************************************************************
 *  mongodb_inifin.cpp - Fawkes MongoDBAspect initializer/finalizer
 *
 *  Created: Mon Dec 06 22:33:03 2010
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

#include <plugins/mongodb/aspect/mongodb_inifin.h>
#include <plugins/mongodb/aspect/mongodb.h>
#include <plugins/mongodb/aspect/mongodb_conncreator.h>
#include <core/threading/thread_finalizer.h>
#include <cstddef>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

MongoDBAspectIniFin::MongoDBAspectIniFin(MongoDBConnCreator *conn_creator)
  : AspectIniFin("MongoDBAspect")
{
  __conn_creator = conn_creator;
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

  mongo::DBClientBase *client = 
    __conn_creator->create_client(mongodb_thread->__mongodb_name,
				  mongodb_thread->__mongodb_user,
				  mongodb_thread->__mongodb_pass);

  mongodb_thread->init_MongoDBAspect(client);
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

  __conn_creator->delete_client(mongodb_thread->mongodb_client);
}



} // end namespace fawkes
