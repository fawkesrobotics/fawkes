
/***************************************************************************
 *  mongodb_thread.cpp - MongoDB Thread
 *
 *  Created: Sun Dec 05 23:32:13 2010
 *  Copyright  2006-2010  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include "mongodb_thread.h"

using namespace std;
using namespace fawkes;

/** @class MongoDBThread "mongodb_thread.h"
 * MongoDB Thread.
 * This thread maintains an active connection to MongoDB and provides an
 * aspect to access MongoDB to make it convenient for other threads to use
 * MongoDB.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
MongoDBThread::MongoDBThread()
  : Thread("MongoDBThread", Thread::OPMODE_WAITFORWAKEUP)
{
}


/** Destructor. */
MongoDBThread::~MongoDBThread()
{
}


void
MongoDBThread::init()
{
}


void
MongoDBThread::finalize()
{
}


void
MongoDBThread::loop()
{
}
