
/***************************************************************************
 *  or_thread.cpp - OpenRAVE Thread
 *
 *  Created: Fri Feb 25 15:08:00 2011
 *  Copyright  2011  Bahram Maleki-Fard
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

#include "or_thread.h"

using namespace fawkes;

/** @class OpenRAVEThread "or_thread.h"
 * OpenRAVE Thread.
 * This thread maintains an active connection to OpenRAVE and provides an
 * aspect to access OpenRAVE to make it convenient for other threads to use
 * OpenRAVE.
 *
 * @author Bahram Maleki-Fard
 */

/** Constructor. */
OpenRAVEThread::OpenRAVEThread()
  : Thread("OpenRAVEThread", Thread::OPMODE_CONTINUOUS),
    AspectProviderAspect("OpenRAVEAspect", &__or_aspectIniFin),
    __or_aspectIniFin(this)
{
}


/** Destructor. */
OpenRAVEThread::~OpenRAVEThread()
{
}


void
OpenRAVEThread::init()
{
}


void
OpenRAVEThread::finalize()
{
}


void
OpenRAVEThread::loop()
{
}