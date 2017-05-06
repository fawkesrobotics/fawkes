
/***************************************************************************
 *  stn-generator_thread.cpp - stn-generator
 *
 *  Created: Sat May  6 20:16:21 2017
 *  Copyright  2017  Matthias Loebach
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

#include "stn-generator_thread.h"

using namespace fawkes;

/** @class StnGeneratorThread 'stn-generator_thread.h' 
 * Generates an STN representation of a sequential task plan
 * @author Matthias Loebach
 */

/** Constructor. */
StnGeneratorThread::StnGeneratorThread()
 : Thread("StnGeneratorThread", Thread::OPMODE_WAITFORWAKEUP)
{
}

void
StnGeneratorThread::init()
{
}

void
StnGeneratorThread::loop()
{
}

void
StnGeneratorThread::finalize()
{
}

