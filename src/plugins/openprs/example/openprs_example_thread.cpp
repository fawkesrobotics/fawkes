
/***************************************************************************
 *  openprs_example_thread.cpp -  OpenPRS example thread
 *
 *  Created: Tue Aug 19 11:51:33 2014
 *  Copyright  2014  Tim Niemueller [www.niemueller.de]
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

#include "openprs_example_thread.h"

#include <logging/logger.h>
#include <plugins/openprs/utils/openprs_comm.h>

#include <unistd.h>

using namespace fawkes;

/** @class OpenPRSExampleThread "openprs_example_thread.h"
 * OpenPRS example thread.
 * @author Tim Niemueller
 */

/** Constructor. */
OpenPRSExampleThread::OpenPRSExampleThread()
  : Thread("OpenPRSExampleThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_THINK),
    OpenPRSAspect("openprs-example", OpenPRSAspect::OPRS)
{
}


/** Destructor. */
OpenPRSExampleThread::~OpenPRSExampleThread()
{
}


void
OpenPRSExampleThread::init()
{
  openprs.lock();
  openprs->send_message(openprs_kernel_name, "(foo bar)");
  openprs->transmit_command(openprs_kernel_name, "add (foo2 bar)");
  openprs.unlock();
}

void
OpenPRSExampleThread::finalize()
{
}


void
OpenPRSExampleThread::loop()
{
  static int modcount = 0;
  if (++modcount % 50 == 0) {
    openprs.lock();
    openprs->transmit_command(openprs_kernel_name, "show db");
    openprs.unlock();
  }
}
