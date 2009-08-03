
/***************************************************************************
 *  blackboard_thread.cpp - Fawkes Example Plugin BlackBoard Thread
 *
 *  Created: Wed Jun 20 16:37:40 2007
 *  Copyright  2007-2008  Tim Niemueller [www.niemueller.de]
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

#include <plugins/examples/basics/blackboard_thread.h>
#include <interfaces/TestInterface.h>

using namespace fawkes;


/** @class ExampleBlackBoardThread <plugins/examples/basics/blackboard_thread.h>
 * Simple demonstration for a thread using the BlackBoard.
 *
 * @author Tim Niemueller
 */


/** Constructor.
 * @param reader set to true, to make this bb thread to open the test interface
 * read-only, false to open it as a writer
 */
ExampleBlackBoardThread::ExampleBlackBoardThread(bool reader)
  : Thread("ExampleBlackBoardThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_THINK)
{
  this->reader = reader;
}


/** Destructor. */
ExampleBlackBoardThread::~ExampleBlackBoardThread()
{
}


void
ExampleBlackBoardThread::finalize()
{
  logger->log_debug(name(), "Closing test interface");
  try {
    blackboard->close(test_interface);
  } catch (Exception &e) {
    logger->log_error(name(), "Could not close kicker interface");
    logger->log_error(name(), e);
  }
}


/** Initialize thread.
 * Here, the device and the BB-interface are opened.
 */
void
ExampleBlackBoardThread::init()
{
  logger->log_debug(name(), "Opening test interface");
  try {
    if ( reader ) {
      test_interface = blackboard->open_for_reading<TestInterface>("Test");
    } else {
      test_interface = blackboard->open_for_writing<TestInterface>("Test");
    }
  } catch (Exception& e) {
    e.append("Opening test interface for writing failed");
    throw;
  }
}

/** Thread loop.
 * Parse messages from the interface and update values in the interface.
 */
void
ExampleBlackBoardThread::loop()
{
  // nothin'
}
