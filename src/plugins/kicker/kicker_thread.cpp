
/***************************************************************************
 *  kicker_thread.cpp - Fawkes Kicker Plugin Thread
 *
 *  Generated: Mon May 14 15:43:59 2007
 *  Copyright  2007  Daniel Beck
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <plugins/kicker/kicker_thread.h>
#include <iostream>

/** @class KickerThread plugins/kicker/kicker_thread.h
 * This thread is started by the Kicker plugin and is the connection
 * between the kicker hardware and the blackboard.
 *
 * @author Daniel Beck
 */

using namespace std;

/** Constructor.
 * @param hook hook to register this thread for
 * @param name thread name
 */
KickerThread::KickerThread(BlockedTimingAspect::WakeupHook hook,
			   const char* name)
  : Thread(name, Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(hook)
{
  kicker_control = new KickerControl();
}

/** Destructor. */
KickerThread::~KickerThread()
{
  delete kicker_control;
  
  logger->log_info("KickerThread", "Destroying thread %s", name());
}

/** Initialize thread.
 * Here, the device and the BB-interface are opened.
 */
void
KickerThread::init()
{
  if ( !kicker_control->open() )
    {
      logger->log_info("KickerThread", "Opening IOWarrior failed.");
    }

  try 
    {
      kicker_interface = interface_manager->openForWriting<KickerInterface>("Kicker");
    }
  catch (Exception& e)
    {
      cout << "Opening interface failed!" << endl;
      e.printTrace();
    }
}

/** Thread loop.
 * Parse messages from the interface and update values in the interface.
 */
void
KickerThread::loop()
{
  if ( kicker_interface->msgq_first_is<KickerInterface::KickMessage>() )
    {
      KickerInterface::KickMessage* msg = kicker_interface->msgq_first<KickerInterface::KickMessage>();

      int intensity = msg->getIntensity();
      if (0 < intensity)
	kicker_control->set_intensity(intensity);
      kicker_control->kick(msg->isCmdKickRight(),
			   msg->isCmdKickCenter(),
			   msg->isCmdKickLeft());

      if (msg->isCmdResetCounter())
	kicker_control->reset_counter();

      
      unsigned int right, center, left;

      kicker_control->get_num_kicks(right, center, left);

      kicker_interface->setNumKicksRight(right);
      kicker_interface->setNumKicksCenter(center);
      kicker_interface->setNumKicksLeft(left);

      kicker_interface->msgq_pop();
    }
}
