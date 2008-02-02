
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
#include <plugins/kicker/kicker_control.h>

#include <interfaces/kicker.h>

/** @class KickerThread <plugins/kicker/kicker_thread.h>
 * This thread is started by the Kicker plugin and is the connection
 * between the kicker hardware and the blackboard.
 *
 * @author Daniel Beck
 */


/** Constructor. */
KickerThread::KickerThread()
  : Thread("KickerThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT)
{
  kicker_control = new KickerControl();
}


/** Destructor. */
KickerThread::~KickerThread()
{
  delete kicker_control;
}


void
KickerThread::finalize()
{
  try {
    blackboard->close(kicker_interface);
  } catch (Exception &e) {
    logger->log_error(name(), "Could not close kicker interface");
    logger->log_error(name(), e);
  }
  kicker_control->close();
}


/** Initialize thread.
 * Here, the device and the BB-interface are opened.
 */
void
KickerThread::init()
{
  if ( ! kicker_control->open() )
    {
      throw Exception("Opening IOWarrior failed");
    }

  try
    {
      kicker_interface = blackboard->open_for_writing<KickerInterface>("Kicker");
    }
  catch (Exception& e)
    {
      kicker_control->close();
      e.append("Opening Kicker interface for writing failed");
      throw;
    }

  kicker_control->set_intensity(0);

  kicker_interface->set_num_kicks_left(0);
  kicker_interface->set_num_kicks_center(0);
  kicker_interface->set_num_kicks_right(0);
  kicker_interface->set_current_intensity(0);
  // per default our ball guidance device guides right if nothing has been done
  // (and the cylinder is in retracted state)
  kicker_interface->set_guide_ball_side(KickerInterface::GUIDE_BALL_RIGHT);
  kicker_interface->write();
}

/** Thread loop.
 * Parse messages from the interface and update values in the interface.
 */
void
KickerThread::loop()
{
  if ( kicker_interface->msgq_empty() )
    {
      return;
    }

  if ( kicker_interface->msgq_first_is<KickerInterface::KickMessage>() )
    {
      KickerInterface::KickMessage* msg = kicker_interface->msgq_first<KickerInterface::KickMessage>();

      unsigned int intensity = msg->intensity();
      if (intensity <= 0xFF)
	{
          if ( intensity != kicker_interface->current_intensity() )
            {
              kicker_control->set_intensity(intensity);
              kicker_interface->set_current_intensity(intensity);
              // kicker_interface->write() is done below in any case!
            }
	}
      else
        {
           logger->log_warn(name(), "KickMessage received with illegal intensity value: %u", intensity);
        }

      logger->log_debug(name(), "Kicking  left: %i  center: %i  right: %i",
			msg->is_right(),
			msg->is_center(),
			msg->is_left());

      kicker_control->kick(msg->is_right(),
			   msg->is_center(),
			   msg->is_left());

      unsigned int right, center, left;

      kicker_control->get_num_kicks(right, center, left);

      kicker_interface->set_num_kicks_right(right);
      kicker_interface->set_num_kicks_center(center);
      kicker_interface->set_num_kicks_left(left);
      kicker_interface->write();
    }
  else if ( kicker_interface->msgq_first_is<KickerInterface::ResetCounterMessage>() )
    {
      kicker_control->reset_counter();
      unsigned int right, center, left;

      kicker_control->get_num_kicks(right, center, left);

      kicker_interface->set_num_kicks_right(right);
      kicker_interface->set_num_kicks_center(center);
      kicker_interface->set_num_kicks_left(left);
      kicker_interface->write();
    }
  else if ( kicker_interface->msgq_first_is<KickerInterface::GuideBallMessage>() )
    {
      KickerInterface::GuideBallMessage* msg = kicker_interface->msgq_first<KickerInterface::GuideBallMessage>();
      if ( msg->guide_ball_side() == KickerInterface::GUIDE_BALL_LEFT ) {
	if ( kicker_control->guidance_left() ) {
	  kicker_interface->set_guide_ball_side(KickerInterface::GUIDE_BALL_LEFT);
	  kicker_interface->write();
          logger->log_debug(name(), "Enabled ball guidance on LEFT side");
	} else {
	  logger->log_error(name(), "Could not activate ball guidance on LEFT side");
	}
      } else if ( msg->guide_ball_side() == KickerInterface::GUIDE_BALL_RIGHT ) {
	if ( kicker_control->guidance_right() ) {
	  kicker_interface->set_guide_ball_side(KickerInterface::GUIDE_BALL_RIGHT);
	  kicker_interface->write();
          logger->log_debug(name(), "Enabled ball guidance on RIGHT side");
	} else {
	  logger->log_error(name(), "Could not activate ball guidance on RIGHT side");
	}
      } else {
	logger->log_error(name(), "Invalid ball guide side received: %i", msg->guide_ball_side());
      }
    }
  else
    {
      logger->log_error(name(), "Unknown message received via KickerInterface (%u messages in queue)", kicker_interface->msgq_size());
    }

  kicker_interface->msgq_pop();
}
