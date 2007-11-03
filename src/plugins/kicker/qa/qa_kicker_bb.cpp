
/***************************************************************************
 *  qa_kicker_bb.cpp - Kicker Control QA Application (via BlackBoard)
 *
 *  Created: Wed Jul 25 16:26:22 2007
 *  Copyright  2007  Tim Niemueller
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

/// @cond QA

#include <core/threading/thread.h>
#include <core/plugin.h>
#include <aspect/blackboard.h>
#include <aspect/logging.h>
#include <interfaces/kicker.h>

class KickerQAThread : public Thread, public BlackBoardAspect, public LoggingAspect
{
 public:
  KickerQAThread()
    : Thread("KickerQAThread")
  {
    kicked = false;
    mod_counter = 0;
  }

  virtual void
  init()
  {
    try  {
	kicker_interface = interface_manager->open_for_reading<KickerInterface>("Kicker");
    } catch (Exception& e) {
      e.append("%s initialization failed, could not open kicker interface for reading", name());
      logger->log_error("NavigatorNetworkThread", "Opening interface for reading failed!");
      logger->log_error("NavigatorNetworkThread", e);
      throw;
    }
  }

  virtual void
  finalize()
  {
    try {
      interface_manager->close(kicker_interface);
    } catch (Exception& e) {
      logger->log_error("NavigatorNetworkThread", "Closing kicker interface failed!");
      logger->log_error("NavigatorNetworkThread", e);
    }
  }

  virtual void
  loop()
  {
    if ( ! kicker_interface->has_writer() ) {
      if ( (++mod_counter %= 20) == 0 ) {
	logger->log_warn(name(), "No writer for kicker interface, kicker not loaded?");
      }
    }
    if ( ! kicked ) {
      KickerInterface::KickMessage *msg =
	new KickerInterface::KickMessage(false, true, false, 150);

      kicker_interface->msgq_enqueue(msg);
      kicked = true;
    }
  }

 private:
  bool kicked;
  unsigned int mod_counter;
  KickerInterface *kicker_interface;
};


class KickerQAPlugin : public Plugin
{
 public:
  KickerQAPlugin()
    : Plugin(Plugin::MOTION, "KickerQAPlugin")
  {
    thread_list.push_back(new KickerQAThread());
  }
};

EXPORT_PLUGIN(KickerQAPlugin);

/// @endcond
