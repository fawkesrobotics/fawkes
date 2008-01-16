
/***************************************************************************
 *  blackboard.cpp - BlackBoard plugin
 *
 *  Generated: Sat Sep 16 17:11:13 2006 (on train to Cologne)
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#include <blackboard/blackboard.h>
#include <blackboard/main_thread.h>

/** @class BlackBoard <blackboard/blackboard.h>
 * BlackBoard main class.
 * This class has to be instantiated to create a BlackBoard with exclusive
 * access. It is the main entry point for the BlackBoard.
 * @author Tim Niemueller
 */

/** Constructor. */
BlackBoard::BlackBoard()
{
  im = new BlackBoardInterfaceManager(/* master */ true);
}


/** Destructor. */
BlackBoard::~BlackBoard()
{
  delete im;
}


/** Get interface manager.
 * @return interface manager.
 */
BlackBoardInterfaceManager *
BlackBoard::interface_manager()
{
  return im;
}

