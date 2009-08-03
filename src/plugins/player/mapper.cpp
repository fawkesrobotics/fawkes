
/***************************************************************************
 *  mapper.cpp - Player proxy to Fawkes interface mapper
 *
 *  Created: Tue Sep 30 00:50:29 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#include "mapper.h"

/** @class PlayerProxyFawkesInterfaceMapper "mapper.h"
 * Player proxy to Fawkes interface mapper interface.
 * This interface defines an interface to map a Player proxy to a Fawkes
 * interface.
 * @author Tim Niemueller
 *
 * @fn PlayerProxyFawkesInterfaceMapper::sync_fawkes_to_player() = 0
 * Sync Fawkes interface to Player proxy.
 * This method should be implemented to copy any outstanding data from the
 * Fawkes interface (messages) to the Player interface. This method will be
 * called in the ACT_EXEC hook of the BlockedTimingAspect (cf. MainLoop).
 *
 * @fn PlayerProxyFawkesInterfaceMapper::sync_player_to_fawkes() = 0
 * Sync Player proxy to Fawkes interface.
 * This method should be implemented to copy any outstanding data from the
 * Player proxy to the Fawkes interface. This method will be called in the
 * ACT_EXEC hook of the BlockedTimingAspect (cf. MainLoop).
 */

/** Constructor.
 * @param varname variable name
 */
PlayerProxyFawkesInterfaceMapper::PlayerProxyFawkesInterfaceMapper(std::string varname)
{
  __varname = varname;
}


/** Virtual empty destructor. */
PlayerProxyFawkesInterfaceMapper::~PlayerProxyFawkesInterfaceMapper()
{
}


/** Get variable name
 * @return variable name
 */
std::string
PlayerProxyFawkesInterfaceMapper::varname() const
{
  return __varname;
}
