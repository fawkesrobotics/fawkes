
/***************************************************************************
 *  processor.cpp - Fawkes RefBox Processor Pure Virtual Class
 *
 *  Created: Sun Apr 19 19:15:46 2009 (German Open 2009)
 *  Copyright  2009  Tim Niemueller [www.niemueller.de]
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

#include "processor.h"

/** @class RefBoxProcessor "processor/processor.h"
 * Referee Box Communication Processor.
 * Classes that implement this interface communicate with a refbox and provide
 * information received to a RefBoxStateHandler.
 * @author Tim Niemueller
 *
 * @fn bool RefBoxProcessor::check_connection()
 * Check if the connection is alive and reconnect.
 * If the connection is not alive the processor shall do a single attempt
 * to reestablish the connection.
 * @return true if the connection is established or could be recovered, false
 * otherwise.
 *
 * @fn void RefBoxProcessor::refbox_process()
 * Process incoming refbox communication.
 * This shall process a (limited number of) communication packets received
 * from the particular refbox at hand.
 */

/** Pure virtual destructor. */
RefBoxProcessor::~RefBoxProcessor()
{
}

/** Set handler.
 * @param rsh Ref box state handler used to process incoming information
 */
void
RefBoxProcessor::set_handler(RefBoxStateHandler *rsh)
{
  _rsh = rsh;
}
