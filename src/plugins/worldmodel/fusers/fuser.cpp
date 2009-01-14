
/***************************************************************************
 *  fuser.cpp - Fawkes WorldModel Fuser Interface
 *
 *  Created: Tue Jan 13 17:17:30 2009
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#include "fuser.h"

/** @class WorldModelFuser "fuser.h"
 * Interface for data fusers for the world model.
 * World model fusers take one or more input interfaces, mangle the content
 * in some way and then spit it into one or more other interfaces. The simplest
 * can be to just copy values for when there is nothing useful to do, but you
 * want to provide a unified world model. More complex scenarios can involve
 * things like generating a fused output from multiple inputs (like Kalman
 * filtered obstacles positions that are grouped in another step such that
 * multiple readings merge to a single obstacle in the world model).
 * @author Tim Niemueller
 *
 * @fn void WorldModelFuser::fuse() = 0
 * The single function that makes fusers work. In this method fusers shall
 * read from their source interfaces, process the data and write to their output
 * interfaces.
 *
 */


/** Virtual empty destructor. */
WorldModelFuser::~WorldModelFuser()
{
}
