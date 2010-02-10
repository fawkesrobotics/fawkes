
/***************************************************************************
 *  mapper_factory.cpp - Factory for Player proxy to Fawkes interface mappers
 *
 *  Created: Tue Sep 30 00:41:08 2008
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

#include "mapper_factory.h"
#include "position_mapper.h"
#include "motor_mapper.h"
#include "laser_mapper.h"

#include <interfaces/ObjectPositionInterface.h>
#include <interfaces/MotorInterface.h>
#include <interfaces/Laser360Interface.h>
#include <libplayerc++/playerc++.h>

using namespace PlayerCc;
using namespace fawkes;

/** @class PlayerMapperFactory "mapper_factory.h"
 * Player Fawkes mapper factory.
 * Factory class to create mappers from Fawkes interfaces to Player proxies.
 * @author Tim Niemueller
 */

/** Create a mapp instance.
 * Tries to figure out the type of the interface and proxy and if a known matching
 * exists will return an appropriate mapper.
 * @param varname variable name
 * @param interface Fawkes interface instance
 * @param proxy Player proxy instance
 * @return a mapper instance for the given interface and proxy otherwise
 * @exception Exception thrown if no known mapping exists for the given
 * interfaces.
 */
PlayerProxyFawkesInterfaceMapper *
PlayerMapperFactory::create_mapper(std::string varname,
				   fawkes::Interface *interface,
				   PlayerCc::ClientProxy *proxy)
{
  PlayerProxyFawkesInterfaceMapper *rv = NULL;

  if ( (rv = try_create<ObjectPositionInterface, Position2dProxy, PlayerPositionMapper>(varname, interface, proxy)) != NULL ) {
    return rv;
  } else if ( (rv = try_create<MotorInterface, Position2dProxy, PlayerMotorPositionMapper>(varname, interface, proxy)) != NULL ) {
    return rv;
  } else if ( (rv = try_create<Laser360Interface, LaserProxy, PlayerLaserMapper>(varname, interface, proxy)) != NULL ) {
    return rv;
  } else {
    throw Exception("Unknown mapping, don't know how to map Fawkes interface %s "
		    "to Player proxy %s",
		    interface->type(), proxy->GetInterfaceStr().c_str());
  }
}
