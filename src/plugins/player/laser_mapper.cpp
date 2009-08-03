
/***************************************************************************
 *  laser_mapper.cpp - Mapper LaserProxy to LaserInterface
 *
 *  Created: Tue Oct 21 00:50:26 2008
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

#include "laser_mapper.h"

#include <core/exceptions/software.h>
#include <utils/math/angle.h>
#include <interfaces/Laser360Interface.h>
#include <libplayerc++/playerc++.h>

/** @class PlayerLaserMapper "laser_mapper.h"
 * Laser mapper for player integration.
 * This class is used to map a Player lsaer proxy to a Fawkes
 * Laser360Interface.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param varname variable name
 * @param interface Fawkes interface instance
 * @param proxy Player proxy instance
 */
PlayerLaserMapper::PlayerLaserMapper(std::string varname,
				     fawkes::Laser360Interface *interface,
				     PlayerCc::LaserProxy *proxy)
  : PlayerProxyFawkesInterfaceMapper(varname)
{
  __interface  = interface;
  __proxy      = proxy;
  __first_read = true;
}


void
PlayerLaserMapper::sync_player_to_fawkes()
{
  //printf("Laser interface, count: %u, min angle= %f, max angle = %f\n",
  //	 __proxy->GetCount(), __proxy->GetMinAngle(), __proxy->GetMaxAngle());

  if ( __proxy->GetCount() != 360 )  return;

  if ( __proxy->IsFresh() ) {

    if ( __first_read ) {
      __index_offset = 360 + fawkes::rad2deg(__proxy->GetMinAngle());
      __first_read = false;
    }

    //printf("Setting %s to (%f, %f, %f)\n", varname().c_str(), __proxy->GetXPos(),
    //       __proxy->GetYPos(), __proxy->GetYaw());
    float distances[360];
    for (int i = 0; i < 360; ++i) {
      distances[(i + __index_offset) % 360] = (*__proxy)[360 - i];
    }
    __interface->set_distances(distances);
    __interface->write();
    __proxy->NotFresh();
  }
}

void
PlayerLaserMapper::sync_fawkes_to_player()
{
}
