
/***************************************************************************
 *  position_mapper.cpp - Mapper Position2dProxy to ObjectPositionInterface
 *
 *  Created: Tue Sep 30 00:53:38 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#include "position_mapper.h"

#include <interfaces/ObjectPositionInterface.h>
#include <libplayerc++/playerc++.h>

/** @class PlayerPositionMapper "position_mapper.h"
 * Position mapper for player integration.
 * This class is used to map a Player position2d proxy to a Fawkes
 * ObjectPositionInterface.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param varname variable name
 * @param interface Fawkes interface instance
 * @param proxy Player proxy instance
 */
PlayerPositionMapper::PlayerPositionMapper(const std::string& varname,
                                           fawkes::ObjectPositionInterface *interface,
                                           PlayerCc::Position2dProxy *proxy)
: PlayerProxyFawkesInterfaceMapper(varname)
{
  interface_ = interface;
  proxy_     = proxy;
}


void
PlayerPositionMapper::sync_player_to_fawkes()
{
  if ( proxy_->IsFresh() ) {
    //printf("Setting %s to (%f, %f, %f)\n", varname().c_str(), proxy_->GetXPos(),
    //       proxy_->GetYPos(), proxy_->GetYaw());
    interface_->set_relative_x(proxy_->GetXPos());
    interface_->set_relative_y(proxy_->GetYPos());
    interface_->set_relative_z(proxy_->GetYaw());
    interface_->write();
    proxy_->NotFresh();
  }
}

void
PlayerPositionMapper::sync_fawkes_to_player()
{
}
