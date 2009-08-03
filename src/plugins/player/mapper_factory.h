
/***************************************************************************
 *  mapper_factory.h - Factory for Player proxy to Fawkes interface mappers
 *
 *  Created: Tue Sep 30 00:28:01 2008
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

#ifndef __PLUGINS_PLAYER_MAPPER_FACTORY_H_
#define __PLUGINS_PLAYER_MAPPER_FACTORY_H_

#include "mapper.h"

namespace fawkes {
  class Interface;
  class ObjectPositionInterface;
}

namespace PlayerCc {
  class ClientProxy;
  class Position2dProxy;
}

class PlayerMapperFactory
{
 public:
  static PlayerProxyFawkesInterfaceMapper *create_mapper(std::string varname,
							 fawkes::Interface *interface,
							 PlayerCc::ClientProxy *proxy);
							 

 private:
  PlayerMapperFactory() {}

  template <class FawkesInterfaceType, class PlayerProxyType, class MapperType>
    static PlayerProxyFawkesInterfaceMapper *  try_create(std::string varname,
							  fawkes::Interface *interface,
							  PlayerCc::ClientProxy *proxy);
};

/** Try to create a mapper instance.
 * Tries to dynamically cast the Fawkes interface and Player proxy to the
 * desired types, and if that succeeds instantiates a new mapper of the given type
 * giving the casted instances as parameters.
 * @param varname variable name
 * @param interface Fawkes interface instance
 * @param proxy Player proxy instance
 * @return NULL if a dynamic cast failed, a mapper instance for the given interface
 * and proxy otherwise
 */
template <class FawkesInterfaceType, class PlayerProxyType, class MapperType>
PlayerProxyFawkesInterfaceMapper *
PlayerMapperFactory::try_create(std::string varname,
				fawkes::Interface *interface,
				PlayerCc::ClientProxy *proxy)
{
  FawkesInterfaceType *fi;
  if ( (fi = dynamic_cast<FawkesInterfaceType *>(interface)) != NULL ) {
    PlayerProxyType *pp;
    if ( (pp = dynamic_cast<PlayerProxyType *>(proxy)) != NULL ) {
      return new MapperType(varname, fi, pp);
    } else {
      return NULL;
    }
  } else {
    return NULL;
  }
}


#endif
