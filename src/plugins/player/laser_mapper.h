
/***************************************************************************
 *  laser_mapper.h - Mapper LaserProxy to Lsaer360Interface
 *
 *  Created: Tue Oct 21 00:48:45 2008
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

#ifndef __PLUGINS_PLAYER_LASER_MAPPER_H_
#define __PLUGINS_PLAYER_LASER_MAPPER_H_

#include "mapper.h"

namespace fawkes {
  class Laser360Interface;
}

namespace PlayerCc {
  class LaserProxy;
}

class PlayerLaserMapper : public PlayerProxyFawkesInterfaceMapper
{
 public:
  PlayerLaserMapper(std::string varname,
		    fawkes::Laser360Interface *interface,
		    PlayerCc::LaserProxy *proxy);

  virtual void sync_fawkes_to_player();
  virtual void sync_player_to_fawkes();

 private:
  fawkes::Laser360Interface  *__interface;
  PlayerCc::LaserProxy       *__proxy;

  bool                        __first_read;
  unsigned int                __index_offset;
};

#endif
