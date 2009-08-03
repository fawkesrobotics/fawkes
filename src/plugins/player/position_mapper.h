
/***************************************************************************
 *  position_mapper.h - Mapper Position2dProxy to ObjectPositionInterface
 *
 *  Created: Tue Sep 30 00:51:51 2008
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

#ifndef __PLUGINS_PLAYER_POSITION_MAPPER_H_
#define __PLUGINS_PLAYER_POSITION_MAPPER_H_

#include "mapper.h"

namespace fawkes {
  class ObjectPositionInterface;
}

namespace PlayerCc {
  class Position2dProxy;
}

class PlayerPositionMapper : public PlayerProxyFawkesInterfaceMapper
{
 public:
  PlayerPositionMapper(std::string varname,
		       fawkes::ObjectPositionInterface *interface,
		       PlayerCc::Position2dProxy *proxy);

  virtual void sync_fawkes_to_player();
  virtual void sync_player_to_fawkes();

 private:
  fawkes::ObjectPositionInterface *__interface;
  PlayerCc::Position2dProxy       *__proxy;
};

#endif
