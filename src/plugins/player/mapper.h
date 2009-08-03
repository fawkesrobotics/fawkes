
/***************************************************************************
 *  mapper.h - Player proxy to Fawkes interface mapper
 *
 *  Created: Tue Sep 30 00:40:16 2008
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

#ifndef __PLUGINS_PLAYER_MAPPER_H_
#define __PLUGINS_PLAYER_MAPPER_H_

#include <string>

class PlayerProxyFawkesInterfaceMapper
{
 public:
  PlayerProxyFawkesInterfaceMapper(std::string varname);
  virtual ~PlayerProxyFawkesInterfaceMapper();

  std::string varname() const;

  virtual void sync_fawkes_to_player() = 0;
  virtual void sync_player_to_fawkes() = 0;

 private:
  std::string __varname;
};

#endif
