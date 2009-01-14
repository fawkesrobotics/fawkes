
/***************************************************************************
 *  fuser.h - Fawkes WorldModel Fuser Interface
 *
 *  Created: Tue Jan 13 11:21:13 2009
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

#ifndef __PLUGINS_WORLDMODEL_FUSER_FUSER_H_
#define __PLUGINS_WORLDMODEL_FUSER_FUSER_H_

class WorldModelFuser
{
 public:
  virtual ~WorldModelFuser();

  virtual void fuse() = 0;
};


#endif
