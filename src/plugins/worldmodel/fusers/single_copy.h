
/***************************************************************************
 *  single_copy.h - Fawkes WorldModel Single Interface Copy Fuser
 *
 *  Created: Tue Jan 13 11:22:21 2009
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_WORLDMODEL_FUSER_SINGLE_COPY_H_
#define __PLUGINS_WORLDMODEL_FUSER_SINGLE_COPY_H_

#include "fuser.h"

namespace fawkes {
  class BlackBoard;
  class Interface;
}

class WorldModelSingleCopyFuser : public WorldModelFuser
{
 public:
  WorldModelSingleCopyFuser(fawkes::BlackBoard *blackboard, const char *type,
			    const char *from_id, const char *to_id);
  ~WorldModelSingleCopyFuser();

  virtual void fuse();

 private:
  fawkes::BlackBoard *__blackboard;
  fawkes::Interface  *__from;
  fawkes::Interface  *__to;
};


#endif
