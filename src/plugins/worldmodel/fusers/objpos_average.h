
/***************************************************************************
 *  objpos_average.h - Fawkes WorldModel Object Position Average Fuser
 *
 *  Created: Tue Jan 13 13:48:49 2009
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

#ifndef __PLUGINS_WORLDMODEL_FUSER_OBJPOS_AVERAGE_H_
#define __PLUGINS_WORLDMODEL_FUSER_OBJPOS_AVERAGE_H_

#include "fuser.h"

#include <blackboard/interface_observer.h>
#include <core/utils/lock_list.h>

namespace fawkes
{
  class BlackBoard;
  class ObjectPositionInterface;
}

class WorldModelObjPosAverageFuser
: public WorldModelFuser,
  public fawkes::BlackBoardInterfaceObserver
{
 public:
  WorldModelObjPosAverageFuser(fawkes::BlackBoard *blackboard,
			       const char *from_id_pattern, const char *to_id);
  ~WorldModelObjPosAverageFuser();


  virtual void fuse();

  virtual void bb_interface_created(const char *type, const char *id) throw();

 private:
  fawkes::BlackBoard *__blackboard;

  fawkes::LockList<fawkes::ObjectPositionInterface *>  __input_ifs;
  fawkes::ObjectPositionInterface                     *__output_if;

  fawkes::LockList<fawkes::ObjectPositionInterface *>::iterator  __iii;
};


#endif
