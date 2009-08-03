
/***************************************************************************
 *  multi_copy.h - Fawkes WorldModel Multi Interface Copy Fuser
 *
 *  Created: Tue Jan 13 11:49:20 2009
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

#ifndef __PLUGINS_WORLDMODEL_FUSER_MULTI_COPY_H_
#define __PLUGINS_WORLDMODEL_FUSER_MULTI_COPY_H_

#include "fuser.h"

#include <blackboard/interface_observer.h>
#include <core/utils/lock_map.h>
#include <string>

namespace fawkes
{
  class BlackBoard;
  class Interface;
}

class WorldModelMultiCopyFuser
: public WorldModelFuser,
  public fawkes::BlackBoardInterfaceObserver
{
 public:
  WorldModelMultiCopyFuser(fawkes::BlackBoard *blackboard, const char *type,
			   const char *from_id_pattern, const char *to_id_format);
  ~WorldModelMultiCopyFuser();


  virtual void fuse();

  virtual void bb_interface_created(const char *type, const char *id) throw();

 private:
  fawkes::BlackBoard *__blackboard;

  std::string __from_id_pattern;
  std::string __to_id_format;

  fawkes::LockMap<fawkes::Interface *, fawkes::Interface *> __ifmap;
  fawkes::LockMap<fawkes::Interface *, fawkes::Interface *>::iterator __imi;
};


#endif
