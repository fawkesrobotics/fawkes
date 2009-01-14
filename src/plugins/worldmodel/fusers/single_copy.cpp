
/***************************************************************************
 *  single_copy.cpp - Fawkes WorldModel Single Interface Copy Fuser
 *
 *  Created: Tue Jan 13 11:46:30 2009
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

#include "single_copy.h"

#include <blackboard/blackboard.h>
#include <interface/interface.h>

using namespace fawkes;

/** @class WorldModelSingleCopyFuser "single_copy.h"
 * Single interface copy fuser.
 * This fuser simply copies the data of one interface to another of the same
 * type.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param blackboard BlackBoard
 * @param type interface type of both interfaces
 * @param from_id ID of the interface to copy from
 * @param to_id ID of the interface to copy to
 */
WorldModelSingleCopyFuser::WorldModelSingleCopyFuser(BlackBoard *blackboard,
						     const char *type,
						     const char *from_id,
						     const char *to_id)
{
  __blackboard = blackboard;
  __from       = blackboard->open_for_reading(type, from_id);
  __to         = blackboard->open_for_writing(type, to_id);

  __from->read();
  __to->copy_values(__from);
  __to->write();
}


/** Destructor. */
WorldModelSingleCopyFuser::~WorldModelSingleCopyFuser()
{
  __blackboard->close(__from);
  __blackboard->close(__to);
}


void
WorldModelSingleCopyFuser::fuse()
{
  if (__from->has_writer()) {
    __from->read();
    __to->copy_values(__from);
    __to->write();
  }
}
