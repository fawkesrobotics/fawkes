
/***************************************************************************
 *  objpos_average.cpp - Fawkes WorldModel Object Position Average Fuser
 *
 *  Created: Tue Jan 13 13:51:45 2009
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

#include "objpos_average.h"

#include <core/threading/mutex_locker.h>
#include <blackboard/blackboard.h>
#include <interfaces/ObjectPositionInterface.h>

#include <cstring>

using namespace fawkes;

/** @class WorldModelObjPosAverageFuser "objpos_average.h"
 * ObjectPositionModel average fuser.
 * This fuser takes a number of ObjectPositionInterface instanced and fuses them
 * into a single ObjectPositionInterface by averaging over the source interfaces.
 * It registers as an observer and opens any newly created interface that matches
 * the ID pattern.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param blackboard BlackBoard
 * @param from_id_pattern pattern for ID of the interfaces to copy from
 * @param to_id ID of the interface to copy to
 */
WorldModelObjPosAverageFuser::WorldModelObjPosAverageFuser(fawkes::BlackBoard *blackboard,
							   const char *from_id_pattern,
							   const char *to_id)
{
  __blackboard = blackboard;

  __input_ifs.clear();
  __output_if = NULL;
  try {
    __input_ifs = blackboard->open_multiple_for_reading<ObjectPositionInterface>(from_id_pattern);
    __output_if = blackboard->open_for_writing<ObjectPositionInterface>(to_id);
  } catch (Exception &e) {
    for (LockList<ObjectPositionInterface *>::iterator i = __input_ifs.begin(); i != __input_ifs.end(); ++i) {
      blackboard->close(*i);
    }
    blackboard->close(__output_if);
    throw;
  }

  bbio_add_observed_create("ObjectPositionInterface", from_id_pattern);
  blackboard->register_observer(this, BlackBoard::BBIO_FLAG_CREATED);
}


/** Destructor. */
WorldModelObjPosAverageFuser::~WorldModelObjPosAverageFuser()
{
  __blackboard->unregister_observer(this);

  __input_ifs.lock();
  for (__iii == __input_ifs.begin(); __iii != __input_ifs.end(); ++__iii) {
    __blackboard->close(*__iii);
  }
  __input_ifs.clear();
  __input_ifs.unlock();

  __blackboard->close(__output_if);
}


void
WorldModelObjPosAverageFuser::bb_interface_created(const char *type, const char *id) throw()
{
  if (strcmp(id, __output_if->id()) == 0) return;

  ObjectPositionInterface *from_if = NULL;
  
  try {
    from_if = __blackboard->open_for_reading<ObjectPositionInterface>(id);

    __input_ifs.push_back_locked(from_if);
  } catch (Exception &e) {
    __blackboard->close(from_if);
    e.print_trace();
  }
}


void
WorldModelObjPosAverageFuser::fuse()
{
  MutexLocker lock(__input_ifs.mutex());

  unsigned int flags = 0;
  unsigned int num_inputs = 0;
  float roll = 0, pitch = 0, yaw = 0, distance = 0, bearing = 0, slope = 0,
    world_x = 0, world_y = 0, world_z = 0,
    relative_x = 0, relative_y = 0, relative_z = 0,
    extent_x = 0, extent_y = 0, extent_z = 0,
    world_x_velocity = 0, world_y_velocity = 0, world_z_velocity = 0,
    relative_x_velocity = 0, relative_y_velocity = 0, relative_z_velocity = 0;

  for (__iii = __input_ifs.begin(); __iii != __input_ifs.end(); ++__iii) {
    ObjectPositionInterface *iface = *__iii;
    if (iface->has_writer()) {
      iface->read();
      flags               |= iface->flags();
      roll                += iface->roll();
      pitch               += iface->pitch();
      yaw                 += iface->yaw();
      distance            += iface->distance();
      bearing             += iface->bearing();
      slope               += iface->slope();
      world_x             += iface->world_x();
      world_y             += iface->world_y();
      world_z             += iface->world_z();
      relative_x          += iface->relative_x();
      relative_y          += iface->relative_y();
      relative_z          += iface->relative_z();
      extent_x            += iface->extent_x();
      extent_y            += iface->extent_y();
      extent_z            += iface->extent_z();
      world_x_velocity    += iface->world_x_velocity();
      world_y_velocity    += iface->world_y_velocity();
      world_z_velocity    += iface->world_z_velocity();
      relative_x_velocity += iface->relative_x_velocity();
      relative_y_velocity += iface->relative_y_velocity();
      relative_z_velocity += iface->relative_z_velocity();
      ++num_inputs;
    }
  }

  if ( num_inputs == 0 ) {
    roll = pitch = yaw = distance = bearing = slope = 0.;
    world_x = world_y = world_z = 0.;
    relative_x = relative_y = relative_z = 0.;
    extent_x = extent_y = extent_z = 0.;
    world_x_velocity = world_y_velocity = world_z_velocity = 0.;
    relative_x_velocity = relative_y_velocity = relative_z_velocity = 0.;
    num_inputs = 1;
  }

  __output_if->set_roll(roll / (float)num_inputs);
  __output_if->set_pitch (pitch  / (float)num_inputs);
  __output_if->set_yaw(yaw / (float)num_inputs);
  __output_if->set_distance(distance / (float)num_inputs);
  __output_if->set_bearing(bearing / (float)num_inputs);
  __output_if->set_slope(slope / (float)num_inputs);
  __output_if->set_world_x(world_x / (float)num_inputs);
  __output_if->set_world_y(world_y / (float)num_inputs);
  __output_if->set_world_z(world_z / (float)num_inputs);
  __output_if->set_relative_x(relative_x / (float)num_inputs);
  __output_if->set_relative_y(relative_y / (float)num_inputs);
  __output_if->set_relative_z(relative_z / (float)num_inputs);
  __output_if->set_extent_x(extent_x / (float)num_inputs);
  __output_if->set_extent_y(extent_y / (float)num_inputs);
  __output_if->set_extent_z(extent_z / (float)num_inputs);
  __output_if->set_world_x_velocity(world_x_velocity / (float)num_inputs);
  __output_if->set_world_y_velocity(world_y_velocity / (float)num_inputs);
  __output_if->set_world_z_velocity(world_z_velocity / (float)num_inputs);
  __output_if->set_relative_x_velocity(relative_x_velocity / (float)num_inputs);
  __output_if->set_relative_y_velocity(relative_y_velocity / (float)num_inputs);
  __output_if->set_relative_z_velocity(relative_z_velocity / (float)num_inputs);

  __output_if->write();
}
