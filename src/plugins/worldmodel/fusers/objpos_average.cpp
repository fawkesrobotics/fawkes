
/***************************************************************************
 *  objpos_average.cpp - Fawkes WorldModel Object Position Average Fuser
 *
 *  Created: Tue Jan 13 13:51:45 2009
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

#include "objpos_average.h"

#include <core/threading/mutex_locker.h>
#include <blackboard/blackboard.h>
#include <utils/logging/logger.h>
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
 * @param logger logger
 */
WorldModelObjPosAverageFuser::WorldModelObjPosAverageFuser(fawkes::Logger *logger,
							   fawkes::BlackBoard *blackboard,
							   const char *from_id_pattern,
							   const char *to_id)
{
  __logger     = logger;
  __blackboard = blackboard;
  __to_id      = to_id;

  __input_ifs.clear();
  __output_if = NULL;
  try {
    __input_ifs = blackboard->open_multiple_for_reading<ObjectPositionInterface>(from_id_pattern);
    __output_if = blackboard->open_for_writing<ObjectPositionInterface>(to_id);

    // If our output interface was already opened open_multiple might have opened
    // it as well, check and close if that was the case
    for (LockList<ObjectPositionInterface *>::iterator i = __input_ifs.begin(); i != __input_ifs.end(); ++i) {
      if (__to_id == (*i)->id()) {
	blackboard->close(*i);
	__input_ifs.erase(i);
	break;
      }
    }
  } catch (Exception &e) {
    for (LockList<ObjectPositionInterface *>::iterator i = __input_ifs.begin(); i != __input_ifs.end(); ++i) {
      blackboard->close(*i);
    }
    __input_ifs.clear();
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
  for (__iii = __input_ifs.begin(); __iii != __input_ifs.end(); ++__iii) {
    __blackboard->close(*__iii);
  }
  __input_ifs.clear();
  __input_ifs.unlock();

  __blackboard->close(__output_if);
}


void
WorldModelObjPosAverageFuser::bb_interface_created(const char *type, const char *id) throw()
{
  if (__to_id == id) return;

  ObjectPositionInterface *from_if = NULL;
  
  try {
    from_if = __blackboard->open_for_reading<ObjectPositionInterface>(id);

    __input_ifs.push_back_locked(from_if);
  } catch (Exception &e) {
    if (from_if != NULL) {
      __blackboard->close(from_if);
    }
    e.print_trace();
  }
}


void
WorldModelObjPosAverageFuser::fuse()
{
  MutexLocker lock(__input_ifs.mutex());

  unsigned int flags = 0;
  unsigned int base_flags = 0;
  unsigned int world_num_inputs=0, extent_num_inputs=0, euler_num_inputs = 0,
    worldvel_num_inputs = 0, relcart_num_inputs = 0, relpolar_num_inputs = 0;
  float roll = 0, pitch = 0, yaw = 0, distance = 0, bearing = 0, slope = 0,
    world_x = 0, world_y = 0, world_z = 0,
    relative_x = 0, relative_y = 0, relative_z = 0,
    extent_x = 0, extent_y = 0, extent_z = 0,
    world_x_velocity = 0, world_y_velocity = 0, world_z_velocity = 0,
    relative_x_velocity = 0, relative_y_velocity = 0, relative_z_velocity = 0;
  bool valid = true, visible = true;
  int vishistory_min = 0, vishistory_max = 0;
  unsigned int object_type = 0;
  bool object_type_warned = false;
  bool flags_read = false;
  bool have_world = false, have_relative = false;

  for (__iii = __input_ifs.begin(); __iii != __input_ifs.end(); ++__iii) {
    ObjectPositionInterface *iface = *__iii;
    if (iface->has_writer()) {
      iface->read();
      if (iface->is_valid()) {
	if ( (object_type != 0) && (iface->object_type() != object_type) &&
	     ! object_type_warned) {
	  __logger->log_warn("WMObjPosAvgFus", "Object types of input interfaces "
			     "for %s disagree, %s has %u, expected was %u",
			     __to_id.c_str(), iface->uid(), iface->object_type(),
			     object_type);
	  object_type_warned = true;
	} else {
	  object_type = iface->object_type();
	}

	if (flags_read) {
	  unsigned int iflags = iface->flags()
	    & (0xFFFFFFFF^ObjectPositionInterface::FLAG_HAS_WORLD)
	    & (0xFFFFFFFF^ObjectPositionInterface::FLAG_HAS_RELATIVE_CARTESIAN)
	    & (0xFFFFFFFF^ObjectPositionInterface::FLAG_HAS_RELATIVE_POLAR);
	  if (iflags != base_flags) {
	    __logger->log_warn("WMObjPosAvgFus", "Interface flags for %s "
			       "disagree. Exected %x, got %x", base_flags, iflags);
	  }
	} else {
	  base_flags = iface->flags()
	    & (0xFFFFFFFF^ObjectPositionInterface::FLAG_HAS_WORLD)
	    & (0xFFFFFFFF^ObjectPositionInterface::FLAG_HAS_RELATIVE_CARTESIAN)
	    & (0xFFFFFFFF^ObjectPositionInterface::FLAG_HAS_RELATIVE_POLAR);
	  flags_read = true;
	}

	if (iface->is_visible()) {
	  if (iface->flags() & ObjectPositionInterface::FLAG_HAS_WORLD) {
	    have_world = true;

	    flags |= ObjectPositionInterface::FLAG_HAS_WORLD;
	    world_x             += iface->world_x();
	    world_y             += iface->world_y();
	    world_z             += iface->world_z();
	    world_num_inputs    += 1;

	    if (iface->flags() & ObjectPositionInterface::FLAG_HAS_EULER_ANGLES) {
	      roll              += iface->roll();
	      pitch             += iface->pitch();
	      yaw               += iface->yaw();
	      flags             |= ObjectPositionInterface::FLAG_HAS_EULER_ANGLES;
	      euler_num_inputs  += 1;
	    }

	    if (iface->flags() & ObjectPositionInterface::FLAG_HAS_WORLD_VELOCITY) {
	      world_x_velocity    += iface->world_x_velocity();
	      world_y_velocity    += iface->world_y_velocity();
	      world_z_velocity    += iface->world_z_velocity();
	      flags               |= ObjectPositionInterface::FLAG_HAS_WORLD_VELOCITY;
	      worldvel_num_inputs += 1;
	    }
	  }

	  if (iface->flags() & ObjectPositionInterface::FLAG_HAS_RELATIVE_CARTESIAN) {
	    have_relative = true;

	    flags |= ObjectPositionInterface::FLAG_HAS_RELATIVE_CARTESIAN;
	    
	    relative_x          += iface->relative_x();
	    relative_y          += iface->relative_y();
	    relative_z          += iface->relative_z();
	    relative_x_velocity += iface->relative_x_velocity();
	    relative_y_velocity += iface->relative_y_velocity();
	    relative_z_velocity += iface->relative_z_velocity();
	    relcart_num_inputs  += 1;
	  }

	  if (iface->flags() & ObjectPositionInterface::FLAG_HAS_RELATIVE_POLAR) {
	    have_relative = true;
	    
	    flags |= ObjectPositionInterface::FLAG_HAS_RELATIVE_POLAR;
	    
	    distance            += iface->distance();
	    bearing             += iface->bearing();
	    slope               += iface->slope();
	    relpolar_num_inputs += 1;
	  }

	  if (iface->flags() & ObjectPositionInterface::FLAG_HAS_EXTENT) {
	    extent_x          += iface->extent_x();
	    extent_y          += iface->extent_y();
	    extent_z          += iface->extent_z();
	    flags               |= ObjectPositionInterface::FLAG_HAS_EXTENT;
	    extent_num_inputs += 1;
	  }

	  if (iface->visibility_history() > vishistory_max) {
	    vishistory_max = iface->visibility_history();
	  }
	} else {
	  if (iface->visibility_history() < vishistory_min) {
	    vishistory_min = iface->visibility_history();
	  }
	}
      }
    }
  }

  if ( world_num_inputs > 0 ) {
    __output_if->set_world_x(world_x / (float)world_num_inputs);
    __output_if->set_world_y(world_y / (float)world_num_inputs);
    __output_if->set_world_z(world_z / (float)world_num_inputs);
  }
  if ( euler_num_inputs > 0 ) {
    __output_if->set_roll(roll / (float)euler_num_inputs);
    __output_if->set_pitch(pitch  / (float)euler_num_inputs);
    __output_if->set_yaw(yaw / (float)euler_num_inputs);
  }
  if ( worldvel_num_inputs > 0) {
    __output_if->set_world_x_velocity(world_x_velocity / (float)worldvel_num_inputs);
    __output_if->set_world_y_velocity(world_y_velocity / (float)worldvel_num_inputs);
    __output_if->set_world_z_velocity(world_z_velocity / (float)worldvel_num_inputs);
  }

  if ( extent_num_inputs > 0 ) {
    __output_if->set_extent_x(extent_x / (float)extent_num_inputs);
    __output_if->set_extent_y(extent_y / (float)extent_num_inputs);
    __output_if->set_extent_z(extent_z / (float)extent_num_inputs);
  }
  if ( relcart_num_inputs > 0) {
  __output_if->set_relative_x(relative_x / (float)relcart_num_inputs);
  __output_if->set_relative_y(relative_y / (float)relcart_num_inputs);
  __output_if->set_relative_z(relative_z / (float)relcart_num_inputs);
  __output_if->set_relative_x_velocity(relative_x_velocity / (float)relcart_num_inputs);
  __output_if->set_relative_y_velocity(relative_y_velocity / (float)relcart_num_inputs);
  __output_if->set_relative_z_velocity(relative_z_velocity / (float)relcart_num_inputs);
  }
  if ( relpolar_num_inputs > 0) {
    __output_if->set_distance(distance / (float)relpolar_num_inputs);
    __output_if->set_bearing(bearing / (float)relpolar_num_inputs);
    __output_if->set_slope(slope / (float)relpolar_num_inputs);
  }

  visible = have_world || have_relative;

  __output_if->set_flags(flags);
  __output_if->set_valid(valid);
  __output_if->set_visible(visible);
  __output_if->set_visibility_history(visible ? vishistory_max : vishistory_min);

  __output_if->write();
}
