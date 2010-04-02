
/***************************************************************************
 *  objpos_majority.cpp - Fawkes WorldModel Object Position Majority Fuser
 *
 *  Created: Thu 01 Apr 2010 05:06:36 PM CEST
 *  Copyright  2010  Christoph Schwering
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

#include "objpos_majority.h"

#include <cmath>
#include <cstring>

#include <core/threading/mutex_locker.h>
#include <blackboard/blackboard.h>
#include <utils/logging/logger.h>
#include <interfaces/ObjectPositionInterface.h>

/** @class WorldModelObjPosMajorityFuser "objpos_majority.h"
 * ObjectPositionInterface majority fuser.
 * The parameters are (1) the ID of the own ObjectPositionInterface, (2) the
 * pattern ID of the other robots' ObjectPositionInterfaces and (3) the
 * maximum-self-confidence-distance.
 *
 * Like the WorldModelObjPosMajorityFuser, it registers as an observer and opens
 * any newly created interface that matches the ID of the own
 * ObjectPositionInterface or the pattern of the foreign
 * ObjectPositionInterfaces.
 * @author Christoph Schwering
 */

/** Constructor.
 * @param blackboard BlackBoard.
 * @param logger Logger.
 * @param own_id The ID of the (single) own interface.
 * @param foreign_id_pattern The pattern of the (multiple) other interfaces.
 * @param output_id The ID of the destination interface.
 */
WorldModelObjPosMajorityFuser::WorldModelObjPosMajorityFuser(
    fawkes::Logger* logger,
    fawkes::BlackBoard* blackboard,
    const std::string& own_id,
    const std::string& foreign_id_pattern,
    const std::string& output_id,
    float self_confidence_radius)
    : blackboard_(blackboard),
      logger_(logger),
      own_id_(own_id),
      output_id_(output_id),
      self_confidence_radius_(self_confidence_radius)
{
  input_ifs_.clear();
  output_if_ = NULL;
  try {
    own_if_ = blackboard_->open_for_reading<OPI>(own_id.c_str());
    input_ifs_ = blackboard_->open_multiple_for_reading<OPI>(
        foreign_id_pattern.c_str());
    output_if_ = blackboard_->open_for_writing<OPI>(output_id.c_str());
    if (own_if_ != NULL) {
      input_ifs_.push_back(own_if_);
    }

    // If our output interface was already opened open_multiple might have opened
    // it as well, check and close if that was the case
    for (OPIList::iterator it = input_ifs_.begin();
         it != input_ifs_.end(); ++it) {
      if (output_id == (*it)->id()) {
	blackboard->close(*it);
	input_ifs_.erase(it);
	break;
      }
    }
  } catch (fawkes::Exception& e) {
    for (OPIList::const_iterator it = input_ifs_.begin();
         it != input_ifs_.end(); ++it) {
      blackboard->close(*it);
    }
    input_ifs_.clear();
    blackboard->close(output_if_);
    throw;
  }

  bbio_add_observed_create("ObjectPositionInterface", own_id.c_str());
  bbio_add_observed_create("ObjectPositionInterface", foreign_id_pattern.c_str());
  blackboard_->register_observer(this, fawkes::BlackBoard::BBIO_FLAG_CREATED);
}


/** Destructor. */
WorldModelObjPosMajorityFuser::~WorldModelObjPosMajorityFuser()
{
  blackboard_->unregister_observer(this);

  input_ifs_.lock();
  for (OPIList::const_iterator it = input_ifs_.begin();
       it != input_ifs_.end(); ++it) {
    blackboard_->close(*it);
  }
  input_ifs_.clear();
  input_ifs_.unlock();

  blackboard_->close(output_if_);
}


void
WorldModelObjPosMajorityFuser::bb_interface_created(const char *type,
                                                    const char *id) throw()
{
  if (output_id_ == id) {
    return;
  }
  OPI* from_if = NULL;
  try {
    from_if = blackboard_->open_for_reading<OPI>(id);
    if (own_if_ == NULL && own_id_ == id) {
      own_if_ = own_if_;
    }
    input_ifs_.push_back_locked(from_if);
  } catch (fawkes::Exception& e) {
    if (from_if != NULL) {
      blackboard_->close(from_if);
    }
    e.print_trace();
  }
}

float
WorldModelObjPosMajorityFuser::length(float x, float y, float z)
{
  return sqrt(x*x + y*y + z*z);
}

float
WorldModelObjPosMajorityFuser::rel_length(const OPI* iface)
{
  return length(iface->relative_x(), iface->relative_y(), iface->relative_z());
}

float
WorldModelObjPosMajorityFuser::world_dist(const OPI* from, const OPI* to)
{
  return length(to->world_x() - from->world_x(),
                to->world_y() - from->world_y(),
                to->world_z() - from->world_z());
}

void
WorldModelObjPosMajorityFuser::fuse()
{
  if (own_if_ != NULL) {
    own_if_->read();
  }

  if (own_if_ != NULL &&
      ((own_if_->flags() & OPI::FLAG_HAS_RELATIVE_CARTESIAN &&
        rel_length(own_if_) <= self_confidence_radius_) ||
       (own_if_->flags() & OPI::FLAG_HAS_RELATIVE_POLAR &&
        own_if_->distance() <= self_confidence_radius_))) {
    // Case 1: just copy own data if the own data claim the ball is very near
    copy_own_if();

  } else {
    // Case 2: group interfaces, look for a majority and average it, if there is
    // none, copy the own interface.

    for (OPIList::const_iterator it = input_ifs_.begin();
         it != input_ifs_.end(); ++it) {
      OPI* iface = *it;
      if (iface != own_if_) { // we've read own_if_ already
        iface->read();
      }
    }
    check();

    OPIBuckets buckets;
    for (OPIList::const_iterator it = input_ifs_.begin();
         it != input_ifs_.end(); ++it) {
      OPI* iface = *it;
      OPIBucket bucket;
      bucket[0] = iface;
      for (OPIList::const_iterator jt = input_ifs_.begin();
           jt != input_ifs_.end(); ++jt) {
        OPI* candidate = *jt;
        if (candidate != iface &&
            world_dist(iface, candidate) <= GROUP_RADIUS) {
          bucket.push_back(candidate);
        }
      }
      buckets[buckets.size()] = bucket;
    }

    OPIBucket majority;
    bool unambiguous = false;
    for (OPIBuckets::const_iterator it = buckets.begin();
         it != buckets.end(); ++it) {
      const OPIBucket& bucket = *it;
      if (majority.size() <= bucket.size()) {
        majority = bucket;
        unambiguous = (majority.size() < bucket.size());
      }
    }
    if (unambiguous) {
      // Case 2a: calculate average of majority.
      average(majority);
    } else {
      // Case 2b: no majority found, copy own data
      copy_own_if();
    }
  }
}

void
WorldModelObjPosMajorityFuser::check()
{
  unsigned int base_flags = 0;

  unsigned int object_type = 0;
  bool object_type_warned = false;
  bool flags_read = false;

  for (OPIList::const_iterator it = input_ifs_.begin();
       it != input_ifs_.end(); ++it) {
    OPI* iface = *it;
    if (!iface->has_writer()) {
      continue;
    }
    if (!iface->is_valid()) {
      continue;
    }
    if (object_type != 0 && iface->object_type() != object_type &&
        !object_type_warned) {
      logger_->log_warn("WMObjPosAvgFus", "Object types of input interfaces "
                        "for %s disagree, %s has %u, expected was %u",
                        output_id_.c_str(), iface->uid(), iface->object_type(),
                        object_type);
      object_type_warned = true;
    } else {
      object_type = iface->object_type();
    }

    if (flags_read) {
      unsigned int iflags = iface->flags()
        & (0xFFFFFFFF ^ OPI::FLAG_HAS_WORLD)
        & (0xFFFFFFFF ^ OPI::FLAG_HAS_RELATIVE_CARTESIAN)
        & (0xFFFFFFFF ^ OPI::FLAG_HAS_RELATIVE_POLAR);
      if (iflags != base_flags) {
        logger_->log_warn("WMObjPosAvgFus", "Interface flags for %s "
                          "disagree. Exected %x, got %x", base_flags,
                          iflags);
      }
    } else {
      base_flags = iface->flags()
        & (0xFFFFFFFF ^ OPI::FLAG_HAS_WORLD)
        & (0xFFFFFFFF ^ OPI::FLAG_HAS_RELATIVE_CARTESIAN)
        & (0xFFFFFFFF ^ OPI::FLAG_HAS_RELATIVE_POLAR);
      flags_read = true;
    }
  }
}

void
WorldModelObjPosMajorityFuser::copy_own_if()
{
  own_if_->copy_values(output_if_);
  own_if_->write();

#if 0
  output_if_->set_flags(own_if_->flags());
  output_if_->set_valid(own_if_->valid());
  output_if_->set_visible(own_if_->visible());
  output_if_->set_visibility_history(own_if_->visibility_history());

  if (output_if_->visible()) {
    if (own_if_->flags() & OPI::FLAG_HAS_WORLD) {
      output_if_->set_world_x(own_if_->world_x());
      output_if_->set_world_y(own_if_->world_y());
      output_if_->set_world_z(own_if_->world_z());

      if (own_if_->flags() & OPI::FLAG_HAS_EULER_ANGLES) {
        output_if_->set_roll(own_if_->roll());
        output_if_->set_pitch(own_if_->pitch());
        output_if_->set_yaw(own_if_->yaw());
      }

      if (own_if_->flags() & OPI::FLAG_HAS_WORLD_VELOCITY) {
        output_if_->set_world_x_velocity(own_if_->world_x_velocity());
        output_if_->set_world_y_velocity(own_if_->world_y_velocity());
        output_if_->set_world_z_velocity(own_if_->world_z_velocity());
      }
    }

    if (own_if_->flags() & OPI::FLAG_HAS_RELATIVE_CARTESIAN) {
      output_if_->set_relative_x(own_if_->relative_x());
      output_if_->set_relative_y(own_if_->relative_y());
      output_if_->set_relative_z(own_if_->relative_z());
      output_if_->set_relative_x_velocity(own_if_->relative_x_velocity());
      output_if_->set_relative_y_velocity(own_if_->relative_y_velocity());
      output_if_->set_relative_z_velocity(own_if_->relative_z_velocity());
    }

    if (own_if_->flags() & OPI::FLAG_HAS_RELATIVE_POLAR) {
      output_if_->set_distance(own_if_->distance());
      output_if_->set_bearing(own_if_->bearing());
      output_if_->set_slope(own_if_->slope());
    }

    if (own_if_->flags() & OPI::FLAG_HAS_EXTENT) {
      output_if_->set_extent_x(own_if_->extent_x());
      output_if_->set_extent_y(own_if_->extent_y());
      output_if_->set_extent_z(own_if_->extent_z());
    }
  }

  output_if_->write()
#endif
}

/** Averages over the given input interfaces.
 * The same like WorldModelObjPosAverageFuser::fuse() except that this method
 * works on a parameter.
 * (Making this fuser a subclass of the average fuser would make sense.)
 * @param input_ifs The interface that are averaged.
 */
void
WorldModelObjPosMajorityFuser::average(const OPIBucket& input_ifs)
{
  unsigned int flags = 0;
  unsigned int world_num_inputs = 0;
  unsigned int extent_num_inputs = 0;
  unsigned int euler_num_inputs = 0;
  unsigned int worldvel_num_inputs = 0;
  unsigned int relcart_num_inputs = 0;
  unsigned int relpolar_num_inputs = 0;

  float roll = 0.0f;
  float pitch = 0.0f;
  float yaw = 0.0f;
  float distance = 0.0f;
  float bearing = 0.0f;
  float slope = 0.0f;

  float world_x = 0.0f;
  float world_y = 0.0f;
  float world_z = 0.0f;

  float relative_x = 0.0f;
  float relative_y = 0.0f;
  float relative_z = 0.0f;

  float extent_x = 0.0f;
  float extent_y = 0.0f;
  float extent_z = 0.0f;

  float world_x_velocity = 0.0f;
  float world_y_velocity = 0.0f;
  float world_z_velocity = 0.0f;

  float relative_x_velocity = 0.0f;
  float relative_y_velocity = 0.0f;
  float relative_z_velocity = 0.0f;

  bool valid = true;
  bool visible = true;
  int vishistory_min = 0;
  int vishistory_max = 0;
  bool have_world = false, have_relative = false;

  for (OPIBucket::const_iterator it = input_ifs.begin();
       it != input_ifs.end(); ++it) {
    OPI* iface = *it;
    if (!iface->has_writer()) {
      continue;
    }
    iface->read();
    if (!iface->is_valid()) {
      continue;
    }

    if (iface->is_visible()) {
      if (iface->flags() & OPI::FLAG_HAS_WORLD) {
        have_world = true;

        flags |= OPI::FLAG_HAS_WORLD;
        world_x             += iface->world_x();
        world_y             += iface->world_y();
        world_z             += iface->world_z();
        world_num_inputs    += 1;

        if (iface->flags() & OPI::FLAG_HAS_EULER_ANGLES) {
          roll              += iface->roll();
          pitch             += iface->pitch();
          yaw               += iface->yaw();
          flags             |= OPI::FLAG_HAS_EULER_ANGLES;
          euler_num_inputs  += 1;
        }

        if (iface->flags() & OPI::FLAG_HAS_WORLD_VELOCITY) {
          world_x_velocity    += iface->world_x_velocity();
          world_y_velocity    += iface->world_y_velocity();
          world_z_velocity    += iface->world_z_velocity();
          flags               |= OPI::FLAG_HAS_WORLD_VELOCITY;
          worldvel_num_inputs += 1;
        }
      }

      if (iface->flags() & OPI::FLAG_HAS_RELATIVE_CARTESIAN) {
        have_relative = true;

        flags |= OPI::FLAG_HAS_RELATIVE_CARTESIAN;
        
        relative_x          += iface->relative_x();
        relative_y          += iface->relative_y();
        relative_z          += iface->relative_z();
        relative_x_velocity += iface->relative_x_velocity();
        relative_y_velocity += iface->relative_y_velocity();
        relative_z_velocity += iface->relative_z_velocity();
        relcart_num_inputs  += 1;
      }

      if (iface->flags() & OPI::FLAG_HAS_RELATIVE_POLAR) {
        have_relative = true;
        
        flags |= OPI::FLAG_HAS_RELATIVE_POLAR;
        
        distance            += iface->distance();
        bearing             += iface->bearing();
        slope               += iface->slope();
        relpolar_num_inputs += 1;
      }

      if (iface->flags() & OPI::FLAG_HAS_EXTENT) {
        extent_x          += iface->extent_x();
        extent_y          += iface->extent_y();
        extent_z          += iface->extent_z();
        flags             |= OPI::FLAG_HAS_EXTENT;
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

  if (world_num_inputs > 0) {
    output_if_->set_world_x(world_x / world_num_inputs);
    output_if_->set_world_y(world_y / world_num_inputs);
    output_if_->set_world_z(world_z / world_num_inputs);
  }
  if (euler_num_inputs > 0) {
    output_if_->set_roll(roll / euler_num_inputs);
    output_if_->set_pitch(pitch  / euler_num_inputs);
    output_if_->set_yaw(yaw / euler_num_inputs);
  }
  if (worldvel_num_inputs > 0) {
    output_if_->set_world_x_velocity(world_x_velocity / worldvel_num_inputs);
    output_if_->set_world_y_velocity(world_y_velocity / worldvel_num_inputs);
    output_if_->set_world_z_velocity(world_z_velocity / worldvel_num_inputs);
  }

  if (extent_num_inputs > 0) {
    output_if_->set_extent_x(extent_x / extent_num_inputs);
    output_if_->set_extent_y(extent_y / extent_num_inputs);
    output_if_->set_extent_z(extent_z / extent_num_inputs);
  }
  if (relcart_num_inputs > 0) {
    output_if_->set_relative_x(relative_x / relcart_num_inputs);
    output_if_->set_relative_y(relative_y / relcart_num_inputs);
    output_if_->set_relative_z(relative_z / relcart_num_inputs);
    output_if_->set_relative_x_velocity(relative_x_velocity / relcart_num_inputs);
    output_if_->set_relative_y_velocity(relative_y_velocity / relcart_num_inputs);
    output_if_->set_relative_z_velocity(relative_z_velocity / relcart_num_inputs);
  }
  if (relpolar_num_inputs > 0) {
    output_if_->set_distance(distance / (float)relpolar_num_inputs);
    output_if_->set_bearing(bearing / (float)relpolar_num_inputs);
    output_if_->set_slope(slope / (float)relpolar_num_inputs);
  }

  visible = have_world || have_relative;

  output_if_->set_flags(flags);
  output_if_->set_valid(valid);
  output_if_->set_visible(visible);
  output_if_->set_visibility_history(visible ? vishistory_max : vishistory_min);

  output_if_->write();
}

