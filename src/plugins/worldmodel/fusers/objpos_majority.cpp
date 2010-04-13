
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

#include <iostream>
#include <cmath>
#include <cstring>
#include <list>

#include <core/threading/mutex_locker.h>
#include <blackboard/blackboard.h>
#include <utils/logging/logger.h>

/** @class WorldModelObjPosMajorityFuser "objpos_majority.h"
 * ObjectPositionInterface majority fuser.
 * The parameters are (1) the ID of the own ObjectPositionInterface, (2) the
 * pattern ID of the other robots' ObjectPositionInterfaces and (3) the
 * maximum-self-confidence-distance.
 *
 * (1) If the own ObjectPositionInterface thinks the object is not further away
 * than self_confidence_radius, then the own interface's data is copied to the
 * output interface.
 * (2) If there is an unambiguous majority of interfaces that say the object is
 * somewhere else and this majority is averaged and the average values are
 * copied to the output interface.
 * Since the other interfaces probably won't agree on one exact position, they
 * are grouped: for each interface A its group is the set of interfaces that
 * claim the object is not further away from the position claimed by A than
 * GROUP_RADIUS. GROUP_RADIUS is currently hard-coded to 1.0 meters.
 * (3) If the other interfaces "cannot settle" on some position of the object,
 * the own interface's data is considered as at least as reliable as theirs and
 * therefore the own interface's data is copied to the output interface.
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
 * @param self_confidence_radius radius in which to consider our perception
 * the best
 */
WorldModelObjPosMajorityFuser::WorldModelObjPosMajorityFuser(
    fawkes::Logger* logger,
    fawkes::BlackBoard* blackboard,
    const std::string& own_id,
    const std::string& foreign_id_pattern,
    const std::string& output_id,
    float self_confidence_radius)
    : logger_(logger),
      blackboard_(blackboard),
      own_id_(own_id),
      output_id_(output_id),
      self_confidence_radius_(self_confidence_radius)
{
  input_ifs_.clear();
  output_if_ = NULL;
  try {
    own_if_ = blackboard_->open_for_reading<Opi>(own_id.c_str());
    std::list<Opi*> input_ifs = blackboard_->open_multiple_for_reading<Opi>(
        foreign_id_pattern.c_str());
    for (std::list<Opi*>::const_iterator it = input_ifs.begin();
         it != input_ifs.end(); ++it) {
      Opi* opi = *it;
      std::pair<OpiSet::iterator,bool> ret = input_ifs_.insert(opi);
      if (!ret.second) {
        blackboard->close(opi);
      }
    }
    if (own_if_ != NULL) {
      std::pair<OpiSet::iterator,bool> ret = input_ifs_.insert(own_if_);
      if (!ret.second) {
        blackboard->close(own_if_);
        own_if_ = *ret.first;
      }
    }

    output_if_ = blackboard_->open_for_writing<Opi>(output_id.c_str());
    OpiSet::iterator iter = input_ifs_.find(output_if_);
    if (iter != input_ifs_.end()) {
      Opi* opi = *iter;
      blackboard->close(opi);
      input_ifs_.erase(iter);
    }
  } catch (fawkes::Exception& e) {
    for (OpiSet::const_iterator it = input_ifs_.begin();
         it != input_ifs_.end(); ++it) {
      blackboard->close(*it);
    }
    input_ifs_.clear();
    if (output_if_ != NULL) {
      blackboard->close(output_if_);
      output_if_ = NULL;
    }
    throw;
  }

  bbio_add_observed_create("ObjectPositionInterface",
                           own_id.c_str());
  bbio_add_observed_create("ObjectPositionInterface",
                           foreign_id_pattern.c_str());
  blackboard_->register_observer(this, fawkes::BlackBoard::BBIO_FLAG_CREATED);
}


/** Destructor. */
WorldModelObjPosMajorityFuser::~WorldModelObjPosMajorityFuser()
{
  blackboard_->unregister_observer(this);

  input_ifs_.lock();
  for (OpiSet::const_iterator it = input_ifs_.begin();
       it != input_ifs_.end(); ++it) {
    blackboard_->close(*it);
  }
  input_ifs_.clear();
  input_ifs_.unlock();

  if (output_if_ != NULL) {
    blackboard_->close(output_if_);
  }
}


void
WorldModelObjPosMajorityFuser::bb_interface_created(const char *type,
                                                    const char *id) throw()
{
  if (output_id_ == id) {
    return;
  }
  Opi* from_if = NULL;
  try {
    from_if = blackboard_->open_for_reading<Opi>(id);
    std::pair<OpiSet::iterator,bool> ret = input_ifs_.insert_locked(from_if);
    if (!ret.second) {
      blackboard_->close(from_if);
    }
    Opi* inserted_if = *ret.first;
    if (own_if_ == NULL && own_id_ == std::string(inserted_if->id())) {
      own_if_ = inserted_if;
    }
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
WorldModelObjPosMajorityFuser::rel_length(const Opi* opi)
{
  return length(opi->relative_x(), opi->relative_y(), opi->relative_z());
}

float
WorldModelObjPosMajorityFuser::world_object_dist(const Opi* from, const Opi* to)
{
  return length(to->world_x() - from->world_x(),
                to->world_y() - from->world_y(),
                to->world_z() - from->world_z());
}

bool
WorldModelObjPosMajorityFuser::same_contents(const OpiBucket& left,
                                             const OpiBucket& right)
{
  if (left.size() != right.size()) {
    return false;
  }

  std::set<OpiWrapper> rightSet(right.begin(), right.end());
  for (OpiBucket::const_iterator it = left.begin();
       it != left.end(); ++it) {
    Opi* opi = *it;
    if (rightSet.find(opi) == rightSet.end()) {
      return false;
    }
  }
  return true;
}

void
WorldModelObjPosMajorityFuser::fuse()
{
  if (own_if_ != NULL) {
    own_if_->read();
  }

  if (own_if_ != NULL &&
      own_if_->has_writer() &&
      own_if_->is_valid() &&
      own_if_->is_visible() &&
      (((own_if_->flags() & Opi::FLAG_HAS_RELATIVE_CARTESIAN) &&
        rel_length(own_if_) <= self_confidence_radius_) ||
       ((own_if_->flags() & Opi::FLAG_HAS_RELATIVE_POLAR) &&
         own_if_->distance() <= self_confidence_radius_))) {
    // Case 1: just copy own data if the own data claims the ball is very near.
    copy_own_if();

  } else {
    // Case 2: group interfaces, look for a majority and average it, if there is
    // none, copy the own interface.

    for (OpiSet::const_iterator it = input_ifs_.begin();
         it != input_ifs_.end(); ++it) {
      Opi* opi = *it;
      if (opi != own_if_) { // we've read own_if_ already
        opi->read();
      }
    }
    check();

    // Group interfaces in buckets.
    input_ifs_.lock();
    OpiBuckets buckets;
    for (OpiSet::const_iterator it = input_ifs_.begin();
         it != input_ifs_.end(); ++it) {
      Opi* opi = *it;
      if (!(opi->flags() & Opi::FLAG_HAS_WORLD) ||
          !opi->has_writer() ||
          !opi->is_valid() ||
          !opi->is_visible()) {
        continue;
      }
      OpiBucket bucket;
      bucket.push_back(opi);
      for (OpiSet::const_iterator jt = input_ifs_.begin();
           jt != input_ifs_.end(); ++jt) {
        Opi* candidate = *jt;
        if (candidate != opi &&
            (candidate->flags() & Opi::FLAG_HAS_WORLD) &&
            world_object_dist(opi, candidate) <= GROUP_RADIUS) {
          bucket.push_back(candidate);
        }
      }
      buckets.push_back(bucket);
    }
    input_ifs_.unlock();

    // Search for majority.
    OpiBucket majority;
    bool unambiguous = false;
    for (OpiBuckets::const_iterator it = buckets.begin();
         it != buckets.end(); ++it) {
      const OpiBucket& bucket = *it;
      if (majority.size() <= bucket.size()) {
        if (majority.size() < bucket.size()) {
          majority = bucket;
          unambiguous = true;
        } else {
          unambiguous = unambiguous && same_contents(majority, bucket);
        }
      }
    }
    if (majority.size() > 0 && unambiguous) {
      // Case 2a: calculate average of majority.
      average(majority);
    } else {
      // Case 2b: no majority found, copy own data.
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

  for (OpiSet::const_iterator it = input_ifs_.begin();
       it != input_ifs_.end(); ++it) {
    Opi* opi = *it;
    if (!opi->has_writer()) {
      continue;
    }
    if (!opi->is_valid()) {
      continue;
    }
    if (object_type != 0 && opi->object_type() != object_type &&
        !object_type_warned) {
      logger_->log_warn("WMObjPosAvgFus", "Object types of input interfaces "
                        "for %s disagree, %s has %u, expected was %u",
                        output_id_.c_str(), opi->uid(), opi->object_type(),
                        object_type);
      object_type_warned = true;
    } else {
      object_type = opi->object_type();
    }

    if (flags_read) {
      unsigned int iflags = opi->flags()
        & (0xFFFFFFFF ^ Opi::FLAG_HAS_WORLD)
        & (0xFFFFFFFF ^ Opi::FLAG_HAS_RELATIVE_CARTESIAN)
        & (0xFFFFFFFF ^ Opi::FLAG_HAS_RELATIVE_POLAR);
      if (iflags != base_flags) {
        logger_->log_warn("WMObjPosAvgFus", "Interface flags for %s "
                          "disagree. Exected %x, got %x", base_flags,
                          iflags);
      }
    } else {
      base_flags = opi->flags()
        & (0xFFFFFFFF ^ Opi::FLAG_HAS_WORLD)
        & (0xFFFFFFFF ^ Opi::FLAG_HAS_RELATIVE_CARTESIAN)
        & (0xFFFFFFFF ^ Opi::FLAG_HAS_RELATIVE_POLAR);
      flags_read = true;
    }
  }
}

void
WorldModelObjPosMajorityFuser::copy_own_if()
{
  output_if_->copy_values(own_if_);
  output_if_->write();
}

/** Averages over the given input interfaces.
 * The same like WorldModelObjPosAverageFuser::fuse() except that this method
 * works on a parameter.
 * (Making this fuser a subclass of the average fuser would make sense.)
 * @param input_ifs The interface that are averaged.
 */
void
WorldModelObjPosMajorityFuser::average(const OpiBucket& input_ifs)
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

  for (OpiBucket::const_iterator it = input_ifs.begin();
       it != input_ifs.end(); ++it) {
    Opi* opi = *it;
    if (!opi->has_writer()) {
      continue;
    }
    opi->read();
    if (!opi->is_valid()) {
      continue;
    }

    if (opi->is_visible()) {
      if (opi->flags() & Opi::FLAG_HAS_WORLD) {
        have_world = true;

        flags |= Opi::FLAG_HAS_WORLD;
        world_x             += opi->world_x();
        world_y             += opi->world_y();
        world_z             += opi->world_z();
        world_num_inputs    += 1;

        if (opi->flags() & Opi::FLAG_HAS_EULER_ANGLES) {
          roll              += opi->roll();
          pitch             += opi->pitch();
          yaw               += opi->yaw();
          flags             |= Opi::FLAG_HAS_EULER_ANGLES;
          euler_num_inputs  += 1;
        }

        if (opi->flags() & Opi::FLAG_HAS_WORLD_VELOCITY) {
          world_x_velocity    += opi->world_x_velocity();
          world_y_velocity    += opi->world_y_velocity();
          world_z_velocity    += opi->world_z_velocity();
          flags               |= Opi::FLAG_HAS_WORLD_VELOCITY;
          worldvel_num_inputs += 1;
        }
      }

      if (opi->flags() & Opi::FLAG_HAS_RELATIVE_CARTESIAN) {
        have_relative = true;

        flags |= Opi::FLAG_HAS_RELATIVE_CARTESIAN;
        
        relative_x          += opi->relative_x();
        relative_y          += opi->relative_y();
        relative_z          += opi->relative_z();
        relative_x_velocity += opi->relative_x_velocity();
        relative_y_velocity += opi->relative_y_velocity();
        relative_z_velocity += opi->relative_z_velocity();
        relcart_num_inputs  += 1;
      }

      if (opi->flags() & Opi::FLAG_HAS_RELATIVE_POLAR) {
        have_relative = true;
        
        flags |= Opi::FLAG_HAS_RELATIVE_POLAR;
        
        distance            += opi->distance();
        bearing             += opi->bearing();
        slope               += opi->slope();
        relpolar_num_inputs += 1;
      }

      if (opi->flags() & Opi::FLAG_HAS_EXTENT) {
        extent_x          += opi->extent_x();
        extent_y          += opi->extent_y();
        extent_z          += opi->extent_z();
        flags             |= Opi::FLAG_HAS_EXTENT;
        extent_num_inputs += 1;
      }

      if (opi->visibility_history() > vishistory_max) {
        vishistory_max = opi->visibility_history();
      }
    } else {
      if (opi->visibility_history() < vishistory_min) {
        vishistory_min = opi->visibility_history();
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

