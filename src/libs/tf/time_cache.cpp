/***************************************************************************
 *  time_cache.cpp - Fawkes tf time cache (based on ROS tf)
 *
 *  Created: Thu Oct 20 11:26:40 2011
 *  Copyright  2011  Tim Niemueller [www.niemueller.de]
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

/* This code is based on ROS tf with the following copyright and license:
 *
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <tf/time_cache.h>

#include <cstdio>

namespace fawkes {
  namespace tf {
#if 0 /* just to make Emacs auto-indent happy */
  }
}
#endif

/** @class TransformStorage <tf/time_cache.h>
 * Storage for transforms and their parent.
 *
 * @fn TransformStorage & TransformStorage::operator=(const TransformStorage& rhs)
 * Assignment operator.
 * @param rhs storage to assign
 * @return reference to this instance
 */

/** Constructor. */
TransformStorage::TransformStorage()
{
}

/** Constructor.
 * @param data initial stamped transform data
 * @param frame_id parent frame ID
 * @param child_frame_id child frame ID
 */
TransformStorage::TransformStorage(const StampedTransform& data,
                                   CompactFrameID frame_id,
                                   CompactFrameID child_frame_id)
: rotation(data.getRotation())
, translation(data.getOrigin())
, stamp(data.stamp)
, frame_id(frame_id)
, child_frame_id(child_frame_id)
{ }


/** @class TimeCacheInterface <tf/time_cache.h>
 * Interface for transform time caches.
 *
 * @fn virtual TimeCacheInterfacePtr TimeCacheInterface::clone(const fawkes::Time &look_back_until = fawkes::Time(0,0)) const = 0
 * Create a copy of this time cache.
 * @param look_back_until if non-zero time is passed only
 * include transforms younger than the given time.
 * @return shared pointer to copy of this time cache
 *
 * @fn virtual bool TimeCacheInterface::get_data(fawkes::Time time, TransformStorage & data_out, std::string* error_str = 0) = 0
 * Get data.
 * @param time time for which go get data
 * @param data_out upon return contains requested data
 * @param error_str error stirng
 * @return false if data not available
 *
 * @fn virtual bool TimeCacheInterface::insert_data(const TransformStorage& new_data) = 0
 * Insert data.
 * @param new_data data to insert
 * @return true on success, false otherwise
 *
 * @fn virtual void TimeCacheInterface::clear_list() = 0
 * Clear storage.
 *
 * @fn virtual CompactFrameID TimeCacheInterface::get_parent(fawkes::Time time, std::string* error_str) = 0
 * Get parent frame number.
 * @param time point in time
 * @param error_str error string
 * @return frame number
 *
 * @fn virtual P_TimeAndFrameID TimeCacheInterface::get_latest_time_and_parent() = 0
 * Get latest time and parent frame number.
 * @return latest time and parent frame number
 *
 * @fn virtual unsigned int TimeCacheInterface::get_list_length() = 0 const
 * Get storage list length.
 * @return storage list length
 *
 * @fn virtual fawkes::Time TimeCacheInterface::get_latest_timestamp() = 0 const
 * Get latest timestamp from cache.
 * @return latest timestamp
 *
 * @fn virtual fawkes::Time TimeCacheInterface::get_oldest_timestamp() = 0 const
 * Get oldest timestamp from cache.
 * @return oldest time stamp.
 *
 * @fn virtual const L_TransformStorage & TimeCacheInterface::get_storage() const = 0
 * Get storage list.
 * @return reference to list of storage elements
 *
 * @fn L_TransformStorage TimeCacheInterface::get_storage_copy() const = 0
 * Get copy of storage elements.
 * @return copied list of storage elements
 */

/** @class TimeCache <tf/time_cache.h>
 * Time based transform cache.
 * A class to keep a sorted linked list in time. This builds and
 * maintains a list of timestamped data.  And provides lookup
 * functions to get data out as a function of time.
 */

/** Constructor.
 * @param max_storage_time maximum time in seconds to cache, defaults to 10 seconds
 */
TimeCache::TimeCache(float max_storage_time)
: max_storage_time_(max_storage_time)
{}


/** Create extrapolation error string.
 * @param t0 requested time
 * @param t1 available time
 * @param error_str upon return contains the error string.
 */
// hoisting these into separate functions causes an ~8% speedup.
// Removing calling them altogether adds another ~10%
void
create_extrapolation_exception1(fawkes::Time t0, fawkes::Time t1, std::string* error_str)
{
  if (error_str)
  {
    char *tmp;
    if (asprintf(&tmp, "Lookup would require extrapolation at time %li.%li, "
                 "but only time %li.%li is in the buffer", t0.get_sec(), t0.get_nsec(),
                 t1.get_sec(), t1.get_usec()) != -1)
    {
      *error_str = tmp;
      free(tmp);
    }
  }
}

/** Create extrapolation error string.
 * @param t0 requested time
 * @param t1 available time
 * @param error_str upon return contains the error string.
 */
void
create_extrapolation_exception2(fawkes::Time t0, fawkes::Time t1, std::string* error_str)
{
  if (error_str)
  {
    char *tmp;
    if (asprintf(&tmp,"Lookup would require extrapolation into the future. "
                 "Requested time %li.%li, but the latest data is at time %li.%li",
                 t0.get_sec(), t0.get_usec(), t1.get_sec(), t1.get_usec()) != -1)
    {
      *error_str = tmp;
      free(tmp);
    }
  }
}

/** Create extrapolation error string.
 * @param t0 requested time
 * @param t1 available time
 * @param error_str upon return contains the error string.
 */
void
create_extrapolation_exception3(fawkes::Time t0, fawkes::Time t1, std::string* error_str)
{
  if (error_str)
  {
    char *tmp;
    if (asprintf(&tmp,"Lookup would require extrapolation into the past. "
                 "Requested time %li.%li, but the latest data is at time %li.%li",
                 t0.get_sec(), t0.get_usec(), t1.get_sec(), t1.get_usec()) != -1)
    {
      *error_str = tmp;
      free(tmp);
    }
  }
}

/// A helper function for getData
//Assumes storage is already locked for it
uint8_t
TimeCache::find_closest(TransformStorage*& one, TransformStorage*& two,
                        fawkes::Time target_time, std::string* error_str)
{
  //No values stored
  if (storage_.empty()) {
    if (error_str) *error_str = "Transform cache storage is empty";
    return 0;
  }

  //If time == 0 return the latest
  if (target_time.is_zero()) {
    one = &storage_.front();
    return 1;
  }

  // One value stored
  if (++storage_.begin() == storage_.end()) {
    TransformStorage& ts = *storage_.begin();
    if (ts.stamp == target_time) {
      one = &ts;
      return 1;
    } else {
      create_extrapolation_exception1(target_time, ts.stamp, error_str);
      return 0;
    }
  }

  fawkes::Time latest_time = (*storage_.begin()).stamp;
  fawkes::Time earliest_time = (*(storage_.rbegin())).stamp;

  if (target_time == latest_time) {
    one = &(*storage_.begin());
    return 1;
  } else if (target_time == earliest_time) {
    one = &(*storage_.rbegin());
    return 1;
  } else if (target_time > latest_time) {
    // Catch cases that would require extrapolation
    create_extrapolation_exception2(target_time, latest_time, error_str);
    return 0;
  } else if (target_time < earliest_time) {
    create_extrapolation_exception3(target_time, earliest_time, error_str);
    return 0;
  }

  //At least 2 values stored
  //Find the first value less than the target value
  L_TransformStorage::iterator storage_it = storage_.begin();
  while(storage_it != storage_.end()) {
    if (storage_it->stamp <= target_time)  break;
    storage_it++;
  }

  //Finally the case were somewhere in the middle  Guarenteed no extrapolation :-)
  one = &*(storage_it); //Older
  two = &*(--storage_it); //Newer
  return 2;
}


void
TimeCache::interpolate(const TransformStorage& one,
                       const TransformStorage& two,
                       fawkes::Time time, TransformStorage& output)
{
  // Check for zero distance case
  if( two.stamp == one.stamp ) {
    output = two;
    return;
  }
  //Calculate the ratio
  btScalar ratio =
    (time.in_sec() - one.stamp.in_sec()) /
    (two.stamp.in_sec() - one.stamp.in_sec());

  //Interpolate translation
  output.translation.setInterpolate3(one.translation, two.translation, ratio);

  //Interpolate rotation
  output.rotation = slerp( one.rotation, two.rotation, ratio);

  output.stamp = one.stamp;
  output.frame_id = one.frame_id;
  output.child_frame_id = one.child_frame_id;
}

TimeCacheInterfacePtr
TimeCache::clone(const fawkes::Time &look_back_until) const
{
	TimeCache *copy = new TimeCache(max_storage_time_);
	if (look_back_until.is_zero()) {
		copy->storage_ = storage_;
	} else {
		L_TransformStorage::const_iterator storage_it = storage_.begin();
		for (storage_it = storage_.begin(); storage_it != storage_.end(); ++storage_it) {
			if (storage_it->stamp <= look_back_until)  break;
			copy->storage_.push_back(*storage_it);
		}
	}
	return std::shared_ptr<TimeCacheInterface>(copy);
}


bool
TimeCache::get_data(fawkes::Time time, TransformStorage & data_out,
                    std::string* error_str)
{
  TransformStorage* p_temp_1 = NULL;
  TransformStorage* p_temp_2 = NULL;

  int num_nodes = find_closest(p_temp_1, p_temp_2, time, error_str);
  if (num_nodes == 0) {
    return false;
  } else if (num_nodes == 1) {
    data_out = *p_temp_1;
  } else if (num_nodes == 2) {
    if( p_temp_1->frame_id == p_temp_2->frame_id) {
      interpolate(*p_temp_1, *p_temp_2, time, data_out);
    } else {
      data_out = *p_temp_1;
    }
  }

  return true;
}

CompactFrameID
TimeCache::get_parent(fawkes::Time time, std::string* error_str)
{
  TransformStorage* p_temp_1 = NULL;
  TransformStorage* p_temp_2 = NULL;

  int num_nodes = find_closest(p_temp_1, p_temp_2, time, error_str);
  if (num_nodes == 0) {
    return 0;
  }

  return p_temp_1->frame_id;
}


bool
TimeCache::insert_data(const TransformStorage& new_data)
{
  L_TransformStorage::iterator storage_it = storage_.begin();

  if(storage_it != storage_.end()) {
    if (storage_it->stamp > new_data.stamp + max_storage_time_) {
      return false;
    }
  }


  while(storage_it != storage_.end()) {
    if (storage_it->stamp <= new_data.stamp)
      break;
    storage_it++;
  }
  storage_.insert(storage_it, new_data);

  prune_list();
  return true;
}

void
TimeCache::clear_list()
{
  storage_.clear();
}

unsigned int
TimeCache::get_list_length() const
{
  return storage_.size();
}


const TimeCacheInterface::L_TransformStorage &
TimeCache::get_storage() const
{
  return storage_;
}

TimeCacheInterface::L_TransformStorage
TimeCache::get_storage_copy() const
{
  return storage_;
}

P_TimeAndFrameID
TimeCache::get_latest_time_and_parent()
{
  if (storage_.empty()) {
    return std::make_pair(fawkes::Time(), 0);
  }

  const TransformStorage& ts = storage_.front();
  return std::make_pair(ts.stamp, ts.frame_id);
}

fawkes::Time
TimeCache::get_latest_timestamp() const
{
  if (storage_.empty()) return fawkes::Time(0,0); //empty list case
  return storage_.front().stamp;
}

fawkes::Time
TimeCache::get_oldest_timestamp() const
{
  if (storage_.empty()) return fawkes::Time(0,0); //empty list case
  return storage_.back().stamp;
}

/** Prune storage list based on maximum cache lifetime. */
void
TimeCache::prune_list()
{
  fawkes::Time latest_time = storage_.begin()->stamp;
  
  while(!storage_.empty() &&
        storage_.back().stamp + max_storage_time_ < latest_time)
  {
    storage_.pop_back();
  }

}


} // end namespace tf
} // end namespace fawkes
