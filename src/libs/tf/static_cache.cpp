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
#include <tf/exceptions.h>
#include <tf/types.h>

namespace fawkes {
  namespace tf {
#if 0 /* just to make Emacs auto-indent happy */
  }
}
#endif

/** @class StaticCache <tf/time_cache.h>
 * Transform cache for static transforms.
 */

/** Constructor.
 */
StaticCache::StaticCache()
	: storage_as_list_(1)
{
}

/** Create a copy of this time cache.
 * @param look_back_until Ignored for static caches
 * @return shared pointer to copy of this time cache
 */
TimeCacheInterfacePtr
StaticCache::clone(const fawkes::Time &look_back_until) const
{
	StaticCache *copy = new StaticCache();
	copy->storage_ = storage_;
	copy->storage_as_list_ = storage_as_list_;
	return std::shared_ptr<TimeCacheInterface>(copy);
}

bool
StaticCache::get_data(fawkes::Time time, TransformStorage & data_out,
                      std::string* error_str)
{
	data_out = storage_;
	data_out.stamp = time;
	return true;
}

bool
StaticCache::insert_data(const TransformStorage& new_data)
{
  storage_ = new_data;
  storage_as_list_.front() = new_data;
  return true;
}

void
StaticCache::clear_list()
{
	return;
}

unsigned int
StaticCache::get_list_length() const
{
	return 1;
}

CompactFrameID
StaticCache::get_parent(fawkes::Time time, std::string *error_str)
{
  return storage_.frame_id;
}

P_TimeAndFrameID
StaticCache::get_latest_time_and_parent()
{
	return std::make_pair(fawkes::Time(0,0), storage_.frame_id);
}

fawkes::Time
StaticCache::get_latest_timestamp() const
{
	return fawkes::Time(0,0);
}

fawkes::Time
StaticCache::get_oldest_timestamp() const
{   
	return fawkes::Time(0,0);
}

const TimeCacheInterface::L_TransformStorage &
StaticCache::get_storage() const
{
  return storage_as_list_;
}

TimeCacheInterface::L_TransformStorage
StaticCache::get_storage_copy() const
{
  return storage_as_list_;
}


} // end namespace tf
} // end namespace fawkes
