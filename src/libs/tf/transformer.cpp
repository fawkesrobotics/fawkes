/***************************************************************************
 *  transformer.cpp - Fawkes tf transformer (based on ROS tf)
 *
 *  Created: Thu Oct 20 10:56:25 2011
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

#include <tf/transformer.h>
#include <tf/time_cache.h>
#include <tf/exceptions.h>
#include <tf/utils.h>

#include <core/threading/mutex_locker.h>
#include <iostream>
#include <sstream>
#include <algorithm>

namespace fawkes {
  namespace tf {
#if 0 /* just to make Emacs auto-indent happy */
  }
}
#endif

/** @class Transformer <tf/transformer.h>
 * Coordinate transforms between any two frames in a system.
 *
 * This class provides a simple interface to allow recording and
 * lookup of relationships between arbitrary frames of the system.
 *
 * TF assumes that there is a tree of coordinate frame transforms
 * which define the relationship between all coordinate frames.  For
 * example your typical robot would have a transform from global to
 * real world.  And then from base to hand, and from base to head.
 * But Base to Hand really is composed of base to shoulder to elbow to
 * wrist to hand.  TF is designed to take care of all the intermediate
 * steps for you.
 *
 * Internal Representation TF will store frames with the parameters
 * necessary for generating the transform into that frame from it's
 * parent and a reference to the parent frame.  Frames are designated
 * using an std::string 0 is a frame without a parent (the top of a
 * tree) The positions of frames over time must be pushed in.
 *
 * All function calls which pass frame ids can potentially throw the
 * exception fawkes::tf::LookupException
 *
 * 
 * @fn bool Transformer::is_enabled() const
 * Check if transformer is enabled.
 * @return true if enabled, false otherwise
 *
 * @var static const unsigned int Transformer::MAX_GRAPH_DEPTH
 * Maximum number of times to recurse before assuming the tree
 * has a loop
 *
 * @param static const int64_t Transformer::DEFAULT_MAX_EXTRAPOLATION_DISTANCE
 * The default amount of time to extrapolate.
 */

/** The default amount of time to extrapolate. */
const float Transformer::DEFAULT_MAX_EXTRAPOLATION_DISTANCE = 0.f;

/// Flag to advise accumulator finalization.
enum WalkEnding
{
  Identity,	///< identity
  TargetParentOfSource,	///< Target is parent of source
  SourceParentOfTarget,	///< Source is parent of target
  FullPath,	///< Full path between source and target.
};

/// Internal error values
enum ErrorValues {
  NO_ERROR = 0, ///< No error occured.
  LOOKUP_ERROR, ///< Frame ID lookup error
  CONNECTIVITY_ERROR, ///< No connection between frames found
  EXTRAPOLATION_ERROR	///< Extrapolation required out of limits.
};

/** Accumulator to test for transformability.
 * Operations are basically noops.
 */
struct CanTransformAccum
{
  /** Gather frame number.
   * @param cache cache
   * @param time time
   * @param error_string string containing error message in case of failure
   * @return parent frame number
   */
  CompactFrameID
  gather(TimeCache* cache, fawkes::Time time, std::string* error_string)
  {
    return cache->get_parent(time, error_string);
  }

  /** Accumulate.
   * @param source true if looking from source
   */
  void accum(bool source)
  {
  }

  /** Finalize accumulation.
   * @param end flag how the walk ended
   * @param _time time
   */
  void finalize(WalkEnding end, fawkes::Time _time)
  {
  }

  /// Transform storage to use.
  TransformStorage st;
};

/** Accumulator to accumulate transforms between two frames.
 */
struct TransformAccum
{
  /** Constructor. */
  TransformAccum()
  : source_to_top_quat(0.0, 0.0, 0.0, 1.0)
  , source_to_top_vec(0.0, 0.0, 0.0)
  , target_to_top_quat(0.0, 0.0, 0.0, 1.0)
  , target_to_top_vec(0.0, 0.0, 0.0)
  , result_quat(0.0, 0.0, 0.0, 1.0)
  , result_vec(0.0, 0.0, 0.0)
  {
  }

  /** Gather frame number.
   * @param cache cache
   * @param time time
   * @param error_string string containing error message in case of failure
   * @return parent frame number
   */
  CompactFrameID
  gather(TimeCache* cache, fawkes::Time time, std::string* error_string)
  {
    if (!cache->get_data(time, st, error_string))
    {
      return 0;
    }

    return st.frame_id_;
  }

  /** Accumulate.
   * @param source true if looking from source
   */
  void accum(bool source)
  {
    if (source)
    {
      source_to_top_vec = quatRotate(st.rotation_, source_to_top_vec) + st.translation_;
      source_to_top_quat = st.rotation_ * source_to_top_quat;
    }
    else
    {
      target_to_top_vec = quatRotate(st.rotation_, target_to_top_vec) + st.translation_;
      target_to_top_quat = st.rotation_ * target_to_top_quat;
    }
  }

  /** Finalize accumulation.
   * @param end flag how the walk ended
   * @param _time time
   */
  void finalize(WalkEnding end, fawkes::Time _time)
  {
    switch (end)
    {
    case Identity:
      break;
    case TargetParentOfSource:
      result_vec = source_to_top_vec;
      result_quat = source_to_top_quat;
      break;
    case SourceParentOfTarget:
      {
        btQuaternion inv_target_quat = target_to_top_quat.inverse();
        btVector3 inv_target_vec = quatRotate(inv_target_quat, -target_to_top_vec);
        result_vec = inv_target_vec;
        result_quat = inv_target_quat;
        break;
      }
    case FullPath:
      {
        btQuaternion inv_target_quat = target_to_top_quat.inverse();
        btVector3 inv_target_vec = quatRotate(inv_target_quat, -target_to_top_vec);

     	result_vec = quatRotate(inv_target_quat, source_to_top_vec) + inv_target_vec;
        result_quat = inv_target_quat * source_to_top_quat;
      }
      break;
    };

    time = _time;
  }

  /// Transform storage.
  TransformStorage st;
  /// time stamp
  fawkes::Time time;
  /// Source to top accumulated quaternion
  btQuaternion source_to_top_quat;
  /// Source to top accumulated vector
  btVector3 source_to_top_vec;
  /// Target to top accumulated quaternion
  btQuaternion target_to_top_quat;
  /// Target to top accumulated vector
  btVector3 target_to_top_vec;

  /// After finalize contains result quaternion.
  btQuaternion result_quat;
  /// After finalize contains result vector.
  btVector3 result_vec;
};

/** Constructor.
 * @param cache_time time in seconds to cache incoming transforms
 */
Transformer::Transformer(float cache_time)
  : cache_time_(cache_time),
    fall_back_to_wall_time_(false)
{
  max_extrapolation_distance_ = DEFAULT_MAX_EXTRAPOLATION_DISTANCE;
  frameIDs_["NO_PARENT"] = 0;
  //unused but needed for iteration over all elements
  frames_.push_back(NULL);
  frameIDs_reverse.push_back("NO_PARENT");

  frame_mutex_ = new Mutex();
}


/** Destructor. */
Transformer::~Transformer()
{
  // deallocate all frames
  MutexLocker lock(frame_mutex_);
  std::vector<TimeCache*>::iterator cache_it;
  for (cache_it = frames_.begin(); cache_it != frames_.end(); ++cache_it)
  {
    delete (*cache_it);
  }

};


/** Set transformer enabled or disabled.
 * @param enabled true to enable, false to disable
 */
void
Transformer::set_enabled(bool enabled)
{
  enabled_ = enabled;
}

/** Clear cached transforms. */
void
Transformer::clear()
{
  MutexLocker lock(frame_mutex_);
  if ( frames_.size() > 1 )
  {
    std::vector< TimeCache*>::iterator cache_it;
    for (cache_it = frames_.begin() + 1; cache_it != frames_.end(); ++cache_it)
    {
      (*cache_it)->clear_list();
    }
  }
}


/** Check if frame exists.
 * @param frame_id_str frame ID
 * @result true if frame exists, false otherwise
 */
bool
Transformer::frame_exists(const std::string& frame_id_str) const
{
  MutexLocker lock(frame_mutex_);
  std::string frame_id_resolveped = /*assert_resolved(tf_prefix_,*/ frame_id_str;
  
  return frameIDs_.count(frame_id_resolveped);
}

void
Transformer::create_connectivity_error_string(CompactFrameID source_frame, CompactFrameID target_frame, std::string* out) const
{
  if (!out) return;

  *out = std::string("Could not find a connection between '"+lookup_frame_string(target_frame)+"' and '"+
                     lookup_frame_string(source_frame)+"' because they are not part of the same tree."+
                     "Tf has two or more unconnected trees.");
}


/** Walk from frame to top-parent of both.
 * @param f accumulator
 * @param time timestamp
 * @param target_id frame number of target
 * @param source_id frame number of source
 * @param error_string accumulated error string
 * @return error flag from ErrorValues
 */
template<typename F>
int
Transformer::walk_to_top_parent(F& f, fawkes::Time time,
                                CompactFrameID target_id, CompactFrameID source_id,
                                std::string* error_string) const
{
  // Short circuit if zero length transform to allow lookups on non existant links
  if (source_id == target_id)
  {
    f.finalize(Identity, time);
    return NO_ERROR;
  }

  //If getting the latest get the latest common time
  if (time == fawkes::Time(0,0))
  {
    int retval = get_latest_common_time(target_id, source_id, time, error_string);
    if (retval != NO_ERROR)
    {
      return retval;
    }
  }

  // Walk the tree to its root from the source frame, accumulating the transform
  CompactFrameID frame = source_id;
  CompactFrameID top_parent = frame;
  uint32_t depth = 0;
  while (frame != 0)
  {
    TimeCache* cache = get_frame(frame);

    if (!cache)
    {
      // There will be no cache for the very root of the tree
      top_parent = frame;
      break;
    }

    CompactFrameID parent = f.gather(cache, time, 0);
    if (parent == 0)
    {
      // Just break out here... there may still be a path from source -> target
      top_parent = frame;
      break;
    }

    // Early out... target frame is a direct parent of the source frame
    if (frame == target_id)
    {
      f.finalize(TargetParentOfSource, time);
      return NO_ERROR;
    }

    f.accum(true);

    top_parent = frame;
    frame = parent;

    ++depth;
    if (depth > MAX_GRAPH_DEPTH)
    {
      if (error_string)
      {
        std::stringstream ss;
        //ss << "The tf tree is invalid because it contains a loop." << std::endl
        //   << allFramesAsString() << std::endl;
        *error_string = ss.str();
      }
      return LOOKUP_ERROR;
    }
  }

  // Now walk to the top parent from the target frame, accumulating its transform
  frame = target_id;
  depth = 0;
  while (frame != top_parent)
  {
    TimeCache* cache = get_frame(frame);

    if (!cache)
    {
      break;
    }

    CompactFrameID parent = f.gather(cache, time, error_string);
    if (parent == 0)
    {
      if (error_string)
      {
        std::stringstream ss;
        ss << "Looking up transform from frame [" << lookup_frame_string(source_id) <<
          "] to frame [" << lookup_frame_string(target_id) << "] failed";
        *error_string = ss.str();
      }

      return EXTRAPOLATION_ERROR;
    }

    // Early out... source frame is a direct parent of the target frame
    if (frame == source_id)
    {
      f.finalize(SourceParentOfTarget, time);
      return NO_ERROR;
    }

    f.accum(false);

    frame = parent;

    ++depth;
    if (depth > MAX_GRAPH_DEPTH)
    {
      if (error_string)
      {
        std::stringstream ss;
        //ss << "The tf tree is invalid because it contains a loop." << std::endl
        //   << allFramesAsString() << std::endl;
        *error_string = "The tf tree is invalid because it contains a loop.";
      }
      return LOOKUP_ERROR;
    }
  }

  if (frame != top_parent)
  {
    create_connectivity_error_string(source_id, target_id, error_string);
    return CONNECTIVITY_ERROR;
  }

  f.finalize(FullPath, time);

  return NO_ERROR;
}

/** Time and frame ID comparator class. */
struct TimeAndFrameIDFrameComparator
{
  /** Constructor.
   * @param id frame number to look for
   */
  TimeAndFrameIDFrameComparator(CompactFrameID id)
  : id(id)
  {}

  /** Application operator.
   * @param rhs right-hand side to compare to
   * @return true if the frame numbers are equal, false otherwise
   */
  bool operator()(const P_TimeAndFrameID& rhs) const
  {
    return rhs.second == id;
  }

  /// Frame number to look for.
  CompactFrameID id;
};


/** Get latest common time of two frames.
 * @param target_id target frame number
 * @param source_id source frame number
 * @param time upon return contains latest common timestamp
 * @param error_string upon error contains accumulated error message.
 * @return value from ErrorValues
 */
int
Transformer::get_latest_common_time(CompactFrameID target_id, CompactFrameID source_id,
                                 fawkes::Time & time, std::string *error_string) const
{
  if (source_id == target_id)
  {
    //Set time to latest timestamp of frameid in case of target and source frame id are the same
    time.stamp();
    return NO_ERROR;
  }

  std::vector<P_TimeAndFrameID> lct_cache;

  // Walk the tree to its root from the source frame, accumulating the list of parent/time as well as the latest time
  // in the target is a direct parent
  CompactFrameID frame = source_id;
  P_TimeAndFrameID temp;
  uint32_t depth = 0;
  fawkes::Time common_time = fawkes::TIME_MAX;
  while (frame != 0)
  {
    TimeCache* cache = get_frame(frame);

    if (!cache) {
      // There will be no cache for the very root of the tree
      break;
    }

    P_TimeAndFrameID latest = cache->get_latest_time_and_parent();

    if (latest.second == 0) {
      // Just break out here... there may still be a path from source -> target
      break;
    }

    if (!latest.first.is_zero()) {
      common_time = std::min(latest.first, common_time);
    }

    lct_cache.push_back(latest);

    frame = latest.second;

    // Early out... target frame is a direct parent of the source frame
    if (frame == target_id) {
      time = common_time;
      if (time == fawkes::TIME_MAX) {
        time = fawkes::Time();
      }
      return NO_ERROR;
    }

    ++depth;
    if (depth > MAX_GRAPH_DEPTH) {
      if (error_string) {
        /*
        std::stringstream ss;
        ss<<"The tf tree is invalid because it contains a loop." << std::endl
          << allFramesAsString() << std::endl;
        *error_string = ss.str();
        */
      }
      return LOOKUP_ERROR;
    }
  }

  // Now walk to the top parent from the target frame, accumulating the latest time and looking for a common parent
  frame = target_id;
  depth = 0;
  common_time = fawkes::TIME_MAX;
  CompactFrameID common_parent = 0;
  while (true)
  {
    TimeCache* cache = get_frame(frame);

    if (!cache) {
      break;
    }

    P_TimeAndFrameID latest = cache->get_latest_time_and_parent();

    if (latest.second == 0) {
      break;
    }

    if (!latest.first.is_zero()) {
      common_time = std::min(latest.first, common_time);
    }

    std::vector<P_TimeAndFrameID>::iterator it =
      std::find_if(lct_cache.begin(), lct_cache.end(),
                   TimeAndFrameIDFrameComparator(latest.second));

    if (it != lct_cache.end()) { // found a common parent
      common_parent = it->second;
      break;
    }

    frame = latest.second;

    // Early out... source frame is a direct parent of the target frame
    if (frame == source_id) {
      time = common_time;
      if (time == fawkes::TIME_MAX) {
        time = fawkes::Time();
      }
      return NO_ERROR;
    }

    ++depth;
    if (depth > MAX_GRAPH_DEPTH) {
      if (error_string) {
        std::stringstream ss;
        //ss<<"The tf tree is invalid because it contains a loop." << std::endl
        //  << allFramesAsString() << std::endl;
        *error_string = ss.str();
      }
      return LOOKUP_ERROR;
    }
  }

  if (common_parent == 0) {
    create_connectivity_error_string(source_id, target_id, error_string);
    return CONNECTIVITY_ERROR;
  }

  // Loop through the source -> root list until we hit the common parent
  {
    std::vector<P_TimeAndFrameID>::iterator it = lct_cache.begin();
    std::vector<P_TimeAndFrameID>::iterator end = lct_cache.end();
    for (; it != end; ++it) {
      if (!it->first.is_zero()) {
        common_time = std::min(common_time, it->first);
      }

      if (it->second == common_parent) {
        break;
      }
    }
  }

  if (common_time == fawkes::TIME_MAX) {
    common_time = fawkes::Time();
  }

  time = common_time;
  return NO_ERROR;
}

/** Get latest common time of two frames.
 * @param source_frame source frame ID
 * @param target_frame frame target frame ID
 * @param time upon return contains latest common timestamp
 * @param error_string upon error contains accumulated error message.
 * @return value from ErrorValues
 */
int
Transformer::get_latest_common_time(const std::string &source_frame,
                                    const std::string &target_frame,
                                    fawkes::Time& time,
                                    std::string* error_string) const
{
  std::string mapped_tgt = /*assert_resolved(tf_prefix_,*/ target_frame;
  std::string mapped_src = /*assert_resolved(tf_prefix_,*/ source_frame;

  if (!frame_exists(mapped_tgt) || !frame_exists(mapped_src)) {
    time = fawkes::Time();
    return LOOKUP_ERROR;
  }

  CompactFrameID source_id = lookup_frame_number(mapped_src);
  CompactFrameID target_id = lookup_frame_number(mapped_tgt);
  return get_latest_common_time(source_id, target_id, time, error_string);
}

/** Set a transform.
 * @param transform transform to set
 * @param authority authority from which the transform was received.
 * @return true on success, false otherwise
 */
bool
Transformer::set_transform(const StampedTransform &transform, const std::string &authority)
{
  StampedTransform mapped_transform((btTransform)transform, transform.stamp,
                                    transform.frame_id, transform.child_frame_id);
  mapped_transform.child_frame_id = /*assert_resolved(tf_prefix_, */ transform.child_frame_id;
  mapped_transform.frame_id = /*assert_resolved(tf_prefix_,*/ transform.frame_id;

 
  bool error_exists = false;
  if (mapped_transform.child_frame_id == mapped_transform.frame_id)
  {
    printf("TF_SELF_TRANSFORM: Ignoring transform from authority \"%s\" with frame_id and child_frame_id  \"%s\" because they are the same",  authority.c_str(), mapped_transform.child_frame_id.c_str());
    error_exists = true;
  }

  if (mapped_transform.child_frame_id == "/")//empty frame id will be mapped to "/"
  {
    printf("TF_NO_CHILD_FRAME_ID: Ignoring transform from authority \"%s\" because child_frame_id not set ", authority.c_str());
    error_exists = true;
  }

  if (mapped_transform.frame_id == "/")//empty parent id will be mapped to "/"
  {
    printf("TF_NO_FRAME_ID: Ignoring transform with child_frame_id \"%s\"  from authority \"%s\" because frame_id not set", mapped_transform.child_frame_id.c_str(), authority.c_str());
    error_exists = true;
  }

  if (std::isnan(mapped_transform.getOrigin().x()) || std::isnan(mapped_transform.getOrigin().y()) || std::isnan(mapped_transform.getOrigin().z())||
      std::isnan(mapped_transform.getRotation().x()) ||       std::isnan(mapped_transform.getRotation().y()) ||       std::isnan(mapped_transform.getRotation().z()) ||       std::isnan(mapped_transform.getRotation().w()))
  {
    printf("TF_NAN_INPUT: Ignoring transform for child_frame_id \"%s\" from authority \"%s\" because of a nan value in the transform (%f %f %f) (%f %f %f %f)",
           mapped_transform.child_frame_id.c_str(), authority.c_str(),
           mapped_transform.getOrigin().x(), mapped_transform.getOrigin().y(), mapped_transform.getOrigin().z(),
           mapped_transform.getRotation().x(), mapped_transform.getRotation().y(), mapped_transform.getRotation().z(), mapped_transform.getRotation().w()
              );
    error_exists = true;
  }

  if (error_exists)
    return false;

  {
    MutexLocker lock(frame_mutex_);
    CompactFrameID frame_number = lookup_or_insert_frame_number(mapped_transform.child_frame_id);
    TimeCache* frame = get_frame(frame_number);
    if (frame == NULL) {
      frames_[frame_number] = new TimeCache(cache_time_);
      frame = frames_[frame_number];
    }

    if (frame->insert_data(TransformStorage(mapped_transform, lookup_or_insert_frame_number(mapped_transform.frame_id), frame_number)))
    {
      frame_authority_[frame_number] = authority;
    } else {
      //ROS_WARN("TF_OLD_DATA ignoring data from the past for frame %s at time %g according to authority %s\nPossible reasons are listed at ", mapped_transform.child_frame_id.c_str(), mapped_transform.stamp.toSec(), authority.c_str());
      return false;
    }
  }

  return true;
};


/** An accessor to get a frame.
 * This is an internal function which will get the pointer to the
 * frame associated with the frame id
 * @param frame_number The frameID of the desired reference frame
 * @return time cache for given frame number
 * @exception LookupException thrown if lookup fails
 */
TimeCache *
Transformer::get_frame(unsigned int frame_number) const
{
  if (frame_number == 0) /// @todo check larger values too
    return NULL;
  else
    return frames_[frame_number];
}


/** An accessor to get access to the frame cache.
 * @param frame_id frame ID
 * @return time cache for given frame ID
 * @exception LookupException thrown if lookup fails
 */
const TimeCache *
Transformer::get_frame_cache(const std::string &frame_id) const
{
  CompactFrameID frame_number = lookup_frame_number(frame_id);
  if (frame_number == 0) {
    throw LookupException("Failed to lookup frame %s", frame_id.c_str());
  } else {
    return frames_[frame_number];
  }
}


/** String to number for frame lookup.
 * @param frameid_str frame ID
 * @return frame number
 * @exception LookupException if frame ID is unknown
 */
CompactFrameID
Transformer::lookup_frame_number(const std::string &frameid_str) const
{
  unsigned int retval = 0;
  //MutexLocker lock(frame_mutex_);
  M_StringToCompactFrameID::const_iterator map_it = frameIDs_.find(frameid_str);
  if (map_it == frameIDs_.end())
  {
    throw LookupException("Frame id %s does not exist! Frames (%zu): %s",
                          frameid_str.c_str(), frameIDs_.size(), "" /*allFramesAsString()*/);
  }
  else
    retval = map_it->second;
  return retval;
};


/** String to number for frame lookup with dynamic allocation of new
 * frames.
 * @param frameid_str frame ID
 * @return frame number, if none existed for the given frame ID a new
 * one is created
 */
CompactFrameID
Transformer::lookup_or_insert_frame_number(const std::string &frameid_str)
{
  unsigned int retval = 0;
  //MutexLocker lock(frame_mutex_);
  M_StringToCompactFrameID::iterator map_it = frameIDs_.find(frameid_str);
  if (map_it == frameIDs_.end())
  {
    retval = frames_.size();
    frameIDs_[frameid_str] = retval;
    frames_.push_back( new TimeCache(cache_time_));
    frameIDs_reverse.push_back(frameid_str);
  }
  else
    retval = frameIDs_[frameid_str];
  return retval;
};

/** Get frame ID given its number.
 * @param frame_num frame number
 * @return frame ID string
 * @exception LookupException thrown if number invalid
 */
std::string
Transformer::lookup_frame_string(unsigned int frame_num) const
{
  if (frame_num >= frameIDs_reverse.size()) {
    throw LookupException("Reverse lookup of frame id %u failed!", frame_num);
  }
  else return frameIDs_reverse[frame_num];
}


/** Lookup transform.
 * @param target_frame target frame ID
 * @param source_frame source frame ID
 * @param time time for which to get the transform, set to (0,0) to get latest
 * common time frame
 * @param transform upon return contains the transform
 * @exception ConnectivityException thrown if no connection between
 * the source and target frame could be found in the tree.
 * @exception ExtrapolationException returning a value would have
 * required extrapolation beyond current limits.
 * @exception LookupException at least one of the two given frames is
 * unknown
 */
void Transformer::lookup_transform(const std::string& target_frame,
                                   const std::string& source_frame,
                                   const fawkes::Time& time,
                                   StampedTransform& transform) const
{
  if (! enabled_) {
    throw DisabledException("Transformer has been disabled");
  }

  std::string mapped_tgt = /*assert_resolved(tf_prefix_,*/ target_frame;
  std::string mapped_src = /*assert_resolved(tf_prefix_,*/ source_frame;

  if (mapped_tgt == mapped_src) {
    transform.setIdentity();
    transform.child_frame_id = mapped_src;
    transform.frame_id       = mapped_tgt;
    transform.stamp          = fawkes::Time();
    return;
  }

  MutexLocker lock(frame_mutex_);

  CompactFrameID target_id = lookup_frame_number(mapped_tgt);
  CompactFrameID source_id = lookup_frame_number(mapped_src);

  std::string error_string;
  TransformAccum accum;
  int rv = walk_to_top_parent(accum, time, target_id, source_id, &error_string);
  if (rv != NO_ERROR)
  {
    switch (rv)
    {
    case CONNECTIVITY_ERROR:
      throw ConnectivityException("Connectivity: %s", error_string.c_str());
    case EXTRAPOLATION_ERROR:
      throw ExtrapolationException("Extrapolation: %s", error_string.c_str());
    case LOOKUP_ERROR:
      throw LookupException("Lookup: %s", error_string.c_str());
    default:
      throw Exception("lookup_transform: unknown error code: %d", rv);
      break;
    }
  }

  transform.setOrigin(accum.result_vec);
  transform.setRotation(accum.result_quat);
  transform.child_frame_id = mapped_src;
  transform.frame_id       = mapped_tgt;
  transform.stamp          = accum.time;
}


/** Lookup transform at latest common time.
 * @param target_frame target frame ID
 * @param source_frame source frame ID
 * @param transform upon return contains the transform
 * @exception ConnectivityException thrown if no connection between
 * the source and target frame could be found in the tree.
 * @exception ExtrapolationException returning a value would have
 * required extrapolation beyond current limits.
 * @exception LookupException at least one of the two given frames is
 * unknown
 */
void Transformer::lookup_transform(const std::string& target_frame,
                                   const std::string& source_frame,
                                   StampedTransform& transform) const
{
  lookup_transform(target_frame, source_frame, fawkes::Time(0,0), transform);
}


/** Lookup transform assuming a fixed frame.
 * This will lookup a transformation from source to target, assuming
 * that there is a fixed frame, by first finding the transform of the
 * source frame in the fixed frame, and then a transformation from the
 * fixed frame to the target frame.
 * @param target_frame target frame ID
 * @param target_time time for the target frame
 * @param source_frame source frame ID
 * @param source_time time in the source frame
 * @param fixed_frame ID of fixed frame
 * @param transform upon return contains the transform
 * @exception ConnectivityException thrown if no connection between
 * the source and target frame could be found in the tree.
 * @exception ExtrapolationException returning a value would have
 * required extrapolation beyond current limits.
 * @exception LookupException at least one of the two given frames is
 * unknown
 */
void
Transformer::lookup_transform(const std::string& target_frame,
                              const fawkes::Time& target_time,
                              const std::string& source_frame,
                              const fawkes::Time& source_time,
                              const std::string& fixed_frame,
                              StampedTransform& transform) const
{
  StampedTransform temp1, temp2;
  lookup_transform(fixed_frame, source_frame, source_time, temp1);
  lookup_transform(target_frame, fixed_frame, target_time, temp2);
  transform.set_data(temp2 * temp1);
  transform.stamp = temp2.stamp;
  transform.frame_id = target_frame;
  transform.child_frame_id = source_frame;
}

/** Test if a transform is possible.
 * @param target_frame The frame into which to transform
 * @param source_frame The frame from which to transform
 * @param time The time at which to transform
 * @return true if the transformation can be calculated, false otherwise
 */
bool
Transformer::can_transform(const std::string& target_frame,
                           const std::string& source_frame,
                           const fawkes::Time& time) const
{
  std::string mapped_tgt = /*assert_resolved(tf_prefix_,*/ target_frame;
  std::string mapped_src = /*assert_resolved(tf_prefix_,*/ source_frame;

  if (mapped_tgt == mapped_src)  return true;

  if (!frame_exists(mapped_tgt) || !frame_exists(mapped_src))  return false;

  MutexLocker lock(frame_mutex_);
  CompactFrameID target_id = lookup_frame_number(mapped_tgt);
  CompactFrameID source_id = lookup_frame_number(mapped_src);

  return can_transform_no_lock(target_id, source_id, time);
}


/** Test if a transform is possible.
 * @param target_frame The frame into which to transform
 * @param target_time The time into which to transform
 * @param source_frame The frame from which to transform
 * @param source_time The time from which to transform
 * @param fixed_frame The frame in which to treat the transform as
 * constant in time
 * @return true if the transformation can be calculated, false otherwise
 */
bool
Transformer::can_transform(const std::string& target_frame,
                           const fawkes::Time& target_time,
                           const std::string& source_frame,
                           const fawkes::Time& source_time,
                           const std::string& fixed_frame) const
{
  return
    can_transform(target_frame, fixed_frame, target_time) &&
    can_transform(fixed_frame, source_frame, source_time);
}


/** Test if a transform is possible.
 * Internal check that does not lock the frame mutex.
 * @param target_id The frame number into which to transform
 * @param source_id The frame number from which to transform
 * @param time The time at which to transform
 * @return true if the transformation can be calculated, false otherwise
 */
bool
Transformer::can_transform_no_lock(CompactFrameID target_id,
                                   CompactFrameID source_id,
                                   const fawkes::Time& time) const
{
  if (target_id == 0 || source_id == 0)  return false;

  CanTransformAccum accum;
  if (walk_to_top_parent(accum, time, target_id, source_id, NULL) == NO_ERROR)
    return true;
  else
    return false;
}


/** Transform a stamped Quaternion into the target frame.
 * This transforms the quaternion relative to the frame set in the
 * stamped quaternion into the target frame.
 * @param target_frame frame into which to transform
 * @param stamped_in stamped quaternion, defines source frame and time
 * @param stamped_out stamped output quaternion in target_frame
 * @exception ConnectivityException thrown if no connection between
 * the source and target frame could be found in the tree.
 * @exception ExtrapolationException returning a value would have
 * required extrapolation beyond current limits.
 * @exception LookupException at least one of the two given frames is
 * unknown
 * @exception InvalidArgument thrown if the Quaternion is invalid, most
 * likely an uninitialized Quaternion (0,0,0,0).
 */
void
Transformer::transform_quaternion(const std::string& target_frame,
                                  const Stamped<Quaternion>& stamped_in,
                                  Stamped<Quaternion>& stamped_out) const
{
  assert_quaternion_valid(stamped_in);

  StampedTransform transform;
  lookup_transform(target_frame, stamped_in.frame_id,
                   stamped_in.stamp, transform);

  stamped_out.set_data(transform * stamped_in);
  stamped_out.stamp = transform.stamp;
  stamped_out.frame_id = target_frame;
}

/** Transform a stamped vector into the target frame.
 * This transforms the vector given relative to the frame set in the
 * stamped vector into the target frame.
 * @param target_frame frame into which to transform
 * @param stamped_in stamped vector, defines source frame and time
 * @param stamped_out stamped output vector in target_frame
 * @exception ConnectivityException thrown if no connection between
 * the source and target frame could be found in the tree.
 * @exception ExtrapolationException returning a value would have
 * required extrapolation beyond current limits.
 * @exception LookupException at least one of the two given frames is
 * unknown
 */
void 
Transformer::transform_vector(const std::string& target_frame,
                              const Stamped<Vector3>& stamped_in,
                              Stamped<Vector3>& stamped_out) const
{
  StampedTransform transform;
  lookup_transform(target_frame, stamped_in.frame_id,
                   stamped_in.stamp, transform);

  // may not be most efficient
  Vector3 end    = stamped_in;
  Vector3 origin = Vector3(0,0,0);
  Vector3 output = (transform * end) - (transform * origin);
  stamped_out.set_data(output);

  stamped_out.stamp = transform.stamp;
  stamped_out.frame_id = target_frame;
}

/** Transform a stamped point into the target frame.
 * This transforms the point given relative to the frame set in the
 * stamped point into the target frame.
 * @param target_frame frame into which to transform
 * @param stamped_in stamped point, defines source frame and time
 * @param stamped_out stamped output point in target_frame
 * @exception ConnectivityException thrown if no connection between
 * the source and target frame could be found in the tree.
 * @exception ExtrapolationException returning a value would have
 * required extrapolation beyond current limits.
 * @exception LookupException at least one of the two given frames is
 * unknown
 */
void 
Transformer::transform_point(const std::string& target_frame,
                             const Stamped<Point>& stamped_in,
                             Stamped<Point>& stamped_out) const
{
  StampedTransform transform;
  lookup_transform(target_frame, stamped_in.frame_id,
                   stamped_in.stamp, transform);

  stamped_out.set_data(transform * stamped_in);
  stamped_out.stamp = transform.stamp;
  stamped_out.frame_id = target_frame;

}

/** Transform a stamped pose into the target frame.
 * This transforms the pose given relative to the frame set in the
 * stamped vector into the target frame.
 * @param target_frame frame into which to transform
 * @param stamped_in stamped pose, defines source frame and time
 * @param stamped_out stamped output pose in target_frame
 * @exception ConnectivityException thrown if no connection between
 * the source and target frame could be found in the tree.
 * @exception ExtrapolationException returning a value would have
 * required extrapolation beyond current limits.
 * @exception LookupException at least one of the two given frames is
 * unknown
 */
void 
Transformer::transform_pose(const std::string& target_frame,
                            const Stamped<Pose>& stamped_in,
                            Stamped<Pose>& stamped_out) const
{
  StampedTransform transform;
  lookup_transform(target_frame, stamped_in.frame_id,
                   stamped_in.stamp, transform);

  stamped_out.set_data(transform * stamped_in);
  stamped_out.stamp = transform.stamp;
  stamped_out.frame_id = target_frame;
}


/** Transform ident pose from one frame to another.
 * This utility method can be used to transform the ident pose,
 * i.e. the origin of one frame, into another. Note that this method
 * does not throw an exception on error, rather the success of the
 * transformation is indicated by a return value.
 *
 * For example, if the source frame is the base frame
 * (e.g. /base_link), and the target frame is the global frame
 * (e.g. /map), then the result would be the robot's global position.
 * @param source_frame frame whose origin to transform
 * @param target_frame frame in which you want to know the position of
 * the origin of the source frame
 * @param stamped_out upon successful completion (check return value)
 * stamped pose of origin of source frame in target frame
 * @param time time for when to do the transformation, by default take
 * the latest matching transformation.
 * @return true if the transform could be successfully determined,
 * false otherwise.
 */
bool
Transformer::transform_origin(const std::string& source_frame,
                              const std::string& target_frame,
                              Stamped<Pose>& stamped_out, const fawkes::Time time) const
{
  tf::Stamped<tf::Pose> ident = tf::ident(source_frame, time);
  try {
    transform_pose(target_frame, ident, stamped_out);
    return true;
  } catch (Exception &e) {
    return false;
  }
}


/** Transform a stamped Quaternion into the target frame assuming a fixed frame.
 * This transforms the quaternion relative to the frame set in the
 * stamped quaternion into the target frame.
 * This will transform the quaternion from source to target, assuming
 * that there is a fixed frame, by first finding the transform of the
 * source frame in the fixed frame, and then a transformation from the
 * fixed frame into the target frame.
 * @param target_frame frame into which to transform
 * @param target_time desired time in the target frame
 * @param stamped_in stamped quaternion, defines source frame and time
 * @param fixed_frame ID of fixed frame
 * @param stamped_out stamped output quaternion in target_frame
 * @exception ConnectivityException thrown if no connection between
 * the source and target frame could be found in the tree.
 * @exception ExtrapolationException returning a value would have
 * required extrapolation beyond current limits.
 * @exception LookupException at least one of the two given frames is
 * unknown
 * @exception InvalidArgument thrown if the Quaternion is invalid, most
 * likely an uninitialized Quaternion (0,0,0,0).
 */
void 
Transformer::transform_quaternion(const std::string& target_frame,
                                  const fawkes::Time& target_time,
                                  const Stamped<Quaternion>& stamped_in,
                                  const std::string& fixed_frame,
                                  Stamped<Quaternion>& stamped_out) const
{
  assert_quaternion_valid(stamped_in);
  StampedTransform transform;
  lookup_transform(target_frame, target_time,
                   stamped_in.frame_id, stamped_in.stamp,
                   fixed_frame, transform);

  stamped_out.set_data(transform * stamped_in);
  stamped_out.stamp = transform.stamp;
  stamped_out.frame_id = target_frame;
}

/** Transform a stamped vector into the target frame assuming a fixed frame.
 * This transforms the vector relative to the frame set in the
 * stamped quaternion into the target frame.
 * This will transform the vector from source to target, assuming
 * that there is a fixed frame, by first finding the transform of the
 * source frame in the fixed frame, and then a transformation from the
 * fixed frame into the target frame.
 * @param target_frame frame into which to transform
 * @param target_time desired time in the target frame
 * @param stamped_in stamped vector, defines source frame and time
 * @param fixed_frame ID of fixed frame
 * @param stamped_out stamped output vector in target_frame
 * @exception ConnectivityException thrown if no connection between
 * the source and target frame could be found in the tree.
 * @exception ExtrapolationException returning a value would have
 * required extrapolation beyond current limits.
 * @exception LookupException at least one of the two given frames is
 * unknown
 */
void 
Transformer::transform_vector(const std::string& target_frame,
                              const fawkes::Time& target_time,
                              const Stamped<Vector3>& stamped_in,
                              const std::string& fixed_frame,
                              Stamped<Vector3>& stamped_out) const
{
  StampedTransform transform;
  lookup_transform(target_frame, target_time,
                   stamped_in.frame_id,stamped_in.stamp,
                   fixed_frame, transform);

  // may not be most efficient
  Vector3 end = stamped_in;
  Vector3 origin(0,0,0);
  Vector3 output = (transform * end) - (transform * origin);
  stamped_out.set_data( output);

  stamped_out.stamp = transform.stamp;
  stamped_out.frame_id = target_frame;

}

/** Transform a stamped point into the target frame assuming a fixed frame.
 * This transforms the point relative to the frame set in the
 * stamped quaternion into the target frame.
 * This will transform the point from source to target, assuming
 * that there is a fixed frame, by first finding the transform of the
 * source frame in the fixed frame, and then a transformation from the
 * fixed frame into the target frame.
 * @param target_frame frame into which to transform
 * @param target_time desired time in the target frame
 * @param stamped_in stamped point, defines source frame and time
 * @param fixed_frame ID of fixed frame
 * @param stamped_out stamped output point in target_frame
 * @exception ConnectivityException thrown if no connection between
 * the source and target frame could be found in the tree.
 * @exception ExtrapolationException returning a value would have
 * required extrapolation beyond current limits.
 * @exception LookupException at least one of the two given frames is
 * unknown
 */
void 
Transformer::transform_point(const std::string& target_frame,
                             const fawkes::Time& target_time,
                             const Stamped<Point>& stamped_in,
                             const std::string& fixed_frame,
                             Stamped<Point>& stamped_out) const
{
  StampedTransform transform;
  lookup_transform(target_frame, target_time,
                   stamped_in.frame_id, stamped_in.stamp,
                   fixed_frame, transform);

  stamped_out.set_data(transform * stamped_in);
  stamped_out.stamp = transform.stamp;
  stamped_out.frame_id = target_frame;
}

/** Transform a stamped pose into the target frame assuming a fixed frame.
 * This transforms the pose relative to the frame set in the
 * stamped quaternion into the target frame.
 * This will transform the pose from source to target, assuming
 * that there is a fixed frame, by first finding the transform of the
 * source frame in the fixed frame, and then a transformation from the
 * fixed frame into the target frame.
 * @param target_frame frame into which to transform
 * @param target_time desired time in the target frame
 * @param stamped_in stamped pose, defines source frame and time
 * @param fixed_frame ID of fixed frame
 * @param stamped_out stamped output pose in target_frame
 * @exception ConnectivityException thrown if no connection between
 * the source and target frame could be found in the tree.
 * @exception ExtrapolationException returning a value would have
 * required extrapolation beyond current limits.
 * @exception LookupException at least one of the two given frames is
 * unknown
 */
void 
Transformer::transform_pose(const std::string& target_frame,
                            const fawkes::Time& target_time,
                            const Stamped<Pose>& stamped_in,
                            const std::string& fixed_frame,
                            Stamped<Pose>& stamped_out) const
{
  StampedTransform transform;
  lookup_transform(target_frame, target_time,
                   stamped_in.frame_id, stamped_in.stamp,
                   fixed_frame, transform);

  stamped_out.set_data(transform * stamped_in);
  stamped_out.stamp = transform.stamp;
  stamped_out.frame_id = target_frame;
}

} // end namespace tf
} // end namespace fawkes
