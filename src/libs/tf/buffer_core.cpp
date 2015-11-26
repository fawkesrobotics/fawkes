/***************************************************************************
 *  buffer_core.h - Fawkes tf buffer core (based on ROS tf2)
 *
 *  Created: Mon Oct 26 11:02:22 2015
 *  Copyright  2015  Tim Niemueller [www.niemueller.de]
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

/* This code is based on ROS tf2 with the following copyright and license:
 *
 * Copyright (c) 2010, Willow Garage, Inc.
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

#include <tf/buffer_core.h>
#include <tf/time_cache.h>
#include <tf/exceptions.h>
#include <tf/types.h>
#include <tf/utils.h>

#include <sstream>
#include <algorithm>

namespace fawkes {
  namespace tf {
#if 0 /* just to make Emacs auto-indent happy */
  }
}
#endif

/// @cond INTERNAL
typedef std::pair<fawkes::Time, CompactFrameID> P_TimeAndFrameID;
/// @endcond


/** Warn if an illegal frame_id was passed.
 * @param function_name_arg informative message content
 * @param frame_id frame ID to check
 * @return true if we have something to warn about, false otherwise
 */
bool
BufferCore::warn_frame_id(const char* function_name_arg, const std::string& frame_id) const
{
  if (frame_id.size() == 0) {
	  printf("Invalid argument passed to %s in tf2 frame_ids cannot be empty", function_name_arg);
    return true;
  }

  if (starts_with_slash(frame_id))
  {
	  printf("Invalid argument '%s' passed to %s in tf2 frame_ids cannot start with a '/'",
	         frame_id.c_str(), function_name_arg);
    return true;
  }

  return false;
}

/** Check if frame ID is valid and return compact ID.
 * @param function_name_arg informational for error message
 * @param frame_id frame ID to validate
 * @return compact frame ID, if frame is valid
 * @throw InvalidArgumentException thrown if argument s invalid,
 * i.e. if it is empty or starts with a slash.
 * @throw LookupException thrown if frame does not exist
 */
CompactFrameID
BufferCore::validate_frame_id(const char* function_name_arg, const std::string& frame_id) const
{
  if (frame_id.empty())
  {
    throw InvalidArgumentException("Invalid argument passed to %s in tf2 frame_ids cannot be empty",
                                   function_name_arg);
  }

  if (starts_with_slash(frame_id))
  {
    throw InvalidArgumentException("Invalid argument \"%s\" passed to %s. "
                                   "In tf2 frame_ids cannot start with a '/'",
                                   frame_id.c_str(), function_name_arg);
  }

  CompactFrameID id = lookup_frame_number(frame_id);
  if (id == 0)
  {
	  throw LookupException("Frame ID \"%s\" passed to %s does not exist.",
	                        frame_id.c_str(), function_name_arg);
  }
  
  return id;
}

/** Constructor
 * @param cache_time How long to keep a history of transforms in nanoseconds
 */
BufferCore::BufferCore(float cache_time)
	: cache_time_(cache_time)
{
  frameIDs_["NO_PARENT"] = 0;
  frames_.push_back(TimeCacheInterfacePtr());
  frameIDs_reverse.push_back("NO_PARENT");
}

BufferCore::~BufferCore()
{
}

/** Clear all data. */
void
BufferCore::clear()
{
  //old_tf_.clear();
  std::unique_lock<std::mutex> lock(frame_mutex_);
  if ( frames_.size() > 1 )
  {
    for (std::vector<TimeCacheInterfacePtr>::iterator cache_it = frames_.begin() + 1; cache_it != frames_.end(); ++cache_it)
    {
      if (*cache_it)
        (*cache_it)->clear_list();
    }
  }
  
}

/** Add transform information to the tf data structure
 * @param transform_in The transform to store
 * @param authority The source of the information for this transform
 * @param is_static Record this transform as a static transform.  It
 * will be good across all time.  (This cannot be changed after the
 * first call.)
 * @return true unless an error occured
 */
bool
BufferCore::set_transform(const StampedTransform &transform_in,
                          const std::string& authority, bool is_static)
{
  StampedTransform stripped = transform_in;
  stripped.frame_id = strip_slash(stripped.frame_id);
  stripped.child_frame_id = strip_slash(stripped.child_frame_id);

  bool error_exists = false;
  if (stripped.child_frame_id == stripped.frame_id)
  {
	  printf("TF_SELF_TRANSFORM: Ignoring transform from authority \"%s\" with frame_id and child_frame_id  \"%s\" because they are the same",  authority.c_str(), stripped.child_frame_id.c_str());
    error_exists = true;
  }

  if (stripped.child_frame_id == "")
  {
	  printf("TF_NO_CHILD_FRAME_ID: Ignoring transform from authority \"%s\" because child_frame_id not set ", authority.c_str());
    error_exists = true;
  }

  if (stripped.frame_id == "")
  {
    printf("TF_NO_FRAME_ID: Ignoring transform with child_frame_id \"%s\"  from authority \"%s\" because frame_id not set", stripped.child_frame_id.c_str(), authority.c_str());
    error_exists = true;
  }

  if (std::isnan(stripped.getOrigin().x()) ||
      std::isnan(stripped.getOrigin().y()) ||
      std::isnan(stripped.getOrigin().z()) ||
      std::isnan(stripped.getRotation().x()) ||
      std::isnan(stripped.getRotation().y()) ||
      std::isnan(stripped.getRotation().z()) ||
      std::isnan(stripped.getRotation().w()))
  {
	  printf("TF_NAN_INPUT: Ignoring transform for child_frame_id \"%s\" "
	         "from authority \"%s\" because of a nan value in the transform (%f %f %f) (%f %f %f %f)",
	         stripped.child_frame_id.c_str(), authority.c_str(),
	         stripped.getOrigin().x(), stripped.getOrigin().y(), stripped.getOrigin().z(),
	         stripped.getRotation().x(), stripped.getRotation().y(),
	         stripped.getRotation().z(), stripped.getRotation().w());
    error_exists = true;
  }

  if (error_exists)
	  return false;
  
  {
    std::unique_lock<std::mutex> lock(frame_mutex_);
    CompactFrameID frame_number = lookup_or_insert_frame_number(stripped.child_frame_id);
    TimeCacheInterfacePtr frame = get_frame(frame_number);
    if (! frame)
      frame = allocate_frame(frame_number, is_static);

    if (frame->insert_data(TransformStorage(stripped, lookup_or_insert_frame_number(stripped.frame_id), frame_number)))
    {
      frame_authority_[frame_number] = authority;
    }
    else
    {
      printf("TF_OLD_DATA ignoring data from the past for frame %s "
             "at time %g according to authority %s\n"
             "Possible reasons are listed at http://wiki.ros.org/tf/Errors%%20explained",
             stripped.child_frame_id.c_str(), stripped.stamp.in_sec(), authority.c_str());
      return false;
    }
  }

  //test_transformable_requests();

  return true;
}

/** Allocate a new frame cache.
 * @param cfid frame ID for which to create the frame cache
 * @param is_static true if the transforms for this frame are static, false otherwise
 * @return pointer to new cache
 */
TimeCacheInterfacePtr
BufferCore::allocate_frame(CompactFrameID cfid, bool is_static)
{
  TimeCacheInterfacePtr frame_ptr = frames_[cfid];
  if (is_static) {
    frames_[cfid] = TimeCacheInterfacePtr(new StaticCache());
  } else {
    frames_[cfid] = TimeCacheInterfacePtr(new TimeCache(cache_time_));
  }

  return frames_[cfid];
}

enum WalkEnding
{
  Identity,
  TargetParentOfSource,
  SourceParentOfTarget,
  FullPath,
};

/** Traverse transform tree: walk from frame to top-parent of both.
 * @param f accumulator
 * @param time timestamp
 * @param target_id frame number of target
 * @param source_id frame number of source
 * @param error_string accumulated error string
 * @return error flag from ErrorValues
 */
template<typename F>
int BufferCore::walk_to_top_parent(F& f, fawkes::Time time,
                                   CompactFrameID target_id, CompactFrameID source_id,
                                   std::string* error_string) const
{
  return walk_to_top_parent(f, time, target_id, source_id, error_string, NULL);
}

/** Traverse transform tree: walk from frame to top-parent of both.
 * @param f accumulator
 * @param time timestamp
 * @param target_id frame number of target
 * @param source_id frame number of source
 * @param error_string accumulated error string
 * @param frame_chain If frame_chain is not NULL, store the traversed
 * frame tree in vector frame_chain.
 * @return error flag from ErrorValues
 */
template<typename F>
int BufferCore::walk_to_top_parent(F& f, fawkes::Time time, CompactFrameID target_id,
                                   CompactFrameID source_id, std::string* error_string,
                                   std::vector<CompactFrameID> *frame_chain) const
{
  if (frame_chain)
    frame_chain->clear();

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

  std::string extrapolation_error_string;
  bool extrapolation_might_have_occurred = false;

  while (frame != 0)
  {
    TimeCacheInterfacePtr cache = get_frame(frame);
    if (frame_chain)
      frame_chain->push_back(frame);

    if (!cache)
    {
      // There will be no cache for the very root of the tree
      top_parent = frame;
      break;
    }

    CompactFrameID parent = f.gather(cache, time, &extrapolation_error_string);
    if (parent == 0)
    {
      // Just break out here... there may still be a path from source -> target
      top_parent = frame;
      extrapolation_might_have_occurred = true;
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
        ss << "The tf tree is invalid because it contains a loop." << std::endl
           << all_frames_as_string_no_lock() << std::endl;
        *error_string = ss.str();
      }
      return LOOKUP_ERROR;
    }
  }

  // Now walk to the top parent from the target frame, accumulating its transform
  frame = target_id;
  depth = 0;
  std::vector<CompactFrameID> reverse_frame_chain;

  while (frame != top_parent)
  {
    TimeCacheInterfacePtr cache = get_frame(frame);
    if (frame_chain)
      reverse_frame_chain.push_back(frame);

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
        ss << *error_string << ", when looking up transform from frame [" << lookup_frame_string(source_id) << "] to frame [" << lookup_frame_string(target_id) << "]";
        *error_string = ss.str();
      }

      return EXTRAPOLATION_ERROR;
    }

    // Early out... source frame is a direct parent of the target frame
    if (frame == source_id)
    {
      f.finalize(SourceParentOfTarget, time);
      if (frame_chain)
      {
        frame_chain->swap(reverse_frame_chain);
      }
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
        ss << "The tf tree is invalid because it contains a loop." << std::endl
           << all_frames_as_string_no_lock() << std::endl;
        *error_string = ss.str();
      }
      return LOOKUP_ERROR;
    }
  }

  if (frame != top_parent)
  {
    if (extrapolation_might_have_occurred)
    {
      if (error_string)
      {
        std::stringstream ss;
        ss << extrapolation_error_string << ", when looking up transform from frame [" << lookup_frame_string(source_id) << "] to frame [" << lookup_frame_string(target_id) << "]";
        *error_string = ss.str();
      }

      return EXTRAPOLATION_ERROR;
      
    }

    create_connectivity_error_string(source_id, target_id, error_string);
    return CONNECTIVITY_ERROR;
  }

  f.finalize(FullPath, time);
  if (frame_chain)
  {
    // Pruning: Compare the chains starting at the parent (end) until they differ
    ssize_t m = reverse_frame_chain.size()-1;
    ssize_t n = frame_chain->size()-1;
    for (; m >= 0 && n >= 0; --m, --n)
    {
      if ((*frame_chain)[n] != reverse_frame_chain[m])
        break;
    }
    // Erase all duplicate items from frame_chain
    if (n > 0)
      frame_chain->erase(frame_chain->begin() + (n-1), frame_chain->end());

    if (m < (ssize_t)reverse_frame_chain.size())
    {
      for (int i = m; i >= 0; --i)
      {
        frame_chain->push_back(reverse_frame_chain[i]);
      }
    }
  }
  
  return NO_ERROR;
}


/// @cond INTERNAL
struct TransformAccum
{
  TransformAccum()
  : source_to_top_quat(0.0, 0.0, 0.0, 1.0)
  , source_to_top_vec(0.0, 0.0, 0.0)
  , target_to_top_quat(0.0, 0.0, 0.0, 1.0)
  , target_to_top_vec(0.0, 0.0, 0.0)
  , result_quat(0.0, 0.0, 0.0, 1.0)
  , result_vec(0.0, 0.0, 0.0)
  {
  }

  CompactFrameID gather(TimeCacheInterfacePtr cache, fawkes::Time time, std::string* error_string)
  {
    if (!cache->get_data(time, st, error_string))
    {
      return 0;
    }

    return st.frame_id;
  }

  void accum(bool source)
  {
    if (source)
    {
      source_to_top_vec = quatRotate(st.rotation, source_to_top_vec) + st.translation;
      source_to_top_quat = st.rotation * source_to_top_quat;
    }
    else
    {
      target_to_top_vec = quatRotate(st.rotation, target_to_top_vec) + st.translation;
      target_to_top_quat = st.rotation * target_to_top_quat;
    }
  }

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
        Quaternion inv_target_quat = target_to_top_quat.inverse();
        Vector3 inv_target_vec = quatRotate(inv_target_quat, -target_to_top_vec);
        result_vec = inv_target_vec;
        result_quat = inv_target_quat;
        break;
      }
    case FullPath:
      {
        Quaternion inv_target_quat = target_to_top_quat.inverse();
        Vector3 inv_target_vec = quatRotate(inv_target_quat, -target_to_top_vec);

     	result_vec = quatRotate(inv_target_quat, source_to_top_vec) + inv_target_vec;
        result_quat = inv_target_quat * source_to_top_quat;
      }
      break;
    };

    time = _time;
  }

  TransformStorage st;
  fawkes::Time time;
  Quaternion source_to_top_quat;
  Vector3 source_to_top_vec;
  Quaternion target_to_top_quat;
  Vector3 target_to_top_vec;

  Quaternion result_quat;
  Vector3 result_vec;
};
///@endcond

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
void
BufferCore::lookup_transform(const std::string& target_frame,
                             const std::string& source_frame,
                             const fawkes::Time& time,
                             StampedTransform& transform) const
{
	std::unique_lock<std::mutex> lock(frame_mutex_);

	if (target_frame == source_frame) {
		transform.setIdentity();
		transform.frame_id = target_frame;
		transform.child_frame_id = source_frame;

		if (time == fawkes::Time(0,0)) {
			CompactFrameID target_id = lookup_frame_number(target_frame);
			TimeCacheInterfacePtr cache = get_frame(target_id);
			if (cache)
				transform.stamp = cache->get_latest_timestamp();
			else
				transform.stamp = time;
		} else {
			transform.stamp = time;
		}
	}

	//Identify case does not need to be validated above
	CompactFrameID target_id = validate_frame_id("lookup_transform argument target_frame", target_frame);
	CompactFrameID source_id = validate_frame_id("lookup_transform argument source_frame", source_frame);

	std::string error_string;
	TransformAccum accum;
	int retval = walk_to_top_parent(accum, time, target_id, source_id, &error_string);
	if (retval != NO_ERROR)
	{
    switch (retval)
    {
    case CONNECTIVITY_ERROR:
	    throw ConnectivityException("%s", error_string.c_str());
    case EXTRAPOLATION_ERROR:
	    throw ExtrapolationException("%s", error_string.c_str());
    case LOOKUP_ERROR:
	    throw LookupException("%s", error_string.c_str());
    default:
	    //logError("Unknown error code: %d", retval);
	    throw TransformException();
    }
  }

  transform.setOrigin(accum.result_vec);
  transform.setRotation(accum.result_quat);
  transform.child_frame_id = source_frame;
  transform.frame_id       = target_frame;
  transform.stamp          = accum.time;
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
BufferCore::lookup_transform(const std::string& target_frame, const fawkes::Time& target_time,
                             const std::string& source_frame, const fawkes::Time& source_time,
                             const std::string& fixed_frame, StampedTransform& transform) const
{
  validate_frame_id("lookup_transform argument target_frame", target_frame);
  validate_frame_id("lookup_transform argument source_frame", source_frame);
  validate_frame_id("lookup_transform argument fixed_frame", fixed_frame);

  StampedTransform temp1, temp2;
  lookup_transform(fixed_frame, source_frame, source_time, temp1);
  lookup_transform(target_frame, fixed_frame, target_time, temp2);
  transform.set_data(temp2 * temp1);
  transform.stamp = temp2.stamp;
  transform.frame_id = target_frame;
  transform.child_frame_id = source_frame;
}

/// @cond INTERNAL
struct CanTransformAccum
{
  CompactFrameID gather(TimeCacheInterfacePtr cache, fawkes::Time time, std::string* error_string)
  {
    return cache->get_parent(time, error_string);
  }

  void accum(bool source)
  {
  }

  void finalize(WalkEnding end, fawkes::Time _time)
  {
  }

  TransformStorage st;
};
/// @endcond

/** Test if a transform is possible.
 * Internal check that does not lock the frame mutex.
 * @param target_id The frame number into which to transform
 * @param source_id The frame number from which to transform
 * @param time The time at which to transform
 * @param error_msg passed to walk_to_top_parent
 * @return true if the transformation can be calculated, false otherwise
 */
bool
BufferCore::can_transform_no_lock(CompactFrameID target_id, CompactFrameID source_id,
                                  const fawkes::Time& time, std::string* error_msg) const
{
  if (target_id == 0 || source_id == 0)
  {
    return false;
  }

  if (target_id == source_id)
  {
    return true;
  }

  CanTransformAccum accum;
  if (walk_to_top_parent(accum, time, target_id, source_id, error_msg) == NO_ERROR)
  {
    return true;
  }

  return false;
}

/** Test if a transform is possible.
 * Internal check that does lock the frame mutex.
 * @param target_id The frame number into which to transform
 * @param source_id The frame number from which to transform
 * @param time The time at which to transform
 * @param error_msg passed to walk_to_top_parent
 * @return true if the transformation can be calculated, false otherwise
 */
bool
BufferCore::can_transform_internal(CompactFrameID target_id, CompactFrameID source_id,
                                   const fawkes::Time& time, std::string* error_msg) const
{
  std::unique_lock<std::mutex> lock(frame_mutex_);
  return can_transform_no_lock(target_id, source_id, time, error_msg);
}

/** Test if a transform is possible
 * @param target_frame The frame into which to transform
 * @param source_frame The frame from which to transform
 * @param time The time at which to transform
 * @param error_msg A pointer to a string which will be filled with why the transform failed, if not NULL
 * @return True if the transform is possible, false otherwise 
 */
bool
BufferCore::can_transform(const std::string& target_frame, const std::string& source_frame,
                          const fawkes::Time& time, std::string* error_msg) const
{
  // Short circuit if target_frame == source_frame
	if (target_frame == source_frame)
		return true;

	if (warn_frame_id("canTransform argument target_frame", target_frame))
		return false;
	if (warn_frame_id("canTransform argument source_frame", source_frame))
    return false;

  std::unique_lock<std::mutex> lock(frame_mutex_);

  CompactFrameID target_id = lookup_frame_number(target_frame);
  CompactFrameID source_id = lookup_frame_number(source_frame);

  return can_transform_no_lock(target_id, source_id, time, error_msg);
}

/** Test if a transform is possible.
 * @param target_frame The frame into which to transform
 * @param target_time The time into which to transform
 * @param source_frame The frame from which to transform
 * @param source_time The time from which to transform
 * @param fixed_frame The frame in which to treat the transform as constant in time
 * @param error_msg A pointer to a string which will be filled with why the transform failed, if not NULL
 * @return true if the transform is possible, false otherwise 
 */
bool
BufferCore::can_transform(const std::string& target_frame, const fawkes::Time& target_time,
                          const std::string& source_frame, const fawkes::Time& source_time,
                          const std::string& fixed_frame, std::string* error_msg) const
{
  if (warn_frame_id("canTransform argument target_frame", target_frame))
    return false;
  if (warn_frame_id("canTransform argument source_frame", source_frame))
    return false;
  if (warn_frame_id("canTransform argument fixed_frame", fixed_frame))
    return false;

  return
	  can_transform(target_frame, fixed_frame, target_time) &&
	  can_transform(fixed_frame, source_frame, source_time, error_msg);
}


/** Accessor to get frame cache.
 * This is an internal function which will get the pointer to the
 * frame associated with the frame id
 * @param frame_id The frameID of the desired Reference Frame
 * @return shared pointer to time cache
 */
TimeCacheInterfacePtr
BufferCore::get_frame(CompactFrameID frame_id) const
{
  if (frame_id >= frames_.size())
    return TimeCacheInterfacePtr();
  else
  {
    return frames_[frame_id];
  }
}

/** Get compact ID for frame.
 * @param frameid_str frame ID string
 * @return compact frame ID, zero if frame unknown
 */
CompactFrameID
BufferCore::lookup_frame_number(const std::string& frameid_str) const
{
  CompactFrameID retval;
  M_StringToCompactFrameID::const_iterator map_it = frameIDs_.find(frameid_str);
  if (map_it == frameIDs_.end())
  {
    retval = CompactFrameID(0);
  }
  else
    retval = map_it->second;
  return retval;
}

/** Get compact ID for frame or create if not existant.
 * @param frameid_str frame ID string
 * @return compact frame ID
 */
CompactFrameID
BufferCore::lookup_or_insert_frame_number(const std::string& frameid_str)
{
  CompactFrameID retval = 0;
  M_StringToCompactFrameID::iterator map_it = frameIDs_.find(frameid_str);
  if (map_it == frameIDs_.end())
  {
    retval = CompactFrameID(frames_.size());
    frames_.push_back(TimeCacheInterfacePtr());//Just a place holder for iteration
    frameIDs_[frameid_str] = retval;
    frameIDs_reverse.push_back(frameid_str);
  }
  else
    retval = frameIDs_[frameid_str];

  return retval;
}

/** Get frame string for compact frame ID.
 * @param frame_id_num compact frame ID
 * @return string for frame ID
 * @throw LookupException thrown if compact frame ID unknown
 */
const std::string&
BufferCore::lookup_frame_string(CompactFrameID frame_id_num) const
{
	if (frame_id_num >= frameIDs_reverse.size())
	{
		throw LookupException("Reverse lookup of frame id %u failed!", frame_id_num);
	}
	else
		return frameIDs_reverse[frame_id_num];
}

/** Create error string.
 * @param source_frame compact ID of source frame
 * @param target_frame compact ID of target frame
 * @param out upon return contains error string if non-NULL
 */
void
BufferCore::create_connectivity_error_string(CompactFrameID source_frame,
                                             CompactFrameID target_frame, std::string* out) const
{
  if (!out)
  {
    return;
  }
  *out = std::string("Could not find a connection between '"+lookup_frame_string(target_frame)+"' and '"+
                     lookup_frame_string(source_frame)+"' because they are not part of the same tree."+
                     "Tf has two or more unconnected trees.");
}
/// @endcond

/** A way to see what frames have been cached.
 * Useful for debugging
 * @return all current frames as a single string
 */
std::string
BufferCore::all_frames_as_string() const
{
  std::unique_lock<std::mutex> lock(frame_mutex_);
  return this->all_frames_as_string_no_lock();
}

/** A way to see what frames have been cached.
 * Useful for debugging. Use this call internally. 
 * @return all current frames as a single string
 */
std::string
BufferCore::all_frames_as_string_no_lock() const
{
  std::stringstream mstream;

  TransformStorage temp;

  //  for (std::vector< TimeCache*>::iterator  it = frames_.begin(); it != frames_.end(); ++it)

  ///regular transforms
  for (unsigned int counter = 1; counter < frames_.size(); counter ++)
  {
    TimeCacheInterfacePtr frame_ptr = get_frame(CompactFrameID(counter));
    if (frame_ptr == NULL)
      continue;
    CompactFrameID frame_id_num;
    if(  frame_ptr->get_data(fawkes::Time(0,0), temp))
      frame_id_num = temp.frame_id;
    else
    {
      frame_id_num = 0;
    }
    mstream << "Frame "<< frameIDs_reverse[counter] << " exists with parent " << frameIDs_reverse[frame_id_num] << "." <<std::endl;
  }

  return mstream.str();
}

/// @cond INTERNAL
struct TimeAndFrameIDFrameComparator
{
  TimeAndFrameIDFrameComparator(CompactFrameID id)
  : id(id)
  {}

  bool operator()(const P_TimeAndFrameID& rhs) const
  {
    return rhs.second == id;
  }

  CompactFrameID id;
};
/// @endcond


/** Get latest common time of two frames.
 * @param target_id target frame number
 * @param source_id source frame number
 * @param time upon return contains latest common timestamp
 * @param error_string upon error contains accumulated error message.
 * @return value from ErrorValues
 */
int
BufferCore::get_latest_common_time(CompactFrameID target_id, CompactFrameID source_id,
                                   fawkes::Time & time, std::string * error_string) const
{
  // Error if one of the frames don't exist.
  if (source_id == 0 || target_id == 0) return LOOKUP_ERROR;

  if (source_id == target_id)
  {
    TimeCacheInterfacePtr cache = get_frame(source_id);
    //Set time to latest timestamp of frameid in case of target and source frame id are the same
    if (cache)
      time = cache->get_latest_timestamp();
    else
	    time = fawkes::Time(0,0);
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
    TimeCacheInterfacePtr cache = get_frame(frame);

    if (!cache)
    {
      // There will be no cache for the very root of the tree
      break;
    }

    P_TimeAndFrameID latest = cache->get_latest_time_and_parent();

    if (latest.second == 0)
    {
      // Just break out here... there may still be a path from source -> target
      break;
    }

    if (!latest.first.is_zero())
    {
      common_time = std::min(latest.first, common_time);
    }

    lct_cache.push_back(latest);

    frame = latest.second;

    // Early out... target frame is a direct parent of the source frame
    if (frame == target_id)
    {
      time = common_time;
      if (time == fawkes::TIME_MAX)
      {
        time = fawkes::Time(0,0);
      }
      return NO_ERROR;
    }

    ++depth;
    if (depth > MAX_GRAPH_DEPTH)
    {
      if (error_string)
      {
        std::stringstream ss;
        ss<<"The tf tree is invalid because it contains a loop." << std::endl
          << all_frames_as_string_no_lock() << std::endl;
        *error_string = ss.str();
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
    TimeCacheInterfacePtr cache = get_frame(frame);

    if (!cache)
    {
      break;
    }

    P_TimeAndFrameID latest = cache->get_latest_time_and_parent();

    if (latest.second == 0)
    {
      break;
    }

    if (!latest.first.is_zero())
    {
      common_time = std::min(latest.first, common_time);
    }

    std::vector<P_TimeAndFrameID>::iterator it =
	    std::find_if(lct_cache.begin(), lct_cache.end(),
    /* Nice version, but requires rather recent compiler
	                 [&latest](const P_TimeAndFrameID& rhs){
		                 return rhs.second == latest.second;
	                 });
    */
                         TimeAndFrameIDFrameComparator(latest.second));



    if (it != lct_cache.end()) // found a common parent
    {
      common_parent = it->second;
      break;
    }

    frame = latest.second;

    // Early out... source frame is a direct parent of the target frame
    if (frame == source_id)
    {
      time = common_time;
      if (time == fawkes::TIME_MAX)
      {
        time = fawkes::Time(0,0);
      }
      return NO_ERROR;
    }

    ++depth;
    if (depth > MAX_GRAPH_DEPTH)
    {
      if (error_string)
      {
        std::stringstream ss;
        ss<<"The tf tree is invalid because it contains a loop." << std::endl
          << all_frames_as_string_no_lock() << std::endl;
        *error_string = ss.str();
      }
      return LOOKUP_ERROR;
    }
  }

  if (common_parent == 0)
  {
    create_connectivity_error_string(source_id, target_id, error_string);
    return CONNECTIVITY_ERROR;
  }

  // Loop through the source -> root list until we hit the common parent
  {
    std::vector<P_TimeAndFrameID>::iterator it = lct_cache.begin();
    std::vector<P_TimeAndFrameID>::iterator end = lct_cache.end();
    for (; it != end; ++it)
    {
      if (!it->first.is_zero())
      {
        common_time = std::min(common_time, it->first);
      }

      if (it->second == common_parent)
      {
        break;
      }
    }
  }

  if (common_time == fawkes::TIME_MAX)
  {
    common_time = fawkes::Time(0,0);
  }

  time = common_time;
  return NO_ERROR;
}

/** A way to see what frames have been cached in yaml format
 * Useful for debugging tools.
 * @param current_time current time to compute delay
 * @return YAML string
 */
std::string
BufferCore::all_frames_as_YAML(double current_time) const
{
  std::stringstream mstream;
  std::unique_lock<std::mutex> lock(frame_mutex_);

  TransformStorage temp;

  if (frames_.size() ==1)
    mstream <<"[]";

  mstream.precision(3);
  mstream.setf(std::ios::fixed,std::ios::floatfield);

   //  for (std::vector< TimeCache*>::iterator  it = frames_.begin(); it != frames_.end(); ++it)
  for (unsigned int counter = 1; counter < frames_.size(); counter ++)//one referenced for 0 is no frame
  {
    CompactFrameID cfid = CompactFrameID(counter);
    CompactFrameID frame_id_num;
    TimeCacheInterfacePtr cache = get_frame(cfid);
    if (!cache)
    {
      continue;
    }

    if(!cache->get_data(fawkes::Time(0,0), temp))
    {
      continue;
    }

    frame_id_num = temp.frame_id;

    std::string authority = "no recorded authority";
    std::map<CompactFrameID, std::string>::const_iterator it = frame_authority_.find(cfid);
    if (it != frame_authority_.end()) {
      authority = it->second;
    }

    double rate = cache->get_list_length() / std::max((cache->get_latest_timestamp().in_sec() -
                                                       cache->get_oldest_timestamp().in_sec() ), 0.0001);

    mstream << std::fixed; //fixed point notation
    mstream.precision(3); //3 decimal places
    mstream << frameIDs_reverse[cfid] << ": " << std::endl;
    mstream << "  parent: '" << frameIDs_reverse[frame_id_num] << "'" << std::endl;
    mstream << "  broadcaster: '" << authority << "'" << std::endl;
    mstream << "  rate: " << rate << std::endl;
    mstream << "  most_recent_transform: " << (cache->get_latest_timestamp()).in_sec() << std::endl;
    mstream << "  oldest_transform: " << (cache->get_oldest_timestamp()).in_sec() << std::endl;
    if ( current_time > 0 ) {
      mstream << "  transform_delay: " << current_time - cache->get_latest_timestamp().in_sec() << std::endl;
    }
    mstream << "  buffer_length: " << (cache->get_latest_timestamp() - cache->get_oldest_timestamp()).in_sec() << std::endl;
  }

  return mstream.str();
}

/** Get latest frames as YAML.
 * @return YAML string
 */
std::string
BufferCore::all_frames_as_YAML() const
{
  return this->all_frames_as_YAML(0.0);
}


} // end namespace tf
} // end namespace fawkes
