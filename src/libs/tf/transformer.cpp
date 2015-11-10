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
#include <core/macros.h>
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
 */

/** Constructor.
 * @param cache_time time in seconds to cache incoming transforms
 */
Transformer::Transformer(float cache_time)
	: BufferCore(cache_time),
	  enabled_(true)
{
}


/** Destructor. */
Transformer::~Transformer()
{
};


/** Set enabled status of transformer.
 * @param enabled true to enable, false to disable
 */
void
Transformer::set_enabled(bool enabled)
{
	enabled_ = enabled;
}

/** Check if transformer is enabled.
 * @return true if enabled, false otherwise
 */
bool
Transformer::is_enabled() const
{
	return enabled_;
}

/** Get cache time.
 * @return time in seconds caches will date back.
 */
float
Transformer::get_cache_time() const
{
	return cache_time_;
}

/** Lock transformer.
 * No new transforms can be added and no lookups can be performed
 * while the lock is held.
 */
void
Transformer::lock()
{
  frame_mutex_.lock();
}


/** Try to acquire lock.
 * @return true if lock has been acquired, alse otherwise
 */
bool
Transformer::try_lock()
{
  return frame_mutex_.try_lock();
}


/** Unlock.
 * Release currently held lock.
 */
void
Transformer::unlock()
{
  frame_mutex_.unlock();
}


/** Check if frame exists.
 * @param frame_id_str frame ID
 * @result true if frame exists, false otherwise
 */
bool
Transformer::frame_exists(const std::string& frame_id_str) const
{
	std::unique_lock<std::mutex> lock(frame_mutex_);

	return (frameIDs_.count(frame_id_str) > 0);
}


/** Get cache for specific frame.
 * @param frame_id ID of frame
 * @return pointer to time cache for frame
 */
TimeCacheInterfacePtr
Transformer::get_frame_cache(const std::string& frame_id) const
{
	return get_frame(lookup_frame_number(frame_id));
}

/** Get all currently existing caches.
 * @return vector of pointer to time caches
 */
std::vector<TimeCacheInterfacePtr>
Transformer::get_frame_caches() const
{
	return frames_;
}


/** Get mappings from frame ID to names.
 * @return vector of mappings from frame IDs to names
 */
std::vector<std::string>
Transformer::get_frame_id_mappings() const
{
	return frameIDs_reverse;
}


/** Test if a transform is possible.
 * @param target_frame The frame into which to transform
 * @param source_frame The frame from which to transform
 * @param time The time at which to transform
 * @param error_msg A pointer to a string which will be filled with
 * why the transform failed, if not NULL
 * @return true if the transformation can be calculated, false otherwise
 */
bool
Transformer::can_transform(const std::string& target_frame,
                           const std::string& source_frame,
                           const fawkes::Time& time,
                           std::string* error_msg) const
{
	if (likely(enabled_)) {
		std::string stripped_target = strip_slash(target_frame);
		std::string stripped_source = strip_slash(source_frame);
		return BufferCore::can_transform(stripped_target, stripped_source, time, error_msg);
	}
	else return false;
}

/** Test if a transform is possible.
 * @param target_frame The frame into which to transform
 * @param target_time The time into which to transform
 * @param source_frame The frame from which to transform
 * @param source_time The time from which to transform
 * @param fixed_frame The frame in which to treat the transform as
 * constant in time
 * @param error_msg A pointer to a string which will be filled with
 * why the transform failed, if not NULL
 * @return true if the transformation can be calculated, false otherwise
 */
bool
Transformer::can_transform(const std::string& target_frame,
                           const fawkes::Time& target_time,
                           const std::string& source_frame,
                           const fawkes::Time& source_time,
                           const std::string& fixed_frame,
                           std::string* error_msg) const
{
	if (likely(enabled_)) {
		std::string stripped_target = strip_slash(target_frame);
		std::string stripped_source = strip_slash(source_frame);
		return BufferCore::can_transform(stripped_target, target_time,
		                                 stripped_source, source_time,
		                                 fixed_frame, error_msg);
	}
	else return false;
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
void
Transformer::lookup_transform(const std::string& target_frame,
                              const std::string& source_frame,
                              const fawkes::Time& time,
                              StampedTransform& transform) const
{
  if (! enabled_) {
    throw DisabledException("Transformer has been disabled");
  }

  std::string stripped_target = strip_slash(target_frame);
  std::string stripped_source = strip_slash(source_frame);

  BufferCore::lookup_transform(stripped_target, stripped_source, time, transform);
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
  if (! enabled_) {
    throw DisabledException("Transformer has been disabled");
  }
  std::string stripped_target = strip_slash(target_frame);
  std::string stripped_source = strip_slash(source_frame);
  
  BufferCore::lookup_transform(stripped_target, target_time,
                               stripped_source, source_time,
                               fixed_frame, transform);
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


/** Get DOT graph of all frames.
 * @param print_time true to add the time of the transform as graph label
 * @param time if not NULL will be assigned the time of the graph generation
 * @return string representation of DOT graph
 */
std::string
Transformer::all_frames_as_dot(bool print_time, fawkes::Time *time) const
{
	std::unique_lock<std::mutex> lock(frame_mutex_);

	fawkes::Time current_time;
	if (time)  *time = current_time;

	std::stringstream mstream;
	mstream << std::fixed; //fixed point notation
	mstream.precision(3); //3 decimal places
	mstream << "digraph { graph [fontsize=14";
	if (print_time) {
		mstream << ", label=\"\\nRecorded at time: "
		        << current_time.str() << " (" << current_time.in_sec() << ")\"";
	}
	mstream << "]; node [fontsize=12]; edge [fontsize=12]; " << std::endl;

	TransformStorage temp;

	if (frames_.size() == 1)
		mstream <<"\"no tf data received\"";

	mstream.precision(3);
	mstream.setf(std::ios::fixed, std::ios::floatfield);
    
	//  for (std::vector< TimeCache*>::iterator  it = frames_.begin(); it != frames_.end(); ++it)
	for (unsigned int cnt = 1; cnt < frames_.size(); ++cnt) //one referenced for 0 is no frame
	{
		std::shared_ptr<TimeCacheInterface> cache = get_frame(cnt);
		if (! cache) continue;

		unsigned int frame_id_num;
		if(cache->get_data(fawkes::Time(0,0), temp)) {
			frame_id_num = temp.frame_id;
		} else {
			frame_id_num = 0;
		}
		if (frame_id_num != 0) {
			std::string authority = "no recorded authority";
			std::map<unsigned int, std::string>::const_iterator it = frame_authority_.find(cnt);
			if (it != frame_authority_.end())
				authority = it->second;

			double rate = cache->get_list_length() /
				std::max((cache->get_latest_timestamp().in_sec() -
				          cache->get_oldest_timestamp().in_sec()), 0.0001);

			mstream << "\"" << frameIDs_reverse[frame_id_num] << "\"" << " -> "
			        << "\"" << frameIDs_reverse[cnt] << "\"" << "[label=\"";

			std::shared_ptr<StaticCache> static_cache =
				std::dynamic_pointer_cast<StaticCache>(cache);
			if (static_cache) {
				mstream << "Static";
			} else {
				//<< "Broadcaster: " << authority << "\\n"
				mstream << "Average rate: " << rate << " Hz\\n"
				        << "Most recent transform: "
				        << (current_time - cache->get_latest_timestamp()).in_sec() << " sec old \\n"
				        << "Buffer length: "
				        << (cache->get_latest_timestamp() - cache->get_oldest_timestamp()).in_sec()
				        << " sec\\n";
			}
			mstream <<"\"];" <<std::endl;
		}
	}

	mstream << "}";
	return mstream.str();
}

} // end namespace tf
} // end namespace fawkes
