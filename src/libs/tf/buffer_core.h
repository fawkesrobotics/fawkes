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

#ifndef __LIBS_TF_BUFFER_CORE_H_
#define __LIBS_TF_BUFFER_CORE_H_

#include <tf/types.h>
#include <tf/transform_storage.h>

#include <utils/time/time.h>

#include <string>
#include <memory>
#include <vector>
#include <mutex>
#include <map>
#include <unordered_map>

namespace fawkes {
  namespace tf {
#if 0 /* just to make Emacs auto-indent happy */
  }
}
#endif

class TimeCacheInterface;
typedef std::shared_ptr<TimeCacheInterface> TimeCacheInterfacePtr;

/// Internal error values
enum ErrorValues {
	NO_ERROR = 0, ///< No error occured.
	LOOKUP_ERROR = 1, ///< Frame ID lookup error
	CONNECTIVITY_ERROR = 2, ///< No connection between frames found
	EXTRAPOLATION_ERROR = 3, ///< Extrapolation required out of limits.
	INVALID_ARGUMENT_ERROR = 4, ///< Invalid argument
	TIMEOUT_ERROR = 5, ///< timeout
	TRANSFORM_ERROR = 6 ///< generic error
};


/** \brief A Class which provides coordinate transforms between any two frames in a system.
 *
 * This class provides a simple interface to allow recording and
 * lookup of relationships between arbitrary frames of the system.
 *
 * libTF assumes that there is a tree of coordinate frame transforms
 * which define the relationship between all coordinate frames.  For
 * example your typical robot would have a transform from global to
 * real world.  And then from base to hand, and from base to head.
 * But Base to Hand really is composed of base to shoulder to elbow to
 * wrist to hand.  libTF is designed to take care of all the
 * intermediate steps for you.
 *
 * Internal Representation
 * libTF will store frames with the parameters necessary for
 * generating the transform into that frame from it's parent and a
 * reference to the parent frame.  Frames are designated using an
 * std::string 0 is a frame without a parent (the top of a tree) The
 * positions of frames over time must be pushed in.
 *
 * All function calls which pass frame ids can potentially throw the
 * exception tf::LookupException
 */
class BufferCore
{
 public:
	static const int DEFAULT_CACHE_TIME = 10;  //!< The default amount of time to cache data in seconds
	static const uint32_t MAX_GRAPH_DEPTH = 1000UL;  //!< Maximum number of times to recurse before
																									 //! assuming the tree has a loop

	BufferCore(float cache_time = DEFAULT_CACHE_TIME);
	virtual ~BufferCore(void);

	void clear();

	bool set_transform(const StampedTransform &transform,
	                   const std::string & authority,
	                   bool is_static = false);


	/*********** Accessors *************/
	void lookup_transform(const std::string& target_frame,
                        const std::string& source_frame,
                        const fawkes::Time& time,
                        StampedTransform& transform) const;

  void lookup_transform(const std::string& target_frame,
                        const fawkes::Time& target_time,
                        const std::string& source_frame,
                        const fawkes::Time& source_time,
                        const std::string& fixed_frame,
                        StampedTransform& transform) const;
  
  bool can_transform(const std::string& target_frame, const std::string& source_frame,
                     const fawkes::Time& time, std::string* error_msg = NULL) const;
  
  bool can_transform(const std::string& target_frame, const fawkes::Time& target_time,
                     const std::string& source_frame, const fawkes::Time& source_time,
                     const std::string& fixed_frame, std::string* error_msg = NULL) const;

  std::string all_frames_as_YAML(double current_time) const;
  std::string all_frames_as_YAML() const;
  std::string all_frames_as_string() const;
  
  /** Get the duration over which this transformer will cache.
   * @return cache length in seconds */
  float get_cache_length() { return cache_time_;}

 protected:
  /** A way to see what frames have been cached.
   * Useful for debugging. Use this call internally. 
   */
  std::string all_frames_as_string_no_lock() const;  


  /******************** Internal Storage ****************/

  /// Vector data type for frame caches.
  typedef std::vector<TimeCacheInterfacePtr> V_TimeCacheInterface;
  /** The pointers to potential frames that the tree can be made of.
   * The frames will be dynamically allocated at run time when set the
   * first time. */
  V_TimeCacheInterface frames_;
  
  /** \brief A mutex to protect testing and allocating new frames on the above vector. */
  mutable std::mutex frame_mutex_;

  /** \brief A map from string frame ids to CompactFrameID */
  typedef std::unordered_map<std::string, CompactFrameID> M_StringToCompactFrameID;
  /// Mapping from frame string IDs to compact IDs.
  M_StringToCompactFrameID frameIDs_;
  /** \brief A map from CompactFrameID frame_id_numbers to string for debugging and output */
  std::vector<std::string> frameIDs_reverse;
  /** \brief A map to lookup the most recent authority for a given frame */
  std::map<CompactFrameID, std::string> frame_authority_;


  /// How long to cache transform history
  float cache_time_;

  /************************* Internal Functions ****************************/

  TimeCacheInterfacePtr get_frame(CompactFrameID c_frame_id) const;

  TimeCacheInterfacePtr allocate_frame(CompactFrameID cfid, bool is_static);


  bool warn_frame_id(const char* function_name_arg, const std::string& frame_id) const;
  CompactFrameID validate_frame_id(const char* function_name_arg, const std::string& frame_id) const;

  /// String to number for frame lookup with dynamic allocation of new frames
  CompactFrameID lookup_frame_number(const std::string& frameid_str) const;

  /// String to number for frame lookup with dynamic allocation of new frames
  CompactFrameID lookup_or_insert_frame_number(const std::string& frameid_str);

  ///Number to string frame lookup may throw LookupException if number invalid
  const std::string& lookup_frame_string(CompactFrameID frame_id_num) const;

  void create_connectivity_error_string(CompactFrameID source_frame, CompactFrameID target_frame, std::string* out) const;

  int get_latest_common_time(CompactFrameID target_frame, CompactFrameID source_frame,
                             fawkes::Time& time, std::string* error_string) const;

  template<typename F>
  int walk_to_top_parent(F& f, fawkes::Time time, CompactFrameID target_id, CompactFrameID source_id, std::string* error_string) const;

  template<typename F>
  int walk_to_top_parent(F& f, fawkes::Time time, CompactFrameID target_id, CompactFrameID source_id, std::string* error_string, std::vector<CompactFrameID> *frame_chain) const;

  bool can_transform_internal(CompactFrameID target_id, CompactFrameID source_id,
                              const fawkes::Time& time, std::string* error_msg) const;
  bool can_transform_no_lock(CompactFrameID target_id, CompactFrameID source_id,
                             const fawkes::Time& time, std::string* error_msg) const;
};

} // end namespace tf
} // end namespace fawkes

#endif
