/***************************************************************************
 *  transformer.h - Fawkes tf transformer (based on ROS tf)
 *
 *  Created: Tue Oct 18 17:03:47 2011
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

#ifndef __LIBS_TF_TRANSFORMER_H_
#define __LIBS_TF_TRANSFORMER_H_

#include <tf/types.h>
#include <stdint.h>

#include <map>
#ifdef __FreeBSD__
#  include <tr1/unordered_map>
#else
#  include <unordered_map>
#endif
#include <vector>
#include <string>

namespace fawkes {

  class Mutex;

  namespace tf {
#if 0 /* just to make Emacs auto-indent happy */
  }
}
#endif

class TimeCache;

class Transformer
{
 public:
  static const unsigned int MAX_GRAPH_DEPTH = 100UL;
  static const float DEFAULT_MAX_EXTRAPOLATION_DISTANCE;

  Transformer(float cache_time_sec = 10.0);
  virtual ~Transformer(void);

  void clear();

  bool set_transform(const StampedTransform &transform,
                     const std::string &authority = "default_authority");

  bool frame_exists(const std::string& frame_id_str) const;


  void lookup_transform(const std::string& target_frame,
                        const std::string& source_frame,
                        const fawkes::Time& time,
                        StampedTransform& transform) const;

  void lookup_transform(const std::string& target_frame,
                        const std::string& source_frame,
                        StampedTransform& transform) const;

  void lookup_transform(const std::string& target_frame,
                        const fawkes::Time& target_time,
                        const std::string& source_frame,
                        const fawkes::Time& source_time,
                        const std::string& fixed_frame,
                        StampedTransform& transform) const;

  bool can_transform(const std::string& target_frame,
                     const std::string& source_frame,
                     const fawkes::Time& time) const;

  bool can_transform(const std::string& target_frame,
                     const fawkes::Time& target_time,
                     const std::string& source_frame,
                     const fawkes::Time& source_time,
                     const std::string& fixed_frame) const;

  const TimeCache *  get_frame_cache(const std::string &frame_id) const;

  void set_enabled(bool enabled);
  bool is_enabled() const { return enabled_; };

  int get_latest_common_time(const std::string &source_frame, const std::string &target_frame,
                             fawkes::Time& time, std::string* error_string = 0) const;

  void transform_quaternion(const std::string& target_frame,
                            const Stamped<Quaternion>& stamped_in,
                            Stamped<Quaternion>& stamped_out) const;
  void transform_vector(const std::string& target_frame,
                        const Stamped<Vector3>& stamped_in,
                        Stamped<Vector3>& stamped_out) const;
  void transform_point(const std::string& target_frame,
                       const Stamped<Point>& stamped_in, Stamped<Point>& stamped_out) const;
  void transform_pose(const std::string& target_frame,
                      const Stamped<Pose>& stamped_in, Stamped<Pose>& stamped_out) const;

  bool transform_origin(const std::string& source_frame,
                        const std::string& target_frame,
                        Stamped<Pose>& stamped_out,
                        const fawkes::Time time = fawkes::Time(0,0)) const;

  void transform_quaternion(const std::string& target_frame, const fawkes::Time& target_time,
                            const Stamped<Quaternion>& stamped_in,
                            const std::string& fixed_frame,
                            Stamped<Quaternion>& stamped_out) const;
  void transform_vector(const std::string& target_frame, const fawkes::Time& target_time,
                       const Stamped<Vector3>& stamped_in,
                       const std::string& fixed_frame,
                       Stamped<Vector3>& stamped_out) const;
  void transform_point(const std::string& target_frame, const fawkes::Time& target_time,
                      const Stamped<Point>& stamped_in,
                      const std::string& fixed_frame,
                      Stamped<Point>& stamped_out) const;
  void transform_pose(const std::string& target_frame, const fawkes::Time& target_time,
                     const Stamped<Pose>& stamped_in,
                     const std::string& fixed_frame,
                     Stamped<Pose>& stamped_out) const;

 protected: /* methods */
  TimeCache *  get_frame(unsigned int frame_number) const;

  CompactFrameID lookup_frame_number(const std::string &frameid_str) const;
  CompactFrameID lookup_or_insert_frame_number(const std::string &frameid_str);
  std::string    lookup_frame_string(unsigned int frame_id_num) const;

 protected:
  /// Flag to mark the transformer as disabled
  bool enabled_;
  /// Map from string frame ids to CompactFrameID.
#ifdef __FreeBSD__
  typedef std::tr1::unordered_map<std::string, CompactFrameID> M_StringToCompactFrameID;
#else
  typedef std::unordered_map<std::string, CompactFrameID> M_StringToCompactFrameID;
#endif
  /// Map from frame IDs to frame numbers
  M_StringToCompactFrameID frameIDs_;
  /// Map from CompactFrameID frame_id_numbers to string for debugging and output.
  std::vector<std::string> frameIDs_reverse;
  /// Map to lookup the most recent authority for a given frame.
  std::map<CompactFrameID, std::string> frame_authority_;

  /** \brief The pointers to potential frames that the tree can be made of.
   * The frames will be dynamically allocated at run time when set the first time. */
  std::vector<TimeCache*> frames_;

  /** \brief A mutex to protect testing and allocating new frames on the above vector. */
  mutable Mutex *frame_mutex_;

  /// How long to cache transform history
  float cache_time_;

  /// whether or not to allow extrapolation
  float max_extrapolation_distance_;

  /// transform prefix to apply as necessary
  std::string tf_prefix_;

  /// Set to true to allow falling back to wall time
  bool fall_back_to_wall_time_;

 private:
  /**Return the latest time which is common across the spanning set.
   * @return zero if fails to cross */
  int get_latest_common_time(CompactFrameID target_frame, CompactFrameID source_frame,
                             fawkes::Time& time, std::string* error_string) const;

  bool can_transform_no_lock(CompactFrameID target_id, CompactFrameID source_id,
                             const fawkes::Time& time) const;
  void create_connectivity_error_string(CompactFrameID source_frame, CompactFrameID target_frame, std::string* out) const;

  template<typename F>
  int walk_to_top_parent(F& f, fawkes::Time time,
                         CompactFrameID target_id, CompactFrameID source_id,
                         std::string* error_string) const;

};


} // end namespace tf
} // end namespace fawkes

#endif
