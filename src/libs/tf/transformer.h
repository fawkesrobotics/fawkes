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
#include <unordered_map>
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
  /** The maximum number of time to recurse before assuming the tree
   *  has a loop. */
  static const unsigned int MAX_GRAPH_DEPTH = 100UL;
  static const float DEFAULT_MAX_EXTRAPOLATION_DISTANCE;

  Transformer(bool interpolating = true,
              float cache_time_sec = 10.0);
  virtual ~Transformer(void);

  /** Clear all data. */
  void clear();

  /** Add transform information to the tf data structure.
   * @param transform The transform to store
   * @param authority The source of the information for this transform
   * returns true unless an error occured
   */
  bool set_transform(const StampedTransform &transform,
                     const std::string &authority = "default_authority");

  /**@brief Check if a frame exists in the tree
   * @param frame_id_str The frame id in question  */
  bool frame_exists(const std::string& frame_id_str) const;


  void lookup_transform(const std::string& target_frame,
                        const std::string& source_frame,
                        const fawkes::Time& time,
                        StampedTransform& transform) const;

  void lookup_transform(const std::string& target_frame,
                        const std::string& source_frame,
                        StampedTransform& transform) const;

  const TimeCache *  get_frame_cache(const std::string &frame_id) const;

 protected: /* methods */
  TimeCache *  getFrame(unsigned int frame_number) const;

  CompactFrameID lookupFrameNumber(const std::string &frameid_str) const;
  CompactFrameID lookupOrInsertFrameNumber(const std::string &frameid_str);

  int getLatestCommonTime(const std::string &source_frame, const std::string &target_frame,
                          fawkes::Time& time, std::string* error_string) const;

 protected:
  /// Map from string frame ids to CompactFrameID.
  typedef std::unordered_map<std::string, CompactFrameID> M_StringToCompactFrameID;
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
  float cache_time;

  /// whether or not to interpolate or extrapolate
  bool interpolating;

  /// whether or not to allow extrapolation
  float max_extrapolation_distance_;

  /// transform prefix to apply as necessary
  std::string tf_prefix_;

  /// Set to true to allow falling back to wall time
  bool fall_back_to_wall_time_;

 private:
  /**@brief Return the latest rostime which is common across the spanning set
   * zero if fails to cross */
  int getLatestCommonTime(CompactFrameID target_frame, CompactFrameID source_frame,
                          fawkes::Time& time, std::string* error_string) const;

  template<typename F>
    int walkToTopParent(F& f, fawkes::Time time,
                        CompactFrameID target_id, CompactFrameID source_id,
                        std::string* error_string) const;

};


} // end namespace tf
} // end namespace fawkes

#endif
