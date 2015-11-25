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

#include <tf/buffer_core.h>
#include <tf/types.h>

namespace fawkes {
  namespace tf {
#if 0 /* just to make Emacs auto-indent happy */
  }
}
#endif

class TimeCacheInterface;
typedef std::shared_ptr<TimeCacheInterface> TimeCacheInterfacePtr;

class Transformer : public BufferCore
{
 public:
	Transformer(float cache_time_sec = BufferCore::DEFAULT_CACHE_TIME);
  virtual ~Transformer(void);

  void set_enabled(bool enabled);
  bool is_enabled() const;
  
  float get_cache_time() const;
  void lock();
  bool try_lock();
  void unlock();

  bool frame_exists(const std::string& frame_id_str) const;
  TimeCacheInterfacePtr get_frame_cache(const std::string& frame_id) const;
  std::vector<TimeCacheInterfacePtr> get_frame_caches() const;
  std::vector<std::string> get_frame_id_mappings() const;
  
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
 
  void lookup_transform(const std::string& target_frame,
                        const std::string& source_frame,
                        StampedTransform& transform) const;
  
  bool can_transform(const std::string& target_frame, const std::string& source_frame,
                     const fawkes::Time& time, std::string* error_msg = NULL) const;
  
  bool can_transform(const std::string& target_frame, const fawkes::Time& target_time,
                     const std::string& source_frame, const fawkes::Time& source_time,
                     const std::string& fixed_frame, std::string* error_msg = NULL) const;

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

  std::string all_frames_as_dot(bool print_time, fawkes::Time *time = 0) const;

 private:
  bool enabled_;
};


} // end namespace tf
} // end namespace fawkes

#endif
