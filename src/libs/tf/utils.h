
/***************************************************************************
 *  utils.h - Fawkes tf utils
 *
 *  Created: Fri Jun  1 14:07:00 2012
 *  Copyright  2012  Tim Niemueller [www.niemueller.de]
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

#ifndef __LIBS_TF_UTILS_H_
#define __LIBS_TF_UTILS_H_

#include <tf/types.h>

namespace fawkes {
  namespace tf {
#if 0 /* just to make Emacs auto-indent happy */
  }
}
#endif

/** Resolve transform name.
 * @param prefix prefix to prepend to frame name
 * @param frame_name frame name
 * @return resolved frame name
 */
inline std::string
resolve(const std::string& prefix, const std::string& frame_name)
{
  if (frame_name.size() > 0) {
    if (frame_name[0] == '/') {
      return frame_name;
    }
  }
  if (prefix.size() > 0) {
    if (prefix[0] == '/') {
      std::string composite = prefix;
      composite.append("/");
      composite.append(frame_name);
      return composite;
    } else {
      std::string composite;
      composite = "/";
      composite.append(prefix);
      composite.append("/");
      composite.append(frame_name);
      return composite;
    }
  } else {
    std::string composite;
    composite = "/";
    composite.append(frame_name);
    return composite;
  }
}

/** Create an ident pose in the given frame.
 * An ident pose is with no translation and Quaternion (0,0,0,1), i.e. with
 * all Euler angles zero.
 * @param frame frame for which to get the ident transform
 * @param t time for when to get the ident transform, defaults to (0,0) which
 * means "latest possible time" for TF transforms.
 * @return ident pose in given frame at given time
 */
inline Stamped<Pose>
ident(std::string frame, Time t = Time(0,0))
{
  return
    tf::Stamped<tf::Pose>(tf::Transform(tf::Quaternion(0, 0, 0, 1),
					tf::Vector3(0, 0, 0)),
			  t, frame);
}

} // end namespace tf
} // end namespace fawkes

#endif
