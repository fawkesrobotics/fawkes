/***************************************************************************
 *  transform_publisher.cpp - Fawkes transform publisher (based on ROS tf)
 *
 *  Created: Mon Oct 24 17:13:20 2011
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

#include <tf/transform_publisher.h>

#include <blackboard/blackboard.h>
#include <interfaces/TransformInterface.h>

#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>

namespace fawkes {
  namespace tf {
#if 0 /* just to make Emacs auto-indent happy */
  }
}
#endif

/** @class TransformPublisher <tf/transform_publisher.h>
 * Utility class to send transforms.
 * The transform publisher opens an instance of TransformInterface on
 * the blackboard for writing and publishes every transform through
 * that interface. Assuming that the event-based listener is used
 * it will catch all updates even though we might send them in quick
 * succession.
 * @author Tim Niemueller
 *
 * @fn   void TransformPublisher::send_transform(const Transform &transform, const fawkes::Time &time, const std::string frame, const std::string child_frame, const bool is_static = false)
 * Convenience wrapper to send a transform.
 * This simply calls send_transform() with a StampedTransform created
 * from the data pased into this method.
 * @param transform transform to publish
 * @param time time of the transform to publish
 * @param frame reference frame ID
 * @param child_frame child frame ID
 * @param is_static true if the transform is static, i.e., it does not
 * change over time, false otherwise
 */

/** Constructor.
 * @param bb blackboard to open transform interface on, if 0 the
 * publisher will be disabled. Trying to send a transform will
 * result in a DisabledException being thrown.
 * @param bb_iface_id the blackboard interface ID to be used for the
 * opened TransformInterface. Note that the name is prefixed with "/tf/".
 */
TransformPublisher::TransformPublisher(BlackBoard *bb,
                                       const char *bb_iface_id)
  : bb_(bb), mutex_(new Mutex())
{
  if (bb_) {
	  std::string bbid = (bb_iface_id[0] == '/') ? bb_iface_id : std::string("/tf/") + bb_iface_id;
    tfif_ = bb_->open_for_writing<TransformInterface>(bbid.c_str());
    tfif_->set_auto_timestamping(false);
  }
}


/** Destructor.
 * Closes TransformInterface, hence BlackBoard must still be alive and
 * valid.
 */
TransformPublisher::~TransformPublisher()
{
  if (bb_) bb_->close(tfif_);
  delete mutex_;
}


/** Publish transform.
 * @param transform transform to publish
 * @param is_static true to mark transform as static, false otherwise
 */
void
TransformPublisher::send_transform(const StampedTransform &transform, bool is_static)
{
  if (! bb_) {
    throw DisabledException("TransformPublisher is disabled");
  }

  MutexLocker lock(mutex_);

  tfif_->set_timestamp(&transform.stamp);
  tfif_->set_frame(transform.frame_id.c_str());
  tfif_->set_child_frame(transform.child_frame_id.c_str());
  tfif_->set_static_transform(is_static);
  double translation[3], rotation[4];
  const Vector3 &t = transform.getOrigin();
  translation[0] = t.x(); translation[1] = t.y(); translation[2] = t.z();
  Quaternion r = transform.getRotation();
  assert_quaternion_valid(r);
  rotation[0] = r.x(); rotation[1] = r.y();
  rotation[2] = r.z(); rotation[3] = r.w();
  tfif_->set_translation(translation);
  tfif_->set_rotation(rotation);
  tfif_->write();
}


} // end namespace tf
} // end namespace fawkes
