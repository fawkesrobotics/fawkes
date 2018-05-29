/***************************************************************************
 *  transform_listener.cpp - Fawkes transform listener (based on ROS tf)
 *
 *  Created: Mon Oct 24 18:47:00 2011
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

#include <tf/transform_listener.h>
#include <tf/transformer.h>

#include <blackboard/blackboard.h>
#include <interfaces/TransformInterface.h>

#include <cstring>

namespace fawkes {
  namespace tf {
#if 0 /* just to make Emacs auto-indent happy */
  }
}
#endif

/** @class TransformListener <tf/transform_listener.h>
 * Receive transforms and answer queries.
 * This class connects to the blackboard and listens to all interfaces
 * publishing transforms. It opens all interfaces of type
 * TransformInterface with a TF prefix. The data is internally
 * cached. Queries are then resolved based on the received
 * information.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param bb blackboard to listen to
 * @param tf_transformer transformer to add transforms to
 * @param bb_is_remote must be true if the blackboard is a RemoteBlackboard
 */
TransformListener::TransformListener(BlackBoard *bb,
    Transformer *tf_transformer, bool bb_is_remote)
  : BlackBoardInterfaceListener("TransformListener"),
    bb_(bb), tf_transformer_(tf_transformer), bb_is_remote_(bb_is_remote)
{
  if (bb_) {
    tfifs_ = bb_->open_multiple_for_reading<TransformInterface>("/tf*");

    std::list<TransformInterface *>::iterator i;
    for (i = tfifs_.begin(); i != tfifs_.end(); ++i) {
      bbil_add_data_interface(*i);
      // update data once we 
      bb_interface_data_changed(*i);
    }
    bb_->register_listener(this);

    bbio_add_observed_create("TransformInterface", "/tf*");
    bb_->register_observer(this);
    tf_transformer->set_enabled(true);
  } else {
    tf_transformer->set_enabled(false);
  }
}


/** Destructor. */
TransformListener::~TransformListener()
{
  if (bb_) {
    bb_->unregister_listener(this);
    bb_->unregister_observer(this);

    std::list<TransformInterface *>::iterator i;
    for (i = tfifs_.begin(); i != tfifs_.end(); ++i) {
      bb_->close(*i);
    }
    tfifs_.clear();
  }
}


void
TransformListener::bb_interface_created(const char *type, const char *id) throw()
{
  if (strncmp(type, "TransformInterface", __INTERFACE_TYPE_SIZE) != 0)  return;

  TransformInterface *tfif;
  try {
    tfif = bb_->open_for_reading<TransformInterface>(id, "TF-Listener");
  } catch (Exception &e) {
    // ignored
    return;
  }

  bb_interface_data_changed(tfif);

  try {
    bbil_add_data_interface(tfif);
    bb_->update_listener(this);
    tfifs_.push_back(tfif);
  } catch (Exception &e) {
    bb_->close(tfif);
    return;
  }
}

void
TransformListener::bb_interface_writer_removed(Interface *interface,
                                               unsigned int instance_serial)
  throw()
{
  conditional_close(interface);
}


void
TransformListener::bb_interface_reader_removed(Interface *interface,
                                               unsigned int instance_serial)
  throw()
{
  conditional_close(interface);
}


void
TransformListener::conditional_close(Interface *interface) throw()
{
  if (bb_is_remote_) {
    return;
  }
  // Verify it's a TransformInterface
  TransformInterface *tfif = dynamic_cast<TransformInterface *>(interface);
  if (! tfif) return;
  
  std::list<TransformInterface *>::iterator i;
  for (i = tfifs_.begin(); i != tfifs_.end(); ++i) {
    if (*interface == **i) {
      if (! interface->has_writer() && (interface->num_readers() == 1)) {
        // It's only us
        bbil_remove_data_interface(*i);
        bb_->update_listener(this);
        bb_->close(*i);
        tfifs_.erase(i);
        break;
      }
    }
  }
}


void
TransformListener::bb_interface_data_changed(Interface *interface) throw()
{
  TransformInterface *tfif = dynamic_cast<TransformInterface *>(interface);
  if (! tfif) return;

  tfif->read();

  std::string authority;
  if (bb_is_remote_) {
    authority = "remote";
  } else {
    std::string authority = tfif->writer();
  }
  
  double *translation = tfif->translation();
  double *rotation = tfif->rotation();
  const Time *time = tfif->timestamp();
  const std::string frame_id = tfif->frame();
  const std::string child_frame_id = tfif->child_frame();

  try {
    Vector3 t(translation[0], translation[1], translation[2]);
    Quaternion r(rotation[0], rotation[1], rotation[2], rotation[3]);
    assert_quaternion_valid(r);
    Transform tr(r, t);

    StampedTransform str(tr, *time, frame_id, child_frame_id);

    tf_transformer_->set_transform(str, authority, tfif->is_static_transform());
  } catch (InvalidArgumentException &e) {
    // ignore invalid, might just be not initialized, yet.
  }
}

} // end namespace tf
} // end namespace fawkes
