/***************************************************************************
 *  transform_listener.h - Fawkes transform listener (based on ROS tf)
 *
 *  Created: Mon Oct 24 18:09:47 2011
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

#ifndef __LIBS_TF_TRANSFORM_LISTENER_H_
#define __LIBS_TF_TRANSFORM_LISTENER_H_

#include <tf/types.h>

#include <blackboard/interface_listener.h>
#include <blackboard/interface_observer.h>

#include <list>

namespace fawkes {

  class BlackBoard;
  class TransformInterface;

  namespace tf {
#if 0 /* just to make Emacs auto-indent happy */
  }
}
#endif

class Transformer;

class TransformListener
: public BlackBoardInterfaceObserver,
  public BlackBoardInterfaceListener
{
 public:
	TransformListener(BlackBoard *bb, Transformer *tf_transformer,
	    bool bb_is_remote=false);
  virtual ~TransformListener();

  // for BlackBoardInterfaceObserver
  virtual void bb_interface_created(const char *type, const char *id) throw();

  // for BlackBoardInterfaceListener
  virtual void bb_interface_data_changed(Interface *interface) throw();
  virtual void bb_interface_writer_removed(Interface *interface,
                                           unsigned int instance_serial) throw();
  virtual void bb_interface_reader_removed(Interface *interface,
                                           unsigned int instance_serial) throw();

 private:
  void conditional_close(Interface *interface) throw();

 private:
  BlackBoard  *bb_;
  Transformer *tf_transformer_;
  bool bb_is_remote_;

  std::list<TransformInterface *> tfifs_;
};

} // end namespace tf
} // end namespace fawkes

#endif
