
/***************************************************************************
 *  clips-protobuf-thread.h - Protobuf communication for CLIPS
 *
 *  Created: Tue Apr 16 13:02:22 2013
 *  Copyright  2006-2013  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#ifndef __PLUGINS_CLIPS_PROTOBUF_CLIPS_PROTOBUF_THREAD_H_
#define __PLUGINS_CLIPS_PROTOBUF_CLIPS_PROTOBUF_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <plugins/clips/aspect/clips_feature.h>

#include <vector>
#include <string>
#include <map>

namespace protobuf_clips {
  class ClipsProtobufCommunicator;
}

class ClipsProtobufThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::CLIPSFeature,
  public fawkes::CLIPSFeatureAspect
{
 public:
  ClipsProtobufThread();
  virtual ~ClipsProtobufThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

  // for CLIPSFeature
  virtual void clips_context_init(const std::string &env_name,
				  fawkes::LockPtr<CLIPS::Environment> &clips);
  virtual void clips_context_destroyed(const std::string &env_name);

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  std::map<std::string, protobuf_clips::ClipsProtobufCommunicator *> pb_comms_;
  std::vector<std::string> cfg_proto_dirs_;

};

#endif
