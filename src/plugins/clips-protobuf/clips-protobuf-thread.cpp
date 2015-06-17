
/***************************************************************************
 *  clips-protobuf-thread.cpp -  Protobuf communication for CLIPS
 *
 *  Created: Tue Apr 16 13:04:07 2013
 *  Copyright  2006-2012  Tim Niemueller [www.niemueller.de]
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

#include "clips-protobuf-thread.h"

#include <protobuf_clips/communicator.h>

using namespace fawkes;
using namespace protobuf_clips;

/** @class ClipsProtobufThread "clips-protobuf-thread.h"
 * Provide protobuf functionality to CLIPS environment.
 * @author Tim Niemueller
 */

/** Constructor. */
ClipsProtobufThread::ClipsProtobufThread()
  : Thread("ClipsProtobufThread", Thread::OPMODE_WAITFORWAKEUP),
    CLIPSFeature("protobuf"), CLIPSFeatureAspect(this)
{
}


/** Destructor. */
ClipsProtobufThread::~ClipsProtobufThread()
{
}


void
ClipsProtobufThread::init()
{
  cfg_proto_dirs_.clear();
  try {
    cfg_proto_dirs_ = config->get_strings("/clips-protobuf/proto-dirs");
    for (size_t i = 0; i < cfg_proto_dirs_.size(); ++i) {
      std::string::size_type pos;
      if ((pos = cfg_proto_dirs_[i].find("@BASEDIR@")) != std::string::npos) {
	cfg_proto_dirs_[i].replace(pos, 9, BASEDIR);
      }
      if ((pos = cfg_proto_dirs_[i].find("@FAWKES_BASEDIR@")) != std::string::npos) {
	cfg_proto_dirs_[i].replace(pos, 16, FAWKES_BASEDIR);
      }
      if ((pos = cfg_proto_dirs_[i].find("@RESDIR@")) != std::string::npos) {
	cfg_proto_dirs_[i].replace(pos, 8, RESDIR);
      }
      if ((pos = cfg_proto_dirs_[i].find("@CONFDIR@")) != std::string::npos) {
	cfg_proto_dirs_[i].replace(pos, 9, CONFDIR);
      }
      if (cfg_proto_dirs_[i][cfg_proto_dirs_.size()-1] != '/') {
	cfg_proto_dirs_[i] += "/";
      }
      //logger->log_warn(name(), "DIR: %s", cfg_proto_dirs_[i].c_str());
    }
  } catch (Exception &e) {
    logger->log_warn(name(), "Failed to load proto paths from config, exception follows");
    logger->log_warn(name(), e);
  } // ignore, use default

}


void
ClipsProtobufThread::finalize()
{
  for (auto pb_comm : pb_comms_) {
    delete pb_comm.second;
  }
  pb_comms_.clear();
}


void
ClipsProtobufThread::clips_context_init(const std::string &env_name,
					LockPtr<CLIPS::Environment> &clips)
{
  logger->log_info(name(), "Called to initialize environment %s", env_name.c_str());
  pb_comms_[env_name] =
    new ClipsProtobufCommunicator(*clips, *clips.objmutex_ptr(), cfg_proto_dirs_,
				  logger);
  clips->batch_evaluate(SRCDIR"/clips/protobuf.clp");
}

void
ClipsProtobufThread::clips_context_destroyed(const std::string &env_name)
{
  logger->log_info(name(), "Removing environment %s", env_name.c_str());
  if (pb_comms_.find(env_name) != pb_comms_.end()) {
    delete pb_comms_[env_name];
    pb_comms_.erase(env_name);
  }
}

void
ClipsProtobufThread::loop()
{
}
