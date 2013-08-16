
/***************************************************************************
 *  clips_thread.cpp -  CLIPS environment providing Thread
 *
 *  Created: Sat Jun 16 14:40:56 2012 
 *  Copyright  2006-2012  Tim Niemueller [www.niemueller.de]
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

#include "clips_thread.h"
#include <plugins/clips/aspect/clips_env_manager.h>

#include <clipsmm.h>

using namespace fawkes;

/** @class CLIPSThread "clips_thread.h"
 * CLIPS environment thread.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
CLIPSThread::CLIPSThread()
  : Thread("CLIPSThread", Thread::OPMODE_WAITFORWAKEUP),
    AspectProviderAspect(inifin_list())
{
}


/** Destructor. */
CLIPSThread::~CLIPSThread()
{
}


void
CLIPSThread::init()
{
  CLIPS::init();
  clips_env_mgr_ = new CLIPSEnvManager(logger);
  clips_aspect_inifin_.set_manager(clips_env_mgr_);
  clips_feature_aspect_inifin_.set_manager(clips_env_mgr_);
}


void
CLIPSThread::finalize()
{
  clips_env_mgr_.clear();
}


void
CLIPSThread::loop()
{
}

std::list<AspectIniFin *>
CLIPSThread::inifin_list()
{
  std::list<AspectIniFin *> rv;
  rv.push_back(&clips_aspect_inifin_);
  rv.push_back(&clips_feature_aspect_inifin_);
  return rv;
}
