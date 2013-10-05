
/***************************************************************************
 *  clips_thread.h - CLIPS aspect provider thread
 *
 *  Created: Sat Jun 16 14:38:21 2012 (Mexico City)
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_CLIPS_CLIPS_THREAD_H_
#define __PLUGINS_CLIPS_CLIPS_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/aspect_provider.h>
#include <aspect/logging.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>
#include <plugins/clips/aspect/clips_inifin.h>
#include <plugins/clips/aspect/clips_feature_inifin.h>
#include <plugins/clips/aspect/clips_manager_inifin.h>

namespace CLIPS {
  class Environment;
}
namespace fawkes {
  class AspectIniFin;
  class CLIPSFeature;
}

class BlackboardCLIPSFeature;

class CLIPSThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ClockAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::AspectProviderAspect
{
 public:
  CLIPSThread();
  virtual ~CLIPSThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  const std::list<fawkes::AspectIniFin *>  inifin_list();

 private:
  fawkes::CLIPSAspectIniFin                 clips_aspect_inifin_;
  fawkes::CLIPSFeatureAspectIniFin          clips_feature_aspect_inifin_;
  fawkes::CLIPSManagerAspectIniFin          clips_manager_aspect_inifin_;
  fawkes::LockPtr<fawkes::CLIPSEnvManager>  clips_env_mgr_;

  std::list<fawkes::CLIPSFeature *> features_;
};

#endif
