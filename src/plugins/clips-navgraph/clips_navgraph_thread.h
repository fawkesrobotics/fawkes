
/***************************************************************************
 *  clips_navgraph_thread.h - NavGraph feature for CLIPS
 *
 *  Created: Wed Oct 09 19:26:41 2013
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

#ifndef __PLUGINS_CLIPS_NAVGRAPH_CLIPS_NAVGRAPH_THREAD_H_
#define __PLUGINS_CLIPS_NAVGRAPH_CLIPS_NAVGRAPH_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <plugins/clips/aspect/clips_feature.h>
#include <navgraph/aspect/navgraph.h>
#include <navgraph/navgraph.h>

#include <vector>
#include <string>
#include <map>

namespace fawkes {
  class NavGraphStaticListEdgeConstraint;
}

class ClipsNavGraphThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::NavGraphAspect,
  public fawkes::CLIPSFeature,
  public fawkes::CLIPSFeatureAspect,
  public fawkes::NavGraph::ChangeListener
{
 public:
  ClipsNavGraphThread();
  virtual ~ClipsNavGraphThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

  // for CLIPSFeature
  virtual void clips_context_init(const std::string &env_name,
				  fawkes::LockPtr<CLIPS::Environment> &clips);
  virtual void clips_context_destroyed(const std::string &env_name);

  virtual void graph_changed() throw();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  void clips_navgraph_load(fawkes::LockPtr<CLIPS::Environment> &clips);
  void clips_navgraph_block_edge(std::string env_name, std::string from, std::string to);
  void clips_navgraph_unblock_edge(std::string env_name, std::string from, std::string to);

 private:
  std::map<std::string, fawkes::LockPtr<CLIPS::Environment> >  envs_;

  fawkes::NavGraphStaticListEdgeConstraint  *edge_constraint_;
};

#endif
