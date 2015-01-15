
/***************************************************************************
 *  clips_navgraph_thread.cpp -  NavGraph feature for CLIPS
 *
 *  Created: Wed Oct 09 19:27:41 2013
 *  Copyright  2006-2013  Tim Niemueller [www.niemueller.de]
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

#include "clips_navgraph_thread.h"

#include <navgraph/navgraph.h>
#include <navgraph/constraints/static_list_edge_constraint.h>
#include <navgraph/constraints/constraint_repo.h>

#include <clipsmm.h>

using namespace fawkes;

/** @class ClipsNavGraphThread "clips-protobuf-thread.h"
 * Provide protobuf functionality to CLIPS environment.
 * @author Tim Niemueller
 */

/** Constructor. */
ClipsNavGraphThread::ClipsNavGraphThread()
  : Thread("ClipsNavGraphThread", Thread::OPMODE_WAITFORWAKEUP),
    CLIPSFeature("navgraph"), CLIPSFeatureAspect(this)
{
}


/** Destructor. */
ClipsNavGraphThread::~ClipsNavGraphThread()
{
}


void
ClipsNavGraphThread::init()
{
  navgraph->add_change_listener(this);

  edge_constraint_ = new NavGraphStaticListEdgeConstraint("clips");
  navgraph->constraint_repo()->register_constraint(edge_constraint_);
}


void
ClipsNavGraphThread::finalize()
{
  navgraph->constraint_repo()->unregister_constraint(edge_constraint_->name());
  delete edge_constraint_;

  navgraph->remove_change_listener(this);
  envs_.clear();
}


void
ClipsNavGraphThread::clips_context_init(const std::string &env_name,
					LockPtr<CLIPS::Environment> &clips)
{
  envs_[env_name] = clips;
  logger->log_info(name(), "Called to initialize environment %s", env_name.c_str());

  clips.lock();
  clips->batch_evaluate(SRCDIR"/clips/navgraph.clp");
  clips_navgraph_load(clips);

  clips->add_function("navgraph-block-edge",
    sigc::slot<void, std::string, std::string>(
      sigc::bind<0>(
        sigc::mem_fun(*this, &ClipsNavGraphThread::clips_navgraph_block_edge),
	env_name)
    )
  );

  clips->add_function("navgraph-unblock-edge",
    sigc::slot<void, std::string, std::string>(
      sigc::bind<0>(
        sigc::mem_fun(*this, &ClipsNavGraphThread::clips_navgraph_unblock_edge),
	env_name)
    )
  );

  clips.unlock();
}

void
ClipsNavGraphThread::clips_context_destroyed(const std::string &env_name)
{
  envs_.erase(env_name);
  logger->log_info(name(), "Removing environment %s", env_name.c_str());
}

void
ClipsNavGraphThread::clips_navgraph_load(LockPtr<CLIPS::Environment> &clips)
{
  try {
    const std::vector<NavGraphNode> &nodes = navgraph->nodes();
    const std::vector<NavGraphEdge> &edges = navgraph->edges();

    clips->assert_fact_f("(navgraph (name \"%s\"))", navgraph->name().c_str());

    for (const NavGraphNode &n : nodes) {
      std::string props_string;
      const std::map<std::string, std::string> &properties = n.properties();
      for (auto p : properties) {
	props_string += " \"" + p.first + "\" \"" + p.second + "\"";
      }
      clips->assert_fact_f("(navgraph-node (name \"%s\") (pos %f %f) (properties %s))",
			   n.name().c_str(), n.x(), n.y(), props_string.c_str());
    }

    for (const NavGraphEdge &e : edges) {
      std::string props_string;
      const std::map<std::string, std::string> &properties = e.properties();
      for (auto p : properties) {
	props_string += " \"" + p.first + "\" \"" + p.second + "\"";
      }
      clips->assert_fact_f("(navgraph-edge (from \"%s\") (to \"%s\") (directed %s) "
			   "(properties %s))",
			   e.from().c_str(), e.to().c_str(),
			   e.is_directed() ? "TRUE" : "FALSE", props_string.c_str());
    }

  } catch (Exception &e) {
    logger->log_warn(name(), "Failed to assert navgraph, exception follows");
    logger->log_warn(name(), e);
    clips->assert_fact_f("(navgraph-load-fail %s)", *(e.begin()));
  }
}


void
ClipsNavGraphThread::clips_navgraph_block_edge(std::string env_name,
					       std::string from, std::string to)
{
  const std::vector<NavGraphEdge> &graph_edges = navgraph->edges();

  for (const NavGraphEdge &edge : graph_edges) {
    if (edge.from() == from && edge.to() == to) {
      edge_constraint_->add_edge(edge);
      return;
    }
  }

  logger->log_warn(name(), "Environment %s tried to block edge %s--%s, "
		   "which does not exist in graph", env_name.c_str(),
		   from.c_str(), to.c_str());
}


void
ClipsNavGraphThread::clips_navgraph_unblock_edge(std::string env_name,
						 std::string from, std::string to)
{
  const std::vector<NavGraphEdge> &graph_edges = navgraph->edges();

  for (const NavGraphEdge &edge : graph_edges) {
    if (edge.from() == from && edge.to() == to) {
      edge_constraint_->remove_edge(edge);
      return;
    }
  }

  logger->log_warn(name(), "Environment %s tried to unblock edge %s--%s, "
		   "which does not exist in graph", env_name.c_str(),
		   from.c_str(), to.c_str());
}

void
ClipsNavGraphThread::graph_changed() throw()
{
  for (auto e : envs_) {
    logger->log_debug(name(), "Graph changed, re-asserting in environment %s", e.first.c_str());
    fawkes::LockPtr<CLIPS::Environment> &clips = e.second;
    clips.lock();
    clips->evaluate("(navgraph-cleanup)");
    clips_navgraph_load(clips);
    clips.unlock();
  }
}


void
ClipsNavGraphThread::loop()
{
}
