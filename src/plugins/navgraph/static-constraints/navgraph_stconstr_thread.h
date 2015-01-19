/***************************************************************************
 *  navgraph_stconstr_thread.h - static constraints for navgraph
 *
 *  Created: Fri Jul 11 17:31:47 2014
 *  Copyright  2014  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_NAVGRAPH_NAVGRAPH_STCONSTR_THREAD_H_
#define __PLUGINS_NAVGRAPH_NAVGRAPH_STCONSTR_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>

#include <navgraph/navgraph.h>
#include <navgraph/aspect/navgraph.h>
#include <navgraph/constraints/constraint_repo.h>

namespace fawkes {
  class NavGraphStaticListNodeConstraint;
  class NavGraphStaticListEdgeConstraint;
  class NavGraphStaticListEdgeCostConstraint;
  class NavGraphPolygonNodeConstraint;
  class NavGraphPolygonEdgeConstraint;
}

class NavGraphStaticConstraintsThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::NavGraphAspect
{
 public:
  NavGraphStaticConstraintsThread();
  virtual ~NavGraphStaticConstraintsThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

  /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run();}

 private:
  fawkes::NavGraphStaticListNodeConstraint      *node_constraint_;
  fawkes::NavGraphStaticListEdgeConstraint      *edge_constraint_;
  fawkes::NavGraphStaticListEdgeCostConstraint  *edge_cost_constraint_;
  fawkes::NavGraphPolygonNodeConstraint         *node_poly_constraint_;
  fawkes::NavGraphPolygonEdgeConstraint         *edge_poly_constraint_;
};

#endif
