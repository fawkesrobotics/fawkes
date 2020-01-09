/***************************************************************************
 *  skiller_simulator_navgraph_thread.h - Skill exec times from navgraph
 *
 *  Created: Tue 07 Jan 2020 15:40:18 CET 15:40
 *  Copyright  2020  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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

#pragma once

#include <aspect/blackboard.h>
#include <aspect/configurable.h>
#include <core/threading/thread.h>
#include <navgraph/aspect/navgraph.h>
#include <plugins/skiller-simulator/execution_time_estimator_aspect/execution_time_estimator_aspect.h>

class SkillerSimulatorNavgraphEstimatorThread
: public fawkes::Thread,
  public fawkes::BlackBoardAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::NavGraphAspect,
  public fawkes::skiller_simulator::ExecutionTimeEstimatorsAspect
{
public:
	SkillerSimulatorNavgraphEstimatorThread();
	void init();
};
