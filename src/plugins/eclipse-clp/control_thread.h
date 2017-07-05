
/***************************************************************************
 *  control_thread.h - Fawkes ECLiPSe Control Thread
 *
 *  Created: Wed Jul 15 15:05:57 2009
 *  Copyright  2009  Daniel Beck
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

#ifndef __PLUGINS_ECLIPSE_CLP_CONTROL_THREAD_H_
#define __PLUGINS_ECLIPSE_CLP_CONTROL_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/blackboard.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <interfaces/EclipseDebuggerInterface.h>
#include <interfaces/ExitSimulationInterface.h>

namespace fawkes {
  class TestInterface;
}

class EclipseAgentThread;

class AgentControlThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::LoggingAspect
{
 public:
  AgentControlThread( EclipseAgentThread* eclipse_thread );
  virtual ~AgentControlThread();

  virtual void init();
  virtual bool prepare_finalize_user();
  virtual void finalize();
  virtual void loop();

 private:
  EclipseAgentThread* m_eclipse_thread;

  fawkes::TestInterface* m_test_iface;
  fawkes::ExitSimulationInterface* m_exit_iface;
  fawkes::EclipseDebuggerInterface* m_debug_iface;
  
  bool 		allow_shutdown_;
  std::string 	fawkes_path_;
  std::string 	simulation_shutdown_script_;
};

#endif /* __PLUGINS_ECLIPSE_CLP_CONTROL_THREAD_H_ */

