
/***************************************************************************
 *  control_thread.cpp - Fawkes ECLiPSe Control Thread
 *
 *  Created: Wed Jul 15 15:09:09 2009
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

#include "control_thread.h"
#include "eclipse_thread.h"
#include <core/exception.h>
#include <interfaces/TestInterface.h>
#include <cstring>
#include <cstdlib>

/** @class AgentControlThread "control_thread.h"
 * This thread controls the agent thread by sending signals.
 * @author Daniel Beck
 */

using namespace fawkes;

/** Constructor.
 * @param eclipse_thread the ECLiPSe agent thread
 */
AgentControlThread::AgentControlThread( EclipseAgentThread* eclipse_thread )
  : Thread( "AgentControlThread", Thread::OPMODE_WAITFORWAKEUP ),
    BlockedTimingAspect( BlockedTimingAspect::WAKEUP_HOOK_THINK ),
    m_eclipse_thread( eclipse_thread )
{
}

/** Destructor. */
AgentControlThread::~AgentControlThread()
{
}

void
AgentControlThread::init()
{
  // open & register interfaces
  m_test_iface = blackboard->open_for_writing< TestInterface >("eclipse_clp_test");
  m_debug_iface = blackboard->open_for_reading< EclipseDebuggerInterface >("eclipse_clp_connect");
}

bool
AgentControlThread::prepare_finalize_user()
{
  m_eclipse_thread->post_event( "terminate" );

  return true;
}

void
AgentControlThread::finalize()
{
  // close interfaces
  blackboard->close( m_test_iface );
}

void
AgentControlThread::loop()
{

  //if the debug interface has a writer (so tktools module is loaded),
  //then post event to check for tktool connection request within eclipse
  if (m_debug_iface->has_writer()){
    m_eclipse_thread->post_event( "check_debug_msg" );
  }

  // this is only used by the dummy agent
  while ( !m_test_iface->msgq_empty() )
  {
    if ( m_test_iface->msgq_first_is< TestInterface::CalculateMessage >() )
    {
      TestInterface::CalculateMessage* msg = m_test_iface->msgq_first< TestInterface::CalculateMessage >();

      m_test_iface->set_result( msg->summand() + msg->addend() );
    }

    m_test_iface->msgq_pop();
  }
  m_test_iface->write();


  m_eclipse_thread->post_event( "update" );

  /* call eclipse thread (that thread has no blocked timing aspect
   * and is therefore not called by the mainapp, so this has to be
   * done here)
  */
  //m_eclipse_thread->loop();
}
