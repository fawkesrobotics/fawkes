
/***************************************************************************
 *  engine_thread.cpp - Thread driving the XABSL Engine
 *
 *  Created: Thu Aug 07 17:01:29 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#include "engine_thread.h"
#include "xabsl_tools.h"
#include "skill_wrapper.h"

#include <core/exceptions/software.h>
#include <utils/time/time.h>
#include <interfaces/SkillerInterface.h>
#include <interfaces/ObjectPositionInterface.h>

#include <XabslEngine/XabslEngine.h>

using namespace fawkes;


/** Global XabslEngineThread required for xet_current_time(). */
static XabslEngineThread *g_xe = NULL;

/** Get current time.
 * Uses a globally set XabslEngineThread instance to determine the current
 * time, may be simulated time!
 * @return continuous time in miliseconds
 */
static unsigned long int
xet_current_time()
{
  if ( ! g_xe) {
    throw NullPointerException("No XabslEngineThread instance exists");
  }

  return g_xe->current_time();
}

/** @class XabslEngineThread "engine_thread.h"
 * Xabsl Engine Thread.
 * This thread drives the Xabsl engine.
 * @author Tim Niemueller
 */


/** Constructor. */
XabslEngineThread::XabslEngineThread()
  : Thread("XabslEngineThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_THINK)
{
}


void
XabslEngineThread::init()
{
  if ( g_xe ) {
    throw Exception("Global XabslEngineThread has already been set.");
  }
  g_xe   = this;

  __xe         = NULL;
  __xleh       = NULL;
  __now        = NULL;
  __ball_rx    = NULL;
  __ball_ry    = NULL;
  __skiller_if = NULL;
  __wm_ball_if = NULL;

  __now  = new Time(clock);
  __xleh = new XabslLoggingErrorHandler(logger);
  __xe   = new xabsl::Engine(*__xleh, &xet_current_time);

  __wm_ball_if = blackboard->open_for_reading<ObjectPositionInterface>("WM Ball");
  __skiller_if = blackboard->open_for_reading<SkillerInterface>("Skiller");

  XabslSkillWrapper::ParameterList params;
  params.push_back(std::make_pair("x", "double"));
  params.push_back(std::make_pair("y", "double"));
  params.push_back(std::make_pair("ori", "double"));
  XabslSkillWrapper *sw = new XabslSkillWrapper("relgoto", *__xleh, params);
  __wrappers[sw->name()] = sw;
  __xe->registerBasicBehavior(*sw);

  __ball_ry = __ball_rx = NULL;
  for (Interface::FieldIterator i = __wm_ball_if->fields(); i != __wm_ball_if->fields_end(); ++i) {
    if ( strcmp(i.get_name(), "relative_x") == 0 ) {
      __ball_rx = new XabslInterfaceFieldWrapper<double, float>(i.get_type(), i.get_name(), (float *)i.get_value());
      __xe->registerDecimalInputSymbol("ball.relative_x", __ball_rx,
				       (double (xabsl::FunctionProvider::*)())&XabslInterfaceFieldWrapper<double, float>::get_value);
    } else if ( strcmp(i.get_name(), "relative_y") == 0 ) {
      __ball_ry = new XabslInterfaceFieldWrapper<double, float>(i.get_type(), i.get_name(), (float *)i.get_value());
      __xe->registerDecimalInputSymbol("ball.relative_y", __ball_ry,
				       (double (xabsl::FunctionProvider::*)())&XabslInterfaceFieldWrapper<double, float>::get_value);
    }
  }

  XabslFileInputSource xinput(XABSLDIR"agent.xabslc");
  __xe->createOptionGraph(xinput);

  if ( __xleh->errorsOccurred ) {
    finalize();
    throw Exception("Error while creating XABSL engine, see log for details");
  }

  /* Test code, exporting interfaces to allow for real skill-level programming
   * is an overly complex and error prone task.
   * Since C++ methods for basic behaviors for sending a message cannot be
   * created on-the-fly wrappers would need to be written or generated for each
   * possible message type.

  __navi_if = blackboard->open_for_reading<NavigatorInterface>("Navigator");

  std::string base_name = "navi_";
  Interface::FieldIterator i;
  for (i = __navi_if->fields(); i != __navi_if->fields_end(); ++i) {
    switch (i.get_type()) {
    case Interface::IFT_BOOL:
      {
	XabslInterfaceFieldWrapper<bool> *ifw = new XabslInterfaceFieldWrapper<bool>(new InterfaceFieldPointer<bool>(i.get_type(), i.get_name(), (bool *)i.get_value()));
	__xe->registerBooleanInputSymbol((base_name + ifw->get_name()).c_str(),
					 ifw,
					 (bool (xabsl::FunctionProvider::*)())&XabslInterfaceFieldWrapper<bool>::get_value);
	__xe->registerBooleanOutputSymbol((base_name + ifw->get_name()).c_str(),
					  ifw,
					  (void (xabsl::FunctionProvider::*)(bool))&XabslInterfaceFieldWrapper<bool>::set_value,
					  (bool (xabsl::FunctionProvider::*)())&XabslInterfaceFieldWrapper<bool>::get_value);
      }
      break;
    case Interface::IFT_INT:
    case Interface::IFT_UINT:
    case Interface::IFT_LONGINT:
    case Interface::IFT_LONGUINT:
    case Interface::IFT_FLOAT:
      {
	XabslInterfaceFieldWrapper<double> *ifw = new XabslInterfaceFieldWrapper<double>(new InterfaceFieldPointer<double>(i.get_type(), i.get_name(), (double *)i.get_value()));
	__xe->registerDecimalInputSymbol((base_name + ifw->get_name()).c_str(),
					 ifw,
					 (double (xabsl::FunctionProvider::*)())&XabslInterfaceFieldWrapper<double>::get_value);
	__xe->registerDecimalOutputSymbol((base_name + ifw->get_name()).c_str(),
					  ifw,
					  (void (xabsl::FunctionProvider::*)(double))&XabslInterfaceFieldWrapper<double>::set_value,
					  (double (xabsl::FunctionProvider::*)())&XabslInterfaceFieldWrapper<double>::get_value);
      }
      break;
    case Interface::IFT_STRING:
      // ignored, XABSL can't handle that
      break;
    }
  }
  */
}


void
XabslEngineThread::finalize()
{
  g_xe = NULL;

  for (__wit = __wrappers.begin(); __wit != __wrappers.end(); ++__wit) {
    delete __wit->second;
  }
  __wrappers.clear();

  delete __xe;
  delete __xleh;
  delete __now;
  delete __ball_rx;
  delete __ball_ry;

  if (__skiller_if)  blackboard->close(__skiller_if);
  if (__wm_ball_if)  blackboard->close(__wm_ball_if);
}


void
XabslEngineThread::once()
{
  try { 
    __skiller_if->msgq_enqueue(new SkillerInterface::AcquireControlMessage());
  } catch (Exception &e) {
    logger->log_error("XabslEngineThread", "Cannot aquire exclusive skiller "
		      "control, exception follows");
    logger->log_error("XabslEngineThread", e);
  }
}

void
XabslEngineThread::loop()
{
  __now->stamp();

  __wm_ball_if->read();
  __skiller_if->read();

  __xe->execute();

  std::string skill_string = "";
  for (__wit = __wrappers.begin(); __wit != __wrappers.end(); ++__wit) {
    std::string css = __wit->second->skill_string();
    if ( css != "" ) {
      skill_string += css + "; ";
    }
  }
  if ( skill_string != "" ) {
    logger->log_debug(name(), "Skill string: %s", skill_string.c_str());
  }

  try { 
    __skiller_if->msgq_enqueue(new SkillerInterface::ExecSkillMessage(skill_string.c_str()));
  } catch (Exception &e) {
    logger->log_warn("XabslEngineThread", "Executing skill failed, exception follows");
    logger->log_warn("XabslEngineThread", e);
  }
}


/** Get current time.
 * @return continuous time in miliseconds
 */
unsigned long int
XabslEngineThread::current_time()
{
  return __now->in_msec();
}
