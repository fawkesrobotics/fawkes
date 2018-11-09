/***************************************************************************
 *  asp_thread.cpp - ASP environment providing Thread
 *
 *  Created: Thu Oct 20 15:49:31 2016
 *  Copyright  2016 Björn Schäpers
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

#include "asp_thread.h"

#include <clingo.hh>

using namespace fawkes;

/** @class ASPThread "clips_thread.h"
 * ASP environment thread.
 *
 * @author Björn Schäpers
 *
 * @property ASPThread::asp_inifin_
 * @brief The initializer/finalizer for the ASPAspect.
 *
 * @property ASPThread::clingo_mgr_inifin_
 * @brief The initializer/finalizer for the ClingoManagerAspect.
 *
 * @property ASPThread::control_mgr_
 * @brief The clingo control manager.
 */

/** Constructor. */
ASPThread::ASPThread()
: Thread("ASPThread", Thread::OPMODE_WAITFORWAKEUP),
  AspectProviderAspect([this]() {
	                       std::list<fawkes::AspectIniFin*> ret;
	                       ret.emplace_back(&asp_inifin_);
	                       ret.emplace_back(&clingo_mgr_inifin_);
	                       return ret;
                       }()),
  control_mgr_(new ClingoControlManager)
{
}

void
ASPThread::init()
{
	control_mgr_->set_logger(logger);
	asp_inifin_.set_control_manager(control_mgr_);
	clingo_mgr_inifin_.set_control_manager(control_mgr_);
}

void
ASPThread::finalize()
{
}

void
ASPThread::loop()
{
}
