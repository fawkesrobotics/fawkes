
/***************************************************************************
 *  asp_inifin.cpp - Fawkes ASPAspect initializer/finalizer
 *
 *  Created: Thu Oct 20 15:49:31 2016
 *  Copyright  2016 Björn Schäpers
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#include "asp.h"
#include "asp_inifin.h"
#include <clingo.hh>
#include <core/threading/thread_finalizer.h>
#include <logging/logger.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class ASPAspectIniFin <plugins/asp/aspect/asp_inifin.h>
 * ASPAspect initializer/finalizer.
 * This initializer/finalizer will provide the ASP node handle to threads with the ASPAspect.
 * @author Björn Schäpers
 *
 * @property ASPAspectIniFin::Log
 * @brief The logger used for Clingo Output.
 */

/** Constructor.
 */
ASPAspectIniFin::ASPAspectIniFin(void) : AspectIniFin("ASPAspect"), Log(nullptr)
{
	return;
}

void
ASPAspectIniFin::init(Thread *thread)
{
	ASPAspect *asp_thread = dynamic_cast<ASPAspect*>(thread);
	if ( asp_thread == nullptr )
	{
	throw CannotInitializeThreadException("Thread '%s' claims to have the ASPAspect, but RTTI says it has not.",
		thread->name());
	} //if ( asp_thread == nullptr )

	auto clingoLogger = [this](const Clingo::WarningCode code, char const *msg)
		{
			fawkes::Logger::LogLevel level = fawkes::Logger::LL_NONE;
			switch ( code )
			{
				case Clingo::WarningCode::AtomUndefined      :
				case Clingo::WarningCode::OperationUndefined :
				case Clingo::WarningCode::RuntimeError       : level = fawkes::Logger::LL_ERROR; break;
				case Clingo::WarningCode::Other              :
				case Clingo::WarningCode::VariableUnbounded  : level = fawkes::Logger::LL_WARN;
				case Clingo::WarningCode::FileIncluded       :
				case Clingo::WarningCode::GlobalVariable     : level = fawkes::Logger::LL_INFO; break;
			} //switch ( code )
			Log->log(level, "Clingo", msg);
			return;
		};
	asp_thread->init_ASPAspect(LockPtr<Clingo::Control>(new Clingo::Control({}, clingoLogger, 100)));
	return;
}

void
ASPAspectIniFin::finalize(Thread *thread)
{
	ASPAspect *asp_thread = dynamic_cast<ASPAspect*>(thread);
	if ( asp_thread == nullptr )
	{
	throw CannotFinalizeThreadException("Thread '%s' claims to have the ASPAspect, but RTTI says it has not.",
		thread->name());
	} //if ( asp_thread == nullptr )

	asp_thread->finalize_ASPAspect();
	return;
}

/**
 * @brief Sets the logger to use for Clingo messages.
 * @param[in] logger The new logger.
 */
void
ASPAspectIniFin::setLogger(Logger *logger)
{
	Log = logger;
	return;
}

} // end namespace fawkes
