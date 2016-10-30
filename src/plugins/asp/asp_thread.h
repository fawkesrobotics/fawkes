
/***************************************************************************
 *  asp_thread.h - ASP aspect provider thread
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

#ifndef __PLUGINS_ASP_ASP_THREAD_H_
#define __PLUGINS_ASP_ASP_THREAD_H_

#include <aspect/aspect_provider.h>
#include <aspect/logging.h>
#include <core/threading/thread.h>

#include "aspect/asp_inifin.h"
#include "aspect/clingo_control_manager.h"
#include "aspect/clingo_manager_inifin.h"

namespace fawkes {
	class AspectIniFin;
}

class ASPThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::AspectProviderAspect
{
	private:
	fawkes::ASPAspectIniFin ASPIniFin;
	fawkes::ClingoManagerAspectIniFin ClingoIniFin;
	fawkes::LockPtr<fawkes::ClingoControlManager> CtrlMgr;

	protected:
	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
	void run(void) override { Thread::run(); return; }

	public:
	ASPThread(void);

	void init(void) override;
	void loop(void) override;
	void finalize(void) override;
};

#endif
