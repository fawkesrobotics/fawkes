/***************************************************************************
 *  gologpp_fawkes_backend.cpp - Fawkes backend for Golog++
 *
 *  Created: Mon 26 Aug 2019 CEST 15:38
 *  Copyright  2019  Victor Mataré <matare@fh-aachen.de>
 *                   Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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

#ifndef FAWKES_GOLOGPP_FAWKES_BACKEND_H_
#define FAWKES_GOLOGPP_FAWKES_BACKEND_H_

#include "action_executor.h"
#include "aspect/action_executor_dispatcher.h"
#include "aspect/action_executor_dispatcher_inifin.h"

#include <aspect/aspect_provider.h>
#include <aspect/clock.h>
#include <aspect/inifins/inifin.h>
#include <blackboard/blackboard.h>
#include <golog++/model/platform_backend.h>
#include <logging/logger.h>

namespace fawkes {
class SkillerInterface;
class Configuration;

namespace gpp {

class GologppFawkesBackend : public gologpp::PlatformBackend,
                             public ClockAspect,
                             public GologppDispatcherAspect,
                             public AspectProviderAspect
{
public:
	GologppFawkesBackend(Configuration *config,
	                     std::string    cfg_prefix,
	                     Logger *       logger,
	                     BlackBoard *   blackboard);
	virtual ~GologppFawkesBackend();

	virtual void preempt_activity(std::shared_ptr<gologpp::Activity> a) override;
	virtual gologpp::Clock::time_point time() const noexcept override;

private:
	virtual void execute_activity(std::shared_ptr<gologpp::Activity>) override;

	SkillerInterface *            skiller_if_;
	Logger *                      logger_;
	BlackBoard *                  blackboard_;
	ActionExecutorDispatcher      action_dispatcher_;
	GologppDispatcherAspectIniFin dispatcher_inifin_;
};

} // namespace gpp
} // namespace fawkes

#endif
