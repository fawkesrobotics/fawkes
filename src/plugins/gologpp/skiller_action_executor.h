/***************************************************************************
 *  skiller_action_executor.h - Execute skills for Golog++ activities
 *
 *  Created: Thu 03 Oct 2019 08:51:27 CEST 08:51
 *  Copyright  2019  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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

#ifndef FAWKES_GOLOGPP_SKILLER_ACTION_EXECUTOR_H
#define FAWKES_GOLOGPP_SKILLER_ACTION_EXECUTOR_H

#include "action_executor.h"

#include <blackboard/interface_listener.h>
#include <utils/misc/map_skill.h>

#include <string>

namespace fawkes {
class Blackboard;
class Configuration;
class SkillerInterface;
} // namespace fawkes

namespace fawkes_gpp {

class SkillerActionExecutor : public ActionExecutor, public fawkes::BlackBoardInterfaceListener
{
public:
	SkillerActionExecutor(fawkes::Logger *       logger,
	                      fawkes::BlackBoard *   blackboard,
	                      fawkes::Configuration *config);
	virtual ~SkillerActionExecutor();
	void         start(std::shared_ptr<gologpp::Activity> activity) override;
	void         stop(std::shared_ptr<gologpp::Grounding<gologpp::Action>> activity) override;
	bool         can_execute_activity(std::shared_ptr<gologpp::Activity> activity) const override;
	virtual void bb_interface_data_changed(fawkes::Interface *) throw() override;

private:
	const char *               name() const;
	void                       initialize_action_skill_mapping();
	std::string                map_activity_to_skill(std::shared_ptr<gologpp::Activity> activity);
	fawkes::ActionSkillMapping action_skill_mapping_;
	fawkes::BlackBoard *       blackboard_;
	fawkes::SkillerInterface * skiller_if_;
	fawkes::Configuration *    config_;
};

} // namespace fawkes_gpp

#endif /* !FAWKES_GOLOGPP_SKILLER_ACTION_EXECUTOR_H */
