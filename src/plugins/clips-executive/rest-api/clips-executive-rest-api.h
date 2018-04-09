
/***************************************************************************
 *  clips-executive-rest-api.h -  CLIPS Executive REST API
 *
 *  Created: Fri Mar 16 17:15:34 2018
 *  Copyright  2006-2018  Tim Niemueller [www.niemueller.de]
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

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/webview.h>
#include <plugins/clips/aspect/clips_manager.h>

#include <webview/rest_api.h>
#include <webview/rest_array.h>

#include <core/utils/lockptr.h>
#include <clipsmm/fact.h>

#include "model/Goal.h"
#include "model/Plan.h"
#include "model/DomainOperator.h"
#include "model/DomainObject.h"
#include "model/DomainPredicate.h"
#include "model/DomainFact.h"
#include "model/DomainPreconditionCompound.h"
#include "model/DomainPreconditionAtom.h"

namespace CLIPS {
	class Environment;
}

class ClipsExecutiveRestApi
: public fawkes::Thread,
  public fawkes::LoggingAspect,
	public fawkes::WebviewAspect,
  public fawkes::CLIPSManagerAspect
{
 public:
	ClipsExecutiveRestApi();
	~ClipsExecutiveRestApi();

	virtual void init();
	virtual void loop();
	virtual void finalize();

 private:
	typedef std::pair<std::string, std::string> PlanKey;
	typedef std::tuple<std::string, std::string, int64_t> PlanActionKey;
	typedef std::list<CLIPS::Fact::pointer> ClipsFactList;
	typedef std::map<PlanActionKey, ClipsFactList> PreCompoundMap;
	typedef std::map<PlanActionKey, ClipsFactList> PreAtomMap;
	typedef std::map<PlanKey, CLIPS::Fact::pointer> PlanMap;
	typedef std::map<PlanKey, ClipsFactList> PlanActionMap;

 private:

	WebviewRestArray<Goal>            cb_list_goals();
	WebviewRestArray<DomainOperator>  cb_list_domain_operators();
	WebviewRestArray<DomainObject>    cb_list_domain_objects();
	WebviewRestArray<DomainPredicate> cb_list_domain_predicates();
	WebviewRestArray<DomainFact>      cb_list_domain_facts();
	WebviewRestArray<Plan>            cb_list_plans();

	Goal cb_get_goal(fawkes::WebviewRestParams& params);
	Plan cb_get_plan(fawkes::WebviewRestParams& params);

	Goal generate_goal(CLIPS::Fact::pointer fact);
	void gen_plan_precompute(std::map<PlanKey, CLIPS::Fact::pointer> &plans,
	                         std::map<PlanKey, ClipsFactList> &plan_actions,
	                         PreCompoundMap &prec,
	                         PreAtomMap &prea);
	std::shared_ptr<DomainPreconditionAtom>
		gen_domain_precondition_atom(const CLIPS::Fact::pointer fact);
	
	std::shared_ptr<DomainPreconditionCompound>
		gen_domain_precondition_compound(const CLIPS::Fact::pointer fact,
		                                 const PlanActionKey &plan_action_key,
		                                 PreCompoundMap &prec,
		                                 PreAtomMap &prea);

	Plan gen_plan(const PlanKey &plan_key,
	              const CLIPS::Fact::pointer fact, PlanActionMap &plan_actions,
	              PreCompoundMap &prec, PreAtomMap &prea);

 private:
	fawkes::WebviewRestApi *rest_api_;
	fawkes::LockPtr<CLIPS::Environment> clips_;
	
};
