
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

#pragma once

#include "model/DomainFact.h"
#include "model/DomainObject.h"
#include "model/DomainOperator.h"
#include "model/DomainPredicate.h"
#include "model/Goal.h"
#include "model/GroundedFormula.h"
#include "model/GroundedPDDLFormula.h"
#include "model/GroundedPDDLPredicate.h"
#include "model/PDDLFormula.h"
#include "model/PDDLGrounding.h"
#include "model/PDDLPredicate.h"
#include "model/Plan.h"

#include <aspect/logging.h>
#include <aspect/webview.h>
#include <clipsmm/fact.h>
#include <core/threading/thread.h>
#include <core/utils/lockptr.h>
#include <plugins/clips/aspect/clips_manager.h>
#include <webview/rest_api.h>
#include <webview/rest_array.h>

namespace CLIPS {
class Environment;
}

class ClipsExecutiveRestApi : public fawkes::Thread,
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
	typedef std::pair<std::string, std::string>                     PlanKey;
	typedef std::tuple<std::string, std::string, int64_t>           PlanActionKey;
	typedef std::tuple<std::string, std::string>                    GroundedPDDLKey;
	typedef std::list<CLIPS::Fact::pointer>                         ClipsFactList;
	typedef std::map<PlanActionKey, ClipsFactList>                  PreCompoundMap;
	typedef std::map<PlanActionKey, ClipsFactList>                  PreAtomMap;
	typedef std::map<PlanKey, CLIPS::Fact::pointer>                 PlanMap;
	typedef std::map<PlanKey, ClipsFactList>                        PlanActionMap;
	typedef std::map<std::string, CLIPS::Fact::pointer>             PDDLGroundingMap;
	typedef std::map<std::string, CLIPS::Fact::pointer>             PDDLFormulaMap;
	typedef std::map<std::string, CLIPS::Fact::pointer>             PDDLPredicateMap;
	typedef std::map<std::string, ClipsFactList>                    GroundedPDDLFormulaMap;
	typedef std::map<std::string, ClipsFactList>                    GroundedPDDLPredicateMap;
	typedef std::tuple<CLIPS::Fact::pointer, CLIPS::Fact::pointer>  PDDLFormulaTreeNode;
	typedef std::list<PDDLFormulaTreeNode>                          PDDLFormulaTreeLevel;
	typedef std::map<std::string, PDDLFormulaTreeLevel>             PDDLFormulaTreeMap;
	typedef std::map<std::string, std::shared_ptr<GroundedFormula>> GroundedFormulaMap;

private:
	WebviewRestArray<Goal>                  cb_list_goals();
	WebviewRestArray<DomainOperator>        cb_list_domain_operators();
	WebviewRestArray<DomainObject>          cb_list_domain_objects();
	WebviewRestArray<DomainPredicate>       cb_list_domain_predicates();
	WebviewRestArray<DomainFact>            cb_list_domain_facts();
	WebviewRestArray<Plan>                  cb_list_plans();
	WebviewRestArray<PDDLGrounding>         cb_list_pddl_groundings();
	WebviewRestArray<PDDLFormula>           cb_list_pddl_formulas();
	WebviewRestArray<PDDLPredicate>         cb_list_pddl_predicates();
	WebviewRestArray<GroundedPDDLFormula>   cb_list_grounded_pddl_formulas();
	WebviewRestArray<GroundedPDDLPredicate> cb_list_grounded_pddl_predicates();

	Goal                  cb_get_goal(fawkes::WebviewRestParams &params);
	Plan                  cb_get_plan(fawkes::WebviewRestParams &params);
	PDDLGrounding         cb_get_pddl_groundings(fawkes::WebviewRestParams &params);
	PDDLFormula           cb_get_pddl_formulas(fawkes::WebviewRestParams &params);
	PDDLPredicate         cb_get_pddl_predicates(fawkes::WebviewRestParams &params);
	GroundedPDDLFormula   cb_get_grounded_pddl_formulas(fawkes::WebviewRestParams &params);
	GroundedPDDLPredicate cb_get_grounded_pddl_predicates(fawkes::WebviewRestParams &params);

	Goal            generate_goal(CLIPS::Fact::pointer fact);
	void            gen_plan_precompute(std::map<PlanKey, CLIPS::Fact::pointer> &plans,
	                                    std::map<PlanKey, ClipsFactList> &       plan_actions,
	                                    PreCompoundMap &                         prec,
	                                    PreAtomMap &                             prea,
	                                    PDDLGroundingMap &                       pgm,
	                                    PDDLFormulaMap &                         pfm,
	                                    PDDLPredicateMap &                       ppm,
	                                    GroundedPDDLFormulaMap &                 gpfm,
	                                    GroundedPDDLPredicateMap &               gppm);
	GroundedFormula gen_plan_compute_precons(PDDLFormulaTreeNode node,
	                                         PDDLFormulaTreeMap  tree,
	                                         PDDLGroundingMap    groundings);

	Plan gen_plan(const PlanKey &            plan_key,
	              const CLIPS::Fact::pointer fact,
	              PlanActionMap &            plan_actions,
	              PreCompoundMap &           prec,
	              PreAtomMap &               prea,
	              PDDLGroundingMap &         pgm,
	              PDDLFormulaMap &           pfm,
	              PDDLPredicateMap &         ppm,
	              GroundedPDDLFormulaMap &   gpfm,
	              GroundedPDDLPredicateMap & gppm);

	std::shared_ptr<PDDLGrounding> gen_pddl_grounding(const CLIPS::Fact::pointer fact);

	std::shared_ptr<PDDLFormula> gen_pddl_formula(const CLIPS::Fact::pointer fact);

	std::shared_ptr<PDDLPredicate> gen_pddl_predicate(const CLIPS::Fact::pointer fact);

	std::shared_ptr<GroundedPDDLFormula> gen_grounded_pddl_formula(const CLIPS::Fact::pointer fact);

	std::shared_ptr<GroundedPDDLPredicate>
	gen_grounded_pddl_predicate(const CLIPS::Fact::pointer fact);

private:
	fawkes::WebviewRestApi *                     rest_api_;
	fawkes::RecursiveLockPtr<CLIPS::Environment> clips_;
};
