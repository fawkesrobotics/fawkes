/****************************************************************************
 *  ClipsExecutive -- Schema DomainPrecondition
 *  (auto-generated, do not modify directly)
 *
 *  CLIPS Executive REST API.
 *  Enables access to goals, plans, and all items in the domain model.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/



/** DomainPrecondition representation for JSON transfer. */
export interface DomainPrecondition
{
	kind: string;
	apiVersion: string;
	name: string;
	type: DomainPrecondition.TypeEnum;
	grounded: boolean;
	is_satisfied: boolean;
}

export namespace DomainPrecondition
{
	export const API_VERSION: string = "v1beta1";

	export type TypeEnum = 'conjunction' | 'disjunction' | 'negation' | 'atom';
	export const TypeEnum = {
		conjunction: 'conjunction' as TypeEnum,
		disjunction: 'disjunction' as TypeEnum,
		negation: 'negation' as TypeEnum,
		atom: 'atom' as TypeEnum
	}
}