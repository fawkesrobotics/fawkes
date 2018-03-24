/****************************************************************************
 *  CLIPS Executive REST API -- Schema DomainPrecondition
 *  (auto-generated, do not modify directly)
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

	export type TypeEnum = 'conjunction' | 'disjunction' | 'atom';
	export const TypeEnum = {
		conjunction: 'conjunction' as TypeEnum,
		disjunction: 'disjunction' as TypeEnum,
		atom: 'atom' as TypeEnum
	}
}