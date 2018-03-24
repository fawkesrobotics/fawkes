/****************************************************************************
 *  CLIPS Executive REST API -- Schema DomainPreconditionAtom
 *  (auto-generated, do not modify directly)
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/



/** DomainPreconditionAtom representation for JSON transfer. */
export interface DomainPreconditionAtom
{
	kind: string;
	apiVersion: string;
	name: string;
	type: DomainPreconditionAtom.TypeEnum;
	grounded: boolean;
	is_satisfied: boolean;
	predicate: string;
	param_names: Array<string>;
	param_values: Array<string>;
	param_constants: Array<string>;
}

export namespace DomainPreconditionAtom
{
	export const API_VERSION: string = "v1beta1";

	export type TypeEnum = 'conjunction' | 'disjunction' | 'atom';
	export const TypeEnum = {
		conjunction: 'conjunction' as TypeEnum,
		disjunction: 'disjunction' as TypeEnum,
		atom: 'atom' as TypeEnum
	}
}