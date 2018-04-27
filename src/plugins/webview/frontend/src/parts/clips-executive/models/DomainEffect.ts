/****************************************************************************
 *  ClipsExecutive -- Schema DomainEffect
 *  (auto-generated, do not modify directly)
 *
 *  CLIPS Executive REST API.
 *  Enables access to goals, plans, and all items in the domain model.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/



/** DomainEffect representation for JSON transfer. */
export interface DomainEffect
{
	kind: string;
	apiVersion: string;
	name: string;
	type: DomainEffect.TypeEnum;
	predicate: string;
	param_names: Array<string>;
	param_values: Array<string>;
	param_constants: Array<string>;
}

export namespace DomainEffect
{
	export const API_VERSION: string = "v1beta1";

	export type TypeEnum = 'POSITIVE' | 'NEGATIVE';
	export const TypeEnum = {
		POSITIVE: 'POSITIVE' as TypeEnum,
		NEGATIVE: 'NEGATIVE' as TypeEnum
	}
}