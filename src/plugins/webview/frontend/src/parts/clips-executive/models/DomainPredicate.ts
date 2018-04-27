/****************************************************************************
 *  ClipsExecutive -- Schema DomainPredicate
 *  (auto-generated, do not modify directly)
 *
 *  CLIPS Executive REST API.
 *  Enables access to goals, plans, and all items in the domain model.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/



/** DomainPredicate representation for JSON transfer. */
export interface DomainPredicate
{
	kind: string;
	apiVersion: string;
	name: string;
	sensed: boolean;
	param_names: Array<string>;
	param_types: Array<string>;
}

export namespace DomainPredicate
{
	export const API_VERSION: string = "v1beta1";

}