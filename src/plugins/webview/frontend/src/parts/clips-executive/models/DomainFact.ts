/****************************************************************************
 *  CLIPS Executive REST API -- Schema DomainFact
 *  (auto-generated, do not modify directly)
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/



/** DomainFact representation for JSON transfer. */
export interface DomainFact
{
	kind: string;
	apiVersion: string;
	name: string;
	param_values: Array<string>;
}

export namespace DomainFact
{
	export const API_VERSION: string = "v1beta1";

}