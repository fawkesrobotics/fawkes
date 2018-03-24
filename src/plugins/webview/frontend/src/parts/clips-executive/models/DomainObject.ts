/****************************************************************************
 *  CLIPS Executive REST API -- Schema DomainObject
 *  (auto-generated, do not modify directly)
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/



/** DomainObject representation for JSON transfer. */
export interface DomainObject
{
	kind: string;
	apiVersion: string;
	name: string;
	type: string;
}

export namespace DomainObject
{
	export const API_VERSION: string = "v1beta1";

}