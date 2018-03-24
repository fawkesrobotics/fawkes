/****************************************************************************
 *  CLIPS Executive REST API -- Schema DomainObjectType
 *  (auto-generated, do not modify directly)
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/



/** DomainObjectType representation for JSON transfer. */
export interface DomainObjectType
{
	kind: string;
	apiVersion: string;
	name: string;
	super_type: string;
}

export namespace DomainObjectType
{
	export const API_VERSION: string = "v1beta1";

}