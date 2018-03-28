/****************************************************************************
 *  Blackboard -- Schema InterfaceData
 *  (auto-generated, do not modify directly)
 *
 *  Fawkes Blackboard REST API.
 *  Access blackboard data through a REST API.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/



/** InterfaceData representation for JSON transfer. */
export interface InterfaceData
{
	kind: string;
	apiVersion: string;
	id: string;
	type: string;
	writer?: string;
	readers?: Array<string>;
	data: object;
	timestamp: string;
}

export namespace InterfaceData
{
	export const API_VERSION: string = "v1beta1";

}