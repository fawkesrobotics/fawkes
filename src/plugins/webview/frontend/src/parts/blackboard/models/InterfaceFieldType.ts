/****************************************************************************
 *  Blackboard -- Schema InterfaceFieldType
 *  (auto-generated, do not modify directly)
 *
 *  Fawkes Blackboard REST API.
 *  Access blackboard data through a REST API.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/



/** InterfaceFieldType representation for JSON transfer. */
export interface InterfaceFieldType
{
	name: string;
	type: string;
	is_array?: boolean;
	enums?: Array<string>;
}

export namespace InterfaceFieldType
{
	export const API_VERSION: string = "v1beta1";

}