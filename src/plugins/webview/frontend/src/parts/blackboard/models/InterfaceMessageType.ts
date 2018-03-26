/****************************************************************************
 *  Blackboard -- Schema InterfaceMessageType
 *  (auto-generated, do not modify directly)
 *
 *  Fawkes Blackboard REST API.
 *  Access blackboard data through a REST API.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/

import { InterfaceFieldType } from './InterfaceFieldType';


/** InterfaceMessageType representation for JSON transfer. */
export interface InterfaceMessageType
{
	name: string;
	fields: Array<InterfaceFieldType>;
}

export namespace InterfaceMessageType
{
	export const API_VERSION: string = "v1beta1";

}