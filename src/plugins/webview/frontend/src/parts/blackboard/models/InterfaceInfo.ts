/****************************************************************************
 *  Blackboard -- Schema InterfaceInfo
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
import { InterfaceMessageType } from './InterfaceMessageType';


/** InterfaceInfo representation for JSON transfer. */
export interface InterfaceInfo
{
	kind: string;
	apiVersion: string;
	id: string;
	type: string;
	hash: string;
	writer?: string;
	readers?: Array<string>;
	fields: Array<InterfaceFieldType>;
	message_types: Array<InterfaceMessageType>;
}

export namespace InterfaceInfo
{
	export const API_VERSION: string = "v1beta1";

}