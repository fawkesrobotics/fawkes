/****************************************************************************
 *  Clips -- Schema Fact
 *  (auto-generated, do not modify directly)
 *
 *  CLIPS REST API.
 *  Enables access to CLIPS environments.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/

import { SlotValue } from './SlotValue';


/** Fact representation for JSON transfer. */
export interface Fact
{
	kind: string;
	apiVersion: string;
	index: number;
	template_name: string;
	formatted?: string;
	slots?: Array<SlotValue>;
}

export namespace Fact
{
	export const API_VERSION: string = "v1beta1";

}