/****************************************************************************
 *  CLIPS Executive REST API -- Schema Plan
 *  (auto-generated, do not modify directly)
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/

import { PlanAction } from './PlanAction';


/** Plan representation for JSON transfer. */
export interface Plan
{
	kind: string;
	apiVersion: string;
	id: string;
	goal_id: string;
	cost?: number;
	actions: Array<PlanAction>;
}

export namespace Plan
{
	export const API_VERSION: string = "v1beta1";

}