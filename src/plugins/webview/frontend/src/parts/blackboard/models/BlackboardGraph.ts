/****************************************************************************
 *  Blackboard -- Schema BlackboardGraph
 *  (auto-generated, do not modify directly)
 *
 *  Fawkes Blackboard REST API.
 *  Access blackboard data through a REST API.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/



/** BlackboardGraph representation for JSON transfer. */
export interface BlackboardGraph
{
	kind: string;
	apiVersion: string;
	dotgraph: string;
}

export namespace BlackboardGraph
{
	export const API_VERSION: string = "v1beta1";

}