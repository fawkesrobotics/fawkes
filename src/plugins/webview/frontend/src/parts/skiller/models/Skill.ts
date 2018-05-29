/****************************************************************************
 *  BehaviorEngine -- Schema Skill
 *  (auto-generated, do not modify directly)
 *
 *  Behavior Engine REST API.
 *  Visualize, monitor, and instruct the Skill Execution Run-Time of
 *  the Lua-based Behavior Engine.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/



/** Skill representation for JSON transfer. */
export interface Skill
{
	kind: string;
	apiVersion: string;
	name: string;
	graph?: string;
	skill_string?: string;
	error?: string;
	msg_id?: number;
	exclusive_controller?: number;
	status?: Skill.StatusEnum;
}

export namespace Skill
{
	export const API_VERSION: string = "v1beta1";

	export type StatusEnum = 'INACTIVE' | 'FINAL' | 'RUNNING' | 'FAILED';
	export const StatusEnum = {
		INACTIVE: 'INACTIVE' as StatusEnum,
		FINAL: 'FINAL' as StatusEnum,
		RUNNING: 'RUNNING' as StatusEnum,
		FAILED: 'FAILED' as StatusEnum
	}
}