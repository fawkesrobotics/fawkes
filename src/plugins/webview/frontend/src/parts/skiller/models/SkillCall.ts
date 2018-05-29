/****************************************************************************
 *  BehaviorEngine -- Schema SkillCall
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



/** SkillCall representation for JSON transfer. */
export interface SkillCall
{
	kind: string;
	apiVersion: string;
	skill_string: string;
}

export namespace SkillCall
{
	export const API_VERSION: string = "v1beta1";

}