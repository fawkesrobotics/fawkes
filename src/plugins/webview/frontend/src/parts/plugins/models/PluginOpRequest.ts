/****************************************************************************
 *  Plugin -- Schema PluginOpRequest
 *  (auto-generated, do not modify directly)
 *
 *  Fawkes Plugin REST API.
 *  List, load, and unload plugins.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/



/** PluginOpRequest representation for JSON transfer. */
export interface PluginOpRequest
{
	kind: string;
	apiVersion: string;
	desired_state: PluginOpRequest.Desired_stateEnum;
}

export namespace PluginOpRequest
{
	export const API_VERSION: string = "v1beta1";

	export type Desired_stateEnum = 'LOADED' | 'AVAILABLE' | 'UNLOADED';
	export const Desired_stateEnum = {
		LOADED: 'LOADED' as Desired_stateEnum,
		AVAILABLE: 'AVAILABLE' as Desired_stateEnum,
		UNLOADED: 'UNLOADED' as Desired_stateEnum
	}
}