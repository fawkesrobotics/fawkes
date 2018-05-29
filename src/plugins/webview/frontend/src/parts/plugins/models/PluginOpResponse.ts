/****************************************************************************
 *  Plugin -- Schema PluginOpResponse
 *  (auto-generated, do not modify directly)
 *
 *  Fawkes Plugin REST API.
 *  List, load, and unload plugins.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/



/** PluginOpResponse representation for JSON transfer. */
export interface PluginOpResponse
{
	kind: string;
	apiVersion: string;
	name: string;
	state: PluginOpResponse.StateEnum;
	message?: string;
}

export namespace PluginOpResponse
{
	export const API_VERSION: string = "v1beta1";

	export type StateEnum = 'LOADED' | 'AVAILABLE' | 'UNLOADED' | 'ERROR';
	export const StateEnum = {
		LOADED: 'LOADED' as StateEnum,
		AVAILABLE: 'AVAILABLE' as StateEnum,
		UNLOADED: 'UNLOADED' as StateEnum,
		ERROR: 'ERROR' as StateEnum
	}
}