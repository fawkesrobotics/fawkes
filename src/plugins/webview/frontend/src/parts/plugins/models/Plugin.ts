/****************************************************************************
 *  Plugin -- Schema Plugin
 *  (auto-generated, do not modify directly)
 *
 *  Fawkes Plugin REST API.
 *  List, load, and unload plugins.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/



/** Plugin representation for JSON transfer. */
export interface Plugin
{
	kind: string;
	apiVersion: string;
	name: string;
	description: string;
	is_meta: boolean;
	meta_children?: Array<string>;
	is_loaded: boolean;
}

export namespace Plugin
{
	export const API_VERSION: string = "v1beta1";

}