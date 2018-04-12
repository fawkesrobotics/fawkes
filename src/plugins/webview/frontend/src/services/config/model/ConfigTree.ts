/****************************************************************************
 *  Configuration -- Schema ConfigTree
 *  (auto-generated, do not modify directly)
 *
 *  Fawkes Configuration REST API.
 *  Retrieve information from the configuration.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/



/** ConfigTree representation for JSON transfer. */
export interface ConfigTree
{
	kind: string;
	apiVersion: string;
	config: object;
}

export namespace ConfigTree
{
	export const API_VERSION: string = "v1beta1";

}