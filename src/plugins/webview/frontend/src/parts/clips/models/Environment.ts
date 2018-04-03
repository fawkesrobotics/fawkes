/****************************************************************************
 *  Clips -- Schema Environment
 *  (auto-generated, do not modify directly)
 *
 *  CLIPS REST API.
 *  Enables access to CLIPS environments.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/



/** Environment representation for JSON transfer. */
export interface Environment
{
	kind: string;
	apiVersion: string;
	name: string;
}

export namespace Environment
{
	export const API_VERSION: string = "v1beta1";

}