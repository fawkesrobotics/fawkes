/****************************************************************************
 *  BackendInfo -- Schema Service
 *  (auto-generated, do not modify directly)
 *
 *  Fawkes Backend Info REST API.
 *  Provides backend meta information to the frontend.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/



/** Service representation for JSON transfer. */
export interface Service
{
	name: string;
	url: string;
}

export namespace Service
{
	export const API_VERSION: string = "v1beta1";

}