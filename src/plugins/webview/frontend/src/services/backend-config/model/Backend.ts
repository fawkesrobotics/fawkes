/****************************************************************************
 *  BackendInfo -- Schema Backend
 *  (auto-generated, do not modify directly)
 *
 *  Fawkes Backend Info REST API.
 *  Provides backend meta information to the frontend.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/

import { Service } from './Service';


/** Backend representation for JSON transfer. */
export interface Backend
{
	kind: string;
	apiVersion: string;
	id: string;
	name: string;
	url?: string;
	services: Array<Service>;
}

export namespace Backend
{
	export const API_VERSION: string = "v1beta1";

}