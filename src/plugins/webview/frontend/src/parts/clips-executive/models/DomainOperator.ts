/****************************************************************************
 *  ClipsExecutive -- Schema DomainOperator
 *  (auto-generated, do not modify directly)
 *
 *  CLIPS Executive REST API.
 *  Enables access to goals, plans, and all items in the domain model.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/

import { DomainOperatorParameter } from './DomainOperatorParameter';


/** DomainOperator representation for JSON transfer. */
export interface DomainOperator
{
	kind: string;
	apiVersion: string;
	name: string;
	wait_sensed: boolean;
	parameters: Array<DomainOperatorParameter>;
}

export namespace DomainOperator
{
	export const API_VERSION: string = "v1beta1";

}