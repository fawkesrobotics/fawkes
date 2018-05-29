/****************************************************************************
 *  ClipsExecutive -- Schema PlanAction
 *  (auto-generated, do not modify directly)
 *
 *  CLIPS Executive REST API.
 *  Enables access to goals, plans, and all items in the domain model.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/

import { DomainOperator } from './DomainOperator';
import { DomainPrecondition } from './DomainPrecondition';
import { DomainPreconditionAtom } from './DomainPreconditionAtom';
import { DomainPreconditionCompound } from './DomainPreconditionCompound';
import { DomainEffect } from './DomainEffect';


/** PlanAction representation for JSON transfer. */
export interface PlanAction
{
	kind: string;
	apiVersion: string;
	id: number;
	operator_name: string;
	param_values: Array<string>;
	duration?: number;
	dispatch_time?: number;
	status: PlanAction.StatusEnum;
	executable: boolean;
	_operator?: DomainOperator;
	preconditions: Array<DomainPrecondition | DomainPreconditionAtom | DomainPreconditionCompound>;
	effects: Array<DomainEffect>;
}

export namespace PlanAction
{
	export const API_VERSION: string = "v1beta1";

	export type StatusEnum = 'FORMULATED' | 'PENDING' | 'WAITING' | 'RUNNING' | 'EXECUTION-SUCCEEDED' | 'SENSED-EFFECTS-WAIT' | 'SENSED-EFFECTS-HOLD' | 'EFFECTS-APPLIED' | 'FINAL' | 'EXECUTION-FAILED' | 'FAILED';
	export const StatusEnum = {
		FORMULATED: 'FORMULATED' as StatusEnum,
		PENDING: 'PENDING' as StatusEnum,
		WAITING: 'WAITING' as StatusEnum,
		RUNNING: 'RUNNING' as StatusEnum,
		EXECUTION_SUCCEEDED: 'EXECUTION-SUCCEEDED' as StatusEnum,
		SENSED_EFFECTS_WAIT: 'SENSED-EFFECTS-WAIT' as StatusEnum,
		SENSED_EFFECTS_HOLD: 'SENSED-EFFECTS-HOLD' as StatusEnum,
		EFFECTS_APPLIED: 'EFFECTS-APPLIED' as StatusEnum,
		FINAL: 'FINAL' as StatusEnum,
		EXECUTION_FAILED: 'EXECUTION-FAILED' as StatusEnum,
		FAILED: 'FAILED' as StatusEnum
	}
}