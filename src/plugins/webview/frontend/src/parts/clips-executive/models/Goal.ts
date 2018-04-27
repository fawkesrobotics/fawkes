/****************************************************************************
 *  ClipsExecutive -- Schema Goal
 *  (auto-generated, do not modify directly)
 *
 *  CLIPS Executive REST API.
 *  Enables access to goals, plans, and all items in the domain model.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/



/** Goal representation for JSON transfer. */
export interface Goal
{
	kind: string;
	apiVersion: string;
	id: string;
	type: Goal.TypeEnum;
	_class: string;
	mode: Goal.ModeEnum;
	outcome?: Goal.OutcomeEnum;
	message?: string;
	parent?: string;
	priority?: number;
	parameters?: Array<string>;
	plans?: Array<string>;
}

export namespace Goal
{
	export const API_VERSION: string = "v1beta1";

	export type TypeEnum = 'ACHIEVE' | 'MAINTAIN';
	export const TypeEnum = {
		ACHIEVE: 'ACHIEVE' as TypeEnum,
		MAINTAIN: 'MAINTAIN' as TypeEnum
	}
	export type ModeEnum = 'FORMULATED' | 'SELECTED' | 'EXPANDED' | 'COMMITTED' | 'DISPATCHED' | 'FINISHED' | 'EVALUATED' | 'REJECTED';
	export const ModeEnum = {
		FORMULATED: 'FORMULATED' as ModeEnum,
		SELECTED: 'SELECTED' as ModeEnum,
		EXPANDED: 'EXPANDED' as ModeEnum,
		COMMITTED: 'COMMITTED' as ModeEnum,
		DISPATCHED: 'DISPATCHED' as ModeEnum,
		FINISHED: 'FINISHED' as ModeEnum,
		EVALUATED: 'EVALUATED' as ModeEnum,
		REJECTED: 'REJECTED' as ModeEnum
	}
	export type OutcomeEnum = 'UNKNOWN' | 'COMPLETED' | 'FAILED';
	export const OutcomeEnum = {
		UNKNOWN: 'UNKNOWN' as OutcomeEnum,
		COMPLETED: 'COMPLETED' as OutcomeEnum,
		FAILED: 'FAILED' as OutcomeEnum
	}
}