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
export interface PlanAction {
  kind: string;
  apiVersion: string;
  id: number;
  operator_name: string;
  param_values: Array<string>;
  duration?: number;
  dispatch_time?: number;
  state: PlanAction.StateEnum;
  executable: boolean;
  _operator?: DomainOperator;
  preconditions: Array<DomainPrecondition | DomainPreconditionAtom | DomainPreconditionCompound>;
  effects: Array<DomainEffect>;
}

export namespace PlanAction {
  export const API_VERSION = 'v1beta1';

  // tslint:disable-next-line:max-line-length
  export type StateEnum = 'FORMULATED' | 'PENDING' | 'WAITING' | 'RUNNING' | 'EXECUTION-SUCCEEDED' | 'SENSED-EFFECTS-WAIT' | 'SENSED-EFFECTS-HOLD' | 'EFFECTS-APPLIED' | 'FINAL' | 'EXECUTION-FAILED' | 'FAILED';
  export const StateEnum = {
    FORMULATED: 'FORMULATED' as StateEnum,
    PENDING: 'PENDING' as StateEnum,
    WAITING: 'WAITING' as StateEnum,
    RUNNING: 'RUNNING' as StateEnum,
    EXECUTION_SUCCEEDED: 'EXECUTION-SUCCEEDED' as StateEnum,
    SENSED_EFFECTS_WAIT: 'SENSED-EFFECTS-WAIT' as StateEnum,
    SENSED_EFFECTS_HOLD: 'SENSED-EFFECTS-HOLD' as StateEnum,
    EFFECTS_APPLIED: 'EFFECTS-APPLIED' as StateEnum,
    FINAL: 'FINAL' as StateEnum,
    EXECUTION_FAILED: 'EXECUTION-FAILED' as StateEnum,
    FAILED: 'FAILED' as StateEnum
  };
}
