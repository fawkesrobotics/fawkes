/****************************************************************************
 *  ClipsExecutive -- Schema GroundedFormula
 *  (auto-generated, do not modify directly)
 *
 *  CLIPS Executive REST API.
 *  Enables access to goals, plans, and all items in the domain model.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/


/** GroundedFormula representation for JSON transfer. */
export interface GroundedFormula {
  kind: string;
  apiVersion: string;
  name: string;
  type: GroundedFormula.TypeEnum;
  is_satisfied: boolean;
  param_names?: Array<string>;
  param_values?: Array<string>;
  param_constants?: Array<string>;
  child?: Array<GroundedFormula>;
}

export namespace GroundedFormula {
  export const API_VERSION = 'v1beta1';

  // tslint:disable-next-line:max-line-length
  export type TypeEnum = 'conjunction' | 'disjunction' | 'negation' | 'atom';
  export const TypeEnum = {
    conjunction: 'conjunction' as TypeEnum,
    disjunction: 'disjunction' as TypeEnum,
    negation: 'negation' as TypeEnum,
    atom: 'atom' as TypeEnum
  };
}
