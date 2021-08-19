/****************************************************************************
 *  ClipsExecutive -- Schema PDDLFormula
 *  (auto-generated, do not modify directly)
 *
 *  CLIPS Executive REST API.
 *  Enables access to goals, plans, and all items in the domain model.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/



/** PDDLFormula representation for JSON transfer. */
export interface PDDLFormula {
  kind: string;
  apiVersion: string;
  id: string;
  type: PDDLFormula.TypeEnum;
  part_of: string;
}

export namespace PDDLFormula {
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
