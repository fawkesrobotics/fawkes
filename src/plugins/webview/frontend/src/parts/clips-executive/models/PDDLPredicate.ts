/****************************************************************************
 *  ClipsExecutive -- Schema PDDLPredicate
 *  (auto-generated, do not modify directly)
 *
 *  CLIPS Executive REST API.
 *  Enables access to goals, plans, and all items in the domain model.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/



/** PDDLPredicate representation for JSON transfer. */
export interface PDDLPredicate {
  kind: string;
  apiVersion: string;
  id: string;
  part_of: string;
  predicate: string;
  param_names: Array<string>;
  param_constants: Array<string>;
}

export namespace PDDLPredicate {
  export const API_VERSION = 'v1beta1';

}
