/****************************************************************************
 *  ClipsExecutive -- Schema GroundedPDDLFormula
 *  (auto-generated, do not modify directly)
 *
 *  CLIPS Executive REST API.
 *  Enables access to goals, plans, and all items in the domain model.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/



/** GroundedPDDLFormula representation for JSON transfer. */
export interface GroundedPDDLFormula {
  kind: string;
  apiVersion: string;
  id: string;
  formula_id: string;
  grounding: string;
  is_satisfied: boolean;
}

export namespace GroundedPDDLFormula {
  export const API_VERSION = 'v1beta1';

}
