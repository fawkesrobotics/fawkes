/****************************************************************************
 *  ClipsExecutive -- Schema PDDLGrounding
 *  (auto-generated, do not modify directly)
 *
 *  CLIPS Executive REST API.
 *  Enables access to goals, plans, and all items in the domain model.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/



/** PDDLGrounding representation for JSON transfer. */
export interface PDDLGrounding {
  kind: string;
  apiVersion: string;
  id: string;
  param_names: Array<string>;
  param_values: Array<string>;
}

export namespace PDDLGrounding {
  export const API_VERSION = 'v1beta1';

}
