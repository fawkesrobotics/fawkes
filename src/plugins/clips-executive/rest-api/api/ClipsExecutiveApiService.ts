
/****************************************************************************
 *  ClipsExecutive -- API Service
 *  (auto-generated, do not modify directly)
 *
 *  CLIPS Executive REST API.
 *  Enables access to goals, plans, and all items in the domain model.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/

import { Injectable } from '@angular/core';
import { HttpClient, HttpHeaders, HttpParams } from '@angular/common/http';
import { Observable } from 'rxjs';

import { BackendConfigurationService } from '../../../services/backend-config/backend-config.service';

import { DomainFact } from '../models/DomainFact';
import { DomainObject } from '../models/DomainObject';
import { DomainOperator } from '../models/DomainOperator';
import { DomainPredicate } from '../models/DomainPredicate';
import { Goal } from '../models/Goal';
import { GroundedPDDLFormula } from '../models/GroundedPDDLFormula';
import { GroundedPDDLPredicate } from '../models/GroundedPDDLPredicate';
import { PDDLFormula } from '../models/PDDLFormula';
import { PDDLGrounding } from '../models/PDDLGrounding';
import { PDDLPredicate } from '../models/PDDLPredicate';
import { Plan } from '../models/Plan';


@Injectable()
export class ClipsExecutiveApiService {
  constructor(private backend: BackendConfigurationService,
              private http: HttpClient) {}

  public list_goals(pretty?: boolean): Observable<Goal[]> {
    // tslint:disable-next-line:prefer-const
    let params = new HttpParams();
    if (pretty) {
      params = params.set('pretty', pretty.toString());
    }
    // tslint:disable-next-line:prefer-const
    let headers = new HttpHeaders();

    headers = headers.set('Accept', 'application/json');
    // tslint:disable-next-line:max-line-length
    return this.http.get<Goal[]>(`${this.backend.url_for('api')}/clips-executive/goals`,
      { headers: headers, params: params,
        observe: 'body', responseType: 'json' });
      }

  public get_goal(id: string, pretty?: boolean): Observable<Goal> {
    if (id === null || id === undefined) {
      throw new Error('Required parameter id is null or undefined (get_goal)');
    }
    // tslint:disable-next-line:prefer-const
    let params = new HttpParams();
    if (pretty) {
      params = params.set('pretty', pretty.toString());
    }
    // tslint:disable-next-line:prefer-const
    let headers = new HttpHeaders();

    headers = headers.set('Accept', 'application/json');
    // tslint:disable-next-line:max-line-length
    return this.http.get<Goal>(`${this.backend.url_for('api')}/clips-executive/goals/${encodeURIComponent(String(id))}`,
      { headers: headers, params: params,
        observe: 'body', responseType: 'json' });
      }

  public list_plans(pretty?: boolean): Observable<Plan[]> {
    // tslint:disable-next-line:prefer-const
    let params = new HttpParams();
    if (pretty) {
      params = params.set('pretty', pretty.toString());
    }
    // tslint:disable-next-line:prefer-const
    let headers = new HttpHeaders();

    headers = headers.set('Accept', 'application/json');
    // tslint:disable-next-line:max-line-length
    return this.http.get<Plan[]>(`${this.backend.url_for('api')}/clips-executive/plans`,
      { headers: headers, params: params,
        observe: 'body', responseType: 'json' });
      }

  public get_plan(goal_id: string, id: string, pretty?: boolean): Observable<Plan> {
    if (goal_id === null || goal_id === undefined) {
      throw new Error('Required parameter goal-id is null or undefined (get_plan)');
    }
    if (id === null || id === undefined) {
      throw new Error('Required parameter id is null or undefined (get_plan)');
    }
    // tslint:disable-next-line:prefer-const
    let params = new HttpParams();
    if (pretty) {
      params = params.set('pretty', pretty.toString());
    }
    // tslint:disable-next-line:prefer-const
    let headers = new HttpHeaders();

    headers = headers.set('Accept', 'application/json');
    // tslint:disable-next-line:max-line-length
    return this.http.get<Plan>(`${this.backend.url_for('api')}/clips-executive/plans/${encodeURIComponent(String(goal_id))}/${encodeURIComponent(String(id))}`,
      { headers: headers, params: params,
        observe: 'body', responseType: 'json' });
      }

  public list_domain_objects(pretty?: boolean): Observable<DomainObject[]> {
    // tslint:disable-next-line:prefer-const
    let params = new HttpParams();
    if (pretty) {
      params = params.set('pretty', pretty.toString());
    }
    // tslint:disable-next-line:prefer-const
    let headers = new HttpHeaders();

    headers = headers.set('Accept', 'application/json');
    // tslint:disable-next-line:max-line-length
    return this.http.get<DomainObject[]>(`${this.backend.url_for('api')}/clips-executive/domain-objects`,
      { headers: headers, params: params,
        observe: 'body', responseType: 'json' });
      }

  public list_domain_predicates(pretty?: boolean): Observable<DomainPredicate[]> {
    // tslint:disable-next-line:prefer-const
    let params = new HttpParams();
    if (pretty) {
      params = params.set('pretty', pretty.toString());
    }
    // tslint:disable-next-line:prefer-const
    let headers = new HttpHeaders();

    headers = headers.set('Accept', 'application/json');
    // tslint:disable-next-line:max-line-length
    return this.http.get<DomainPredicate[]>(`${this.backend.url_for('api')}/clips-executive/domain-predicates`,
      { headers: headers, params: params,
        observe: 'body', responseType: 'json' });
      }

  public list_domain_facts(pretty?: boolean): Observable<DomainFact[]> {
    // tslint:disable-next-line:prefer-const
    let params = new HttpParams();
    if (pretty) {
      params = params.set('pretty', pretty.toString());
    }
    // tslint:disable-next-line:prefer-const
    let headers = new HttpHeaders();

    headers = headers.set('Accept', 'application/json');
    // tslint:disable-next-line:max-line-length
    return this.http.get<DomainFact[]>(`${this.backend.url_for('api')}/clips-executive/domain-facts`,
      { headers: headers, params: params,
        observe: 'body', responseType: 'json' });
      }

  public list_domain_operators(pretty?: boolean): Observable<DomainOperator[]> {
    // tslint:disable-next-line:prefer-const
    let params = new HttpParams();
    if (pretty) {
      params = params.set('pretty', pretty.toString());
    }
    // tslint:disable-next-line:prefer-const
    let headers = new HttpHeaders();

    headers = headers.set('Accept', 'application/json');
    // tslint:disable-next-line:max-line-length
    return this.http.get<DomainOperator[]>(`${this.backend.url_for('api')}/clips-executive/domain-operators`,
      { headers: headers, params: params,
        observe: 'body', responseType: 'json' });
      }

  public list_pddl_groundings(pretty?: boolean): Observable<PDDLGrounding[]> {
    // tslint:disable-next-line:prefer-const
    let params = new HttpParams();
    if (pretty) {
      params = params.set('pretty', pretty.toString());
    }
    // tslint:disable-next-line:prefer-const
    let headers = new HttpHeaders();

    headers = headers.set('Accept', 'application/json');
    // tslint:disable-next-line:max-line-length
    return this.http.get<PDDLGrounding[]>(`${this.backend.url_for('api')}/clips-executive/pddl-groundings`,
      { headers: headers, params: params,
        observe: 'body', responseType: 'json' });
      }

  public get_pddl_groundings(id: string, pretty?: boolean): Observable<PDDLGrounding> {
    if (id === null || id === undefined) {
      throw new Error('Required parameter id is null or undefined (get_pddl_groundings)');
    }
    // tslint:disable-next-line:prefer-const
    let params = new HttpParams();
    if (pretty) {
      params = params.set('pretty', pretty.toString());
    }
    // tslint:disable-next-line:prefer-const
    let headers = new HttpHeaders();

    headers = headers.set('Accept', 'application/json');
    // tslint:disable-next-line:max-line-length
    return this.http.get<PDDLGrounding>(`${this.backend.url_for('api')}/clips-executive/pddl-groundings/${encodeURIComponent(String(id))}`,
      { headers: headers, params: params,
        observe: 'body', responseType: 'json' });
      }

  public list_pddl_formulas(pretty?: boolean): Observable<PDDLFormula[]> {
    // tslint:disable-next-line:prefer-const
    let params = new HttpParams();
    if (pretty) {
      params = params.set('pretty', pretty.toString());
    }
    // tslint:disable-next-line:prefer-const
    let headers = new HttpHeaders();

    headers = headers.set('Accept', 'application/json');
    // tslint:disable-next-line:max-line-length
    return this.http.get<PDDLFormula[]>(`${this.backend.url_for('api')}/clips-executive/pddl-formulas`,
      { headers: headers, params: params,
        observe: 'body', responseType: 'json' });
      }

  public get_pddl_formulas(id: string, pretty?: boolean): Observable<PDDLFormula> {
    if (id === null || id === undefined) {
      throw new Error('Required parameter id is null or undefined (get_pddl_formulas)');
    }
    // tslint:disable-next-line:prefer-const
    let params = new HttpParams();
    if (pretty) {
      params = params.set('pretty', pretty.toString());
    }
    // tslint:disable-next-line:prefer-const
    let headers = new HttpHeaders();

    headers = headers.set('Accept', 'application/json');
    // tslint:disable-next-line:max-line-length
    return this.http.get<PDDLFormula>(`${this.backend.url_for('api')}/clips-executive/pddl-formulas/${encodeURIComponent(String(id))}`,
      { headers: headers, params: params,
        observe: 'body', responseType: 'json' });
      }

  public list_pddl_predicates(pretty?: boolean): Observable<PDDLPredicate[]> {
    // tslint:disable-next-line:prefer-const
    let params = new HttpParams();
    if (pretty) {
      params = params.set('pretty', pretty.toString());
    }
    // tslint:disable-next-line:prefer-const
    let headers = new HttpHeaders();

    headers = headers.set('Accept', 'application/json');
    // tslint:disable-next-line:max-line-length
    return this.http.get<PDDLPredicate[]>(`${this.backend.url_for('api')}/clips-executive/pddl-predicates`,
      { headers: headers, params: params,
        observe: 'body', responseType: 'json' });
      }

  public get_pddl_predicates(id: string, pretty?: boolean): Observable<PDDLPredicate> {
    if (id === null || id === undefined) {
      throw new Error('Required parameter id is null or undefined (get_pddl_predicates)');
    }
    // tslint:disable-next-line:prefer-const
    let params = new HttpParams();
    if (pretty) {
      params = params.set('pretty', pretty.toString());
    }
    // tslint:disable-next-line:prefer-const
    let headers = new HttpHeaders();

    headers = headers.set('Accept', 'application/json');
    // tslint:disable-next-line:max-line-length
    return this.http.get<PDDLPredicate>(`${this.backend.url_for('api')}/clips-executive/pddl-predicates/${encodeURIComponent(String(id))}`,
      { headers: headers, params: params,
        observe: 'body', responseType: 'json' });
      }

  public list_grounded_pddl_formulas(pretty?: boolean): Observable<GroundedPDDLFormula[]> {
    // tslint:disable-next-line:prefer-const
    let params = new HttpParams();
    if (pretty) {
      params = params.set('pretty', pretty.toString());
    }
    // tslint:disable-next-line:prefer-const
    let headers = new HttpHeaders();

    headers = headers.set('Accept', 'application/json');
    // tslint:disable-next-line:max-line-length
    return this.http.get<GroundedPDDLFormula[]>(`${this.backend.url_for('api')}/clips-executive/grounded-pddl-formulas`,
      { headers: headers, params: params,
        observe: 'body', responseType: 'json' });
      }

  public get_grounded_pddl_formulas(id: string, pretty?: boolean): Observable<GroundedPDDLFormula> {
    if (id === null || id === undefined) {
      throw new Error('Required parameter id is null or undefined (get_grounded_pddl_formulas)');
    }
    // tslint:disable-next-line:prefer-const
    let params = new HttpParams();
    if (pretty) {
      params = params.set('pretty', pretty.toString());
    }
    // tslint:disable-next-line:prefer-const
    let headers = new HttpHeaders();

    headers = headers.set('Accept', 'application/json');
    // tslint:disable-next-line:max-line-length
    return this.http.get<GroundedPDDLFormula>(`${this.backend.url_for('api')}/clips-executive/grounded-pddl-formulas/${encodeURIComponent(String(id))}`,
      { headers: headers, params: params,
        observe: 'body', responseType: 'json' });
      }

  public list_grounded_pddl_predicates(pretty?: boolean): Observable<GroundedPDDLPredicate[]> {
    // tslint:disable-next-line:prefer-const
    let params = new HttpParams();
    if (pretty) {
      params = params.set('pretty', pretty.toString());
    }
    // tslint:disable-next-line:prefer-const
    let headers = new HttpHeaders();

    headers = headers.set('Accept', 'application/json');
    // tslint:disable-next-line:max-line-length
    return this.http.get<GroundedPDDLPredicate[]>(`${this.backend.url_for('api')}/clips-executive/grounded-pddl-predicates`,
      { headers: headers, params: params,
        observe: 'body', responseType: 'json' });
      }

  public get_grounded_pddl_predicates(id: string, pretty?: boolean): Observable<GroundedPDDLPredicate> {
    if (id === null || id === undefined) {
      throw new Error('Required parameter id is null or undefined (get_grounded_pddl_predicates)');
    }
    // tslint:disable-next-line:prefer-const
    let params = new HttpParams();
    if (pretty) {
      params = params.set('pretty', pretty.toString());
    }
    // tslint:disable-next-line:prefer-const
    let headers = new HttpHeaders();

    headers = headers.set('Accept', 'application/json');
    // tslint:disable-next-line:max-line-length
    return this.http.get<GroundedPDDLPredicate>(`${this.backend.url_for('api')}/clips-executive/grounded-pddl-predicates/${encodeURIComponent(String(id))}`,
      { headers: headers, params: params,
        observe: 'body', responseType: 'json' });
      }

}
