
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
import { HttpClient, HttpHeaders, HttpParams, HttpResponse } from '@angular/common/http';
import { Observable } from 'rxjs';

import { BackendConfigurationService } from '../../../services/backend-config/backend-config.service';

import { DomainFact } from '../models/DomainFact';
import { DomainObject } from '../models/DomainObject';
import { DomainOperator } from '../models/DomainOperator';
import { DomainPredicate } from '../models/DomainPredicate';
import { Goal } from '../models/Goal';
import { Plan } from '../models/Plan';


@Injectable()
export class ClipsExecutiveApiService
{
  constructor(private backend: BackendConfigurationService,
              private http: HttpClient) {}

  public list_goals(pretty?: boolean): Observable<Goal[]>
  {
		let params = new HttpParams();
		if (pretty) {
		  params = params.set("pretty", pretty.toString());
		}
    let headers = new HttpHeaders();
		
    headers = headers.set('Accept', 'application/json');
    return this.http.get<Goal[]>(`${this.backend.url_for('api')}/clips-executive/goals`, 
		  { headers: headers, params: params,
		    observe: 'body', responseType: 'json' })	;
	}

  public get_goal(id: string, pretty?: boolean): Observable<Goal>
  {
    if (id === null || id == undefined) {
      throw new Error("Required parameter id is null or undefined (get_goal)");
    }
		let params = new HttpParams();
		if (pretty) {
		  params = params.set("pretty", pretty.toString());
		}
    let headers = new HttpHeaders();
		
    headers = headers.set('Accept', 'application/json');
    return this.http.get<Goal>(`${this.backend.url_for('api')}/clips-executive/goals/${encodeURIComponent(String(id))}`, 
		  { headers: headers, params: params,
		    observe: 'body', responseType: 'json' })	;
	}

  public list_plans(pretty?: boolean): Observable<Plan[]>
  {
		let params = new HttpParams();
		if (pretty) {
		  params = params.set("pretty", pretty.toString());
		}
    let headers = new HttpHeaders();
		
    headers = headers.set('Accept', 'application/json');
    return this.http.get<Plan[]>(`${this.backend.url_for('api')}/clips-executive/plans`, 
		  { headers: headers, params: params,
		    observe: 'body', responseType: 'json' })	;
	}

  public get_plan(goal_id: string, id: string, pretty?: boolean): Observable<Plan>
  {
    if (goal_id === null || goal_id == undefined) {
      throw new Error("Required parameter goal-id is null or undefined (get_plan)");
    }
    if (id === null || id == undefined) {
      throw new Error("Required parameter id is null or undefined (get_plan)");
    }
		let params = new HttpParams();
		if (pretty) {
		  params = params.set("pretty", pretty.toString());
		}
    let headers = new HttpHeaders();
		
    headers = headers.set('Accept', 'application/json');
    return this.http.get<Plan>(`${this.backend.url_for('api')}/clips-executive/plans/${encodeURIComponent(String(goal_id))}/${encodeURIComponent(String(id))}`, 
		  { headers: headers, params: params,
		    observe: 'body', responseType: 'json' })	;
	}

  public list_domain_objects(pretty?: boolean): Observable<DomainObject[]>
  {
		let params = new HttpParams();
		if (pretty) {
		  params = params.set("pretty", pretty.toString());
		}
    let headers = new HttpHeaders();
		
    headers = headers.set('Accept', 'application/json');
    return this.http.get<DomainObject[]>(`${this.backend.url_for('api')}/clips-executive/domain-objects`, 
		  { headers: headers, params: params,
		    observe: 'body', responseType: 'json' })	;
	}

  public list_domain_predicates(pretty?: boolean): Observable<DomainPredicate[]>
  {
		let params = new HttpParams();
		if (pretty) {
		  params = params.set("pretty", pretty.toString());
		}
    let headers = new HttpHeaders();
		
    headers = headers.set('Accept', 'application/json');
    return this.http.get<DomainPredicate[]>(`${this.backend.url_for('api')}/clips-executive/domain-predicates`, 
		  { headers: headers, params: params,
		    observe: 'body', responseType: 'json' })	;
	}

  public list_domain_facts(pretty?: boolean): Observable<DomainFact[]>
  {
		let params = new HttpParams();
		if (pretty) {
		  params = params.set("pretty", pretty.toString());
		}
    let headers = new HttpHeaders();
		
    headers = headers.set('Accept', 'application/json');
    return this.http.get<DomainFact[]>(`${this.backend.url_for('api')}/clips-executive/domain-facts`, 
		  { headers: headers, params: params,
		    observe: 'body', responseType: 'json' })	;
	}

  public list_domain_operators(pretty?: boolean): Observable<DomainOperator[]>
  {
		let params = new HttpParams();
		if (pretty) {
		  params = params.set("pretty", pretty.toString());
		}
    let headers = new HttpHeaders();
		
    headers = headers.set('Accept', 'application/json');
    return this.http.get<DomainOperator[]>(`${this.backend.url_for('api')}/clips-executive/domain-operators`, 
		  { headers: headers, params: params,
		    observe: 'body', responseType: 'json' })	;
	}

}