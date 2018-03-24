
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
import { HttpClient, HttpHeaders } from '@angular/common/http';
import { Observable } from 'rxjs/Observable';
import 'rxjs/add/observable/of';
import 'rxjs/add/operator/map';

import { ConfigurationService } from '../../../services/config.service';

import { DomainFact } from '../models/DomainFact';
import { DomainObject } from '../models/DomainObject';
import { DomainOperator } from '../models/DomainOperator';
import { DomainPredicate } from '../models/DomainPredicate';
import { Goal } from '../models/Goal';
import { Plan } from '../models/Plan';


@Injectable()
export class ClipsExecutiveApiService
{
  constructor(private config: ConfigurationService,
              private http: HttpClient) {}

  public list_goals(pretty?: boolean): Observable<Goal[]>
  {
    let headers = new HttpHeaders();
    headers = headers.set('Accept', 'application/json');
    let options = { headers: headers }
    return this.http.get<Goal[]>(`${this.config.get('apiurl')}/clips-executive/goals`, options);
	}

  public add_goal(): Observable<any>
  {
    let headers = new HttpHeaders();
    headers = headers.set('Content-type', 'application/json');
    let options = { headers: headers }
    return this.http.post<any>(`${this.config.get('apiurl')}/clips-executive/goals`, options);
	}

  public get_goal(id: string, pretty?: boolean): Observable<Goal>
  {
    if (id === null || id == undefined) {
      throw new Error("Required parameter id is null or undefined (get_goal)");
    }
    let headers = new HttpHeaders();
    headers = headers.set('Accept', 'application/json');
    let options = { headers: headers }
    return this.http.get<Goal>(`${this.config.get('apiurl')}/clips-executive/goals/${encodeURIComponent(String(id))}`, options);
	}

  public list_plans(pretty?: boolean): Observable<Plan[]>
  {
    let headers = new HttpHeaders();
    headers = headers.set('Accept', 'application/json');
    let options = { headers: headers }
    return this.http.get<Plan[]>(`${this.config.get('apiurl')}/clips-executive/plans`, options);
	}

  public get_plan(goal_id: string, id: string, pretty?: boolean): Observable<Plan>
  {
    if (goal_id === null || goal_id == undefined) {
      throw new Error("Required parameter goal-id is null or undefined (get_plan)");
    }
    if (id === null || id == undefined) {
      throw new Error("Required parameter id is null or undefined (get_plan)");
    }
    let headers = new HttpHeaders();
    headers = headers.set('Accept', 'application/json');
    let options = { headers: headers }
    return this.http.get<Plan>(`${this.config.get('apiurl')}/clips-executive/plans/${encodeURIComponent(String(goal_id))}/${encodeURIComponent(String(id))}`, options);
	}

  public list_domain_objects(pretty?: boolean): Observable<DomainObject[]>
  {
    let headers = new HttpHeaders();
    headers = headers.set('Accept', 'application/json');
    let options = { headers: headers }
    return this.http.get<DomainObject[]>(`${this.config.get('apiurl')}/clips-executive/domain-objects`, options);
	}

  public list_domain_predicates(pretty?: boolean): Observable<DomainPredicate[]>
  {
    let headers = new HttpHeaders();
    headers = headers.set('Accept', 'application/json');
    let options = { headers: headers }
    return this.http.get<DomainPredicate[]>(`${this.config.get('apiurl')}/clips-executive/domain-predicates`, options);
	}

  public list_domain_facts(pretty?: boolean): Observable<DomainFact[]>
  {
    let headers = new HttpHeaders();
    headers = headers.set('Accept', 'application/json');
    let options = { headers: headers }
    return this.http.get<DomainFact[]>(`${this.config.get('apiurl')}/clips-executive/domain-facts`, options);
	}

  public list_domain_operators(pretty?: boolean): Observable<DomainOperator[]>
  {
    let headers = new HttpHeaders();
    headers = headers.set('Accept', 'application/json');
    let options = { headers: headers }
    return this.http.get<DomainOperator[]>(`${this.config.get('apiurl')}/clips-executive/domain-operators`, options);
	}

}