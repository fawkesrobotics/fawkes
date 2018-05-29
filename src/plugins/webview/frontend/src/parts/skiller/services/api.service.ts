
/****************************************************************************
 *  BehaviorEngine -- API Service
 *  (auto-generated, do not modify directly)
 *
 *  Behavior Engine REST API.
 *  Visualize, monitor, and instruct the Skill Execution Run-Time of
 *  the Lua-based Behavior Engine.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/

import { Injectable } from '@angular/core';
import { HttpClient, HttpHeaders, HttpParams, HttpResponse } from '@angular/common/http';
import { Observable } from 'rxjs';

import { BackendConfigurationService } from '../../../services/backend-config/backend-config.service';

import { Skill } from '../models/Skill';
import { SkillCall } from '../models/SkillCall';
import { SkillInfo } from '../models/SkillInfo';


@Injectable()
export class BehaviorEngineApiService
{
  constructor(private backend: BackendConfigurationService,
              private http: HttpClient) {}

  public list_skills(pretty?: boolean): Observable<SkillInfo[]>
  {
		let params = new HttpParams();
		if (pretty) {
		  params = params.set("pretty", pretty.toString());
		}
    let headers = new HttpHeaders();
		
    headers = headers.set('Accept', 'application/json');
    return this.http.get<SkillInfo[]>(`${this.backend.url_for('api')}/skiller/skills`, 
		  { headers: headers, params: params,
		    observe: 'body', responseType: 'json' })	;
	}

  public get_skill(id: string, pretty?: boolean): Observable<Skill>
  {
    if (id === null || id == undefined) {
      throw new Error("Required parameter id is null or undefined (get_skill)");
    }
		let params = new HttpParams();
		if (pretty) {
		  params = params.set("pretty", pretty.toString());
		}
    let headers = new HttpHeaders();
		
    headers = headers.set('Accept', 'application/json');
    return this.http.get<Skill>(`${this.backend.url_for('api')}/skiller/skills/${encodeURIComponent(String(id))}`, 
		  { headers: headers, params: params,
		    observe: 'body', responseType: 'json' })	;
	}

  public stop_skill(id: string): Observable<HttpResponse<string>>
  {
    if (id === null || id == undefined) {
      throw new Error("Required parameter id is null or undefined (stop_skill)");
    }
		let params = new HttpParams();
    let headers = new HttpHeaders();
		
    return this.http.delete(`${this.backend.url_for('api')}/skiller/skills/${encodeURIComponent(String(id))}`, 
		  { headers: headers, params: params,
		    observe: 'response', responseType: 'text' })	;
	}

  public exec_skill(skill_call: SkillCall): Observable<Skill>
  {
    if (skill_call === null || skill_call == undefined) {
      throw new Error("Required parameter skill_call is null or undefined (exec_skill)");
    }
		let params = new HttpParams();
    let headers = new HttpHeaders();
		
    headers = headers.set('Accept', 'application/json');
    return this.http.post<Skill>(`${this.backend.url_for('api')}/skiller/call`, skill_call, 
		  { headers: headers, params: params,
		    observe: 'body', responseType: 'json' })	;
	}

}