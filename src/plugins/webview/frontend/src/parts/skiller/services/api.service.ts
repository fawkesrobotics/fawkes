
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
import { HttpClient, HttpHeaders } from '@angular/common/http';
import { Observable } from 'rxjs/Observable';
import 'rxjs/add/observable/of';
import 'rxjs/add/operator/map';

import { ConfigurationService } from '../../../services/config.service';

import { Skill } from '../models/Skill';
import { SkillInfo } from '../models/SkillInfo';


@Injectable()
export class BehaviorEngineApiService
{
  constructor(private config: ConfigurationService,
              private http: HttpClient) {}

  public list_skills(pretty?: boolean): Observable<SkillInfo[]>
  {
    let headers = new HttpHeaders();
    headers = headers.set('Accept', 'application/json');
    let options = { headers: headers }
    return this.http.get<SkillInfo[]>(`${this.config.get('apiurl')}/skiller/skills`, options);
	}

  public get_skill(id: string, pretty?: boolean): Observable<Skill>
  {
    if (id === null || id == undefined) {
      throw new Error("Required parameter id is null or undefined (get_skill)");
    }
    let headers = new HttpHeaders();
    headers = headers.set('Accept', 'application/json');
    let options = { headers: headers }
    return this.http.get<Skill>(`${this.config.get('apiurl')}/skiller/skills/${encodeURIComponent(String(id))}`, options);
	}

}