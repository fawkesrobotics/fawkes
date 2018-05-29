
/****************************************************************************
 *  Clips -- API Service
 *  (auto-generated, do not modify directly)
 *
 *  CLIPS REST API.
 *  Enables access to CLIPS environments.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/

import { Injectable } from '@angular/core';
import { HttpClient, HttpHeaders, HttpParams, HttpResponse } from '@angular/common/http';
import { Observable } from 'rxjs';

import { BackendConfigurationService } from '../../../services/backend-config/backend-config.service';

import { Environment } from '../models/Environment';
import { Fact } from '../models/Fact';


@Injectable()
export class ClipsApiService
{
  constructor(private backend: BackendConfigurationService,
              private http: HttpClient) {}

  public get_facts(env: string, pretty?: boolean, formatted?: boolean): Observable<Fact[]>
  {
    if (env === null || env == undefined) {
      throw new Error("Required parameter env is null or undefined (get_facts)");
    }
		let params = new HttpParams();
		if (pretty) {
		  params = params.set("pretty", pretty.toString());
		}
		if (formatted) {
		  params = params.set("formatted", formatted.toString());
		}
    let headers = new HttpHeaders();
		
    headers = headers.set('Accept', 'application/json');
    return this.http.get<Fact[]>(`${this.backend.url_for('api')}/clips/${encodeURIComponent(String(env))}/facts`, 
		  { headers: headers, params: params,
		    observe: 'body', responseType: 'json' })	;
	}

  public list_environments(pretty?: boolean): Observable<Environment[]>
  {
		let params = new HttpParams();
		if (pretty) {
		  params = params.set("pretty", pretty.toString());
		}
    let headers = new HttpHeaders();
		
    headers = headers.set('Accept', 'application/json');
    return this.http.get<Environment[]>(`${this.backend.url_for('api')}/clips/`, 
		  { headers: headers, params: params,
		    observe: 'body', responseType: 'json' })	;
	}

}