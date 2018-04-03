
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
import { HttpClient, HttpHeaders, HttpParams } from '@angular/common/http';
import { Observable } from 'rxjs/Observable';
import 'rxjs/add/observable/of';
import 'rxjs/add/operator/map';

import { ConfigurationService } from '../../../services/config.service';

import { Environment } from '../models/Environment';
import { Fact } from '../models/Fact';


@Injectable()
export class ClipsApiService
{
  constructor(private config: ConfigurationService,
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
    let options = { headers: headers, params: params };
    return this.http.get<Fact[]>(`${this.config.get('apiurl')}/clips/${encodeURIComponent(String(env))}/facts`, options);
	}

  public list_environments(pretty?: boolean): Observable<Environment[]>
  {
		let params = new HttpParams();
		if (pretty) {
		  params = params.set("pretty", pretty.toString());
		}
    let headers = new HttpHeaders();
    headers = headers.set('Accept', 'application/json');
    let options = { headers: headers, params: params };
    return this.http.get<Environment[]>(`${this.config.get('apiurl')}/clips/`, options);
	}

}