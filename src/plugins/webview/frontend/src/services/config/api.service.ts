
/****************************************************************************
 *  Configuration -- API Service
 *  (auto-generated, do not modify directly)
 *
 *  Fawkes Configuration REST API.
 *  Retrieve information from the configuration.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/

import { Injectable } from '@angular/core';
import { HttpClient, HttpHeaders, HttpParams, HttpResponse } from '@angular/common/http';
import { Observable } from 'rxjs';

import { BackendConfigurationService } from '../backend-config/backend-config.service';

import { ConfigTree } from './model/ConfigTree';


@Injectable()
export class ConfigurationApiService
{
  constructor(private backend: BackendConfigurationService,
              private http: HttpClient) {}

  public get_config(query?: string, pretty?: boolean): Observable<ConfigTree>
  {
		let params = new HttpParams();
		if (query) {
		  params = params.set("query", query.toString());
		}
		if (pretty) {
		  params = params.set("pretty", pretty.toString());
		}
    let headers = new HttpHeaders();
		
    headers = headers.set('Accept', 'application/json');
    return this.http.get<ConfigTree>(`${this.backend.url_for('api')}/config`, 
		  { headers: headers, params: params,
		    observe: 'body', responseType: 'json' })	;
	}

}
