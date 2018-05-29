
/****************************************************************************
 *  Transforms -- API Service
 *  (auto-generated, do not modify directly)
 *
 *  Fawkes Transforms REST API.
 *  Transforms information and some calculations.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/

import { Injectable } from '@angular/core';
import { HttpClient, HttpHeaders, HttpParams, HttpResponse } from '@angular/common/http';
import { Observable } from 'rxjs';

import { BackendConfigurationService } from '../../../services/backend-config/backend-config.service';

import { TransformsGraph } from '../models/TransformsGraph';


@Injectable()
export class TransformsApiService
{
  constructor(private backend: BackendConfigurationService,
              private http: HttpClient) {}

  public get_graph(pretty?: boolean): Observable<TransformsGraph>
  {
		let params = new HttpParams();
		if (pretty) {
		  params = params.set("pretty", pretty.toString());
		}
    let headers = new HttpHeaders();
		
    headers = headers.set('Accept', 'application/json');
    return this.http.get<TransformsGraph>(`${this.backend.url_for('api')}/transforms/graph`, 
		  { headers: headers, params: params,
		    observe: 'body', responseType: 'json' })	;
	}

}