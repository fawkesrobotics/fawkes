
/****************************************************************************
 *  Blackboard -- API Service
 *  (auto-generated, do not modify directly)
 *
 *  Fawkes Blackboard REST API.
 *  Access blackboard data through a REST API.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/

import { Injectable } from '@angular/core';
import { HttpClient, HttpHeaders, HttpParams, HttpResponse } from '@angular/common/http';
import { Observable } from 'rxjs';

import { BackendConfigurationService } from '../../../services/backend-config/backend-config.service';

import { BlackboardGraph } from '../models/BlackboardGraph';
import { InterfaceData } from '../models/InterfaceData';
import { InterfaceInfo } from '../models/InterfaceInfo';


@Injectable()
export class BlackboardApiService
{
  constructor(private backend: BackendConfigurationService,
              private http: HttpClient) {}

  public list_interfaces(pretty?: boolean): Observable<InterfaceInfo[]>
  {
		let params = new HttpParams();
		if (pretty) {
		  params = params.set("pretty", pretty.toString());
		}
    let headers = new HttpHeaders();
		
    headers = headers.set('Accept', 'application/json');
    return this.http.get<InterfaceInfo[]>(`${this.backend.url_for('api')}/blackboard/interfaces`, 
		  { headers: headers, params: params,
		    observe: 'body', responseType: 'json' })	;
	}

  public get_interface_info(type: string, id: string, pretty?: boolean): Observable<InterfaceInfo>
  {
    if (type === null || type == undefined) {
      throw new Error("Required parameter type is null or undefined (get_interface_info)");
    }
    if (id === null || id == undefined) {
      throw new Error("Required parameter id is null or undefined (get_interface_info)");
    }
		let params = new HttpParams();
		if (pretty) {
		  params = params.set("pretty", pretty.toString());
		}
    let headers = new HttpHeaders();
		
    headers = headers.set('Accept', 'application/json');
    return this.http.get<InterfaceInfo>(`${this.backend.url_for('api')}/blackboard/interfaces/${encodeURIComponent(String(type))}/${encodeURIComponent(String(id))}`, 
		  { headers: headers, params: params,
		    observe: 'body', responseType: 'json' })	;
	}

  public get_interface_data(type: string, id: string, pretty?: boolean): Observable<InterfaceData>
  {
    if (type === null || type == undefined) {
      throw new Error("Required parameter type is null or undefined (get_interface_data)");
    }
    if (id === null || id == undefined) {
      throw new Error("Required parameter id is null or undefined (get_interface_data)");
    }
		let params = new HttpParams();
		if (pretty) {
		  params = params.set("pretty", pretty.toString());
		}
    let headers = new HttpHeaders();
		
    headers = headers.set('Accept', 'application/json');
    return this.http.get<InterfaceData>(`${this.backend.url_for('api')}/blackboard/interfaces/${encodeURIComponent(String(type))}/${encodeURIComponent(String(id))}/data`, 
		  { headers: headers, params: params,
		    observe: 'body', responseType: 'json' })	;
	}

  public get_graph(pretty?: boolean): Observable<BlackboardGraph>
  {
		let params = new HttpParams();
		if (pretty) {
		  params = params.set("pretty", pretty.toString());
		}
    let headers = new HttpHeaders();
		
    headers = headers.set('Accept', 'application/json');
    return this.http.get<BlackboardGraph>(`${this.backend.url_for('api')}/blackboard/graph`, 
		  { headers: headers, params: params,
		    observe: 'body', responseType: 'json' })	;
	}

}