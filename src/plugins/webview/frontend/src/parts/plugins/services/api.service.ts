
/****************************************************************************
 *  Plugin -- API Service
 *  (auto-generated, do not modify directly)
 *
 *  Fawkes Plugin REST API.
 *  List, load, and unload plugins.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/

import { Injectable } from '@angular/core';
import { HttpClient, HttpHeaders, HttpParams, HttpResponse } from '@angular/common/http';
import { Observable } from 'rxjs';

import { BackendConfigurationService } from '../../../services/backend-config/backend-config.service';

import { Plugin } from '../models/Plugin';
import { PluginOpRequest } from '../models/PluginOpRequest';
import { PluginOpResponse } from '../models/PluginOpResponse';


@Injectable()
export class PluginApiService
{
  constructor(private backend: BackendConfigurationService,
              private http: HttpClient) {}

  public list_plugins(pretty?: boolean): Observable<Plugin[]>
  {
		let params = new HttpParams();
		if (pretty) {
		  params = params.set("pretty", pretty.toString());
		}
    let headers = new HttpHeaders();
		
    headers = headers.set('Accept', 'application/json');
    return this.http.get<Plugin[]>(`${this.backend.url_for('api')}/plugins`, 
		  { headers: headers, params: params,
		    observe: 'body', responseType: 'json' })	;
	}

  public get_plugin(name: string, pretty?: boolean): Observable<Plugin>
  {
    if (name === null || name == undefined) {
      throw new Error("Required parameter name is null or undefined (get_plugin)");
    }
		let params = new HttpParams();
		if (pretty) {
		  params = params.set("pretty", pretty.toString());
		}
    let headers = new HttpHeaders();
		
    headers = headers.set('Accept', 'application/json');
    return this.http.get<Plugin>(`${this.backend.url_for('api')}/plugins/${encodeURIComponent(String(name))}`, 
		  { headers: headers, params: params,
		    observe: 'body', responseType: 'json' })	;
	}

  public set_plugin_state(name: string, operation: PluginOpRequest, pretty?: boolean): Observable<PluginOpResponse>
  {
    if (name === null || name == undefined) {
      throw new Error("Required parameter name is null or undefined (set_plugin_state)");
    }
    if (operation === null || operation == undefined) {
      throw new Error("Required parameter operation is null or undefined (set_plugin_state)");
    }
		let params = new HttpParams();
		if (pretty) {
		  params = params.set("pretty", pretty.toString());
		}
    let headers = new HttpHeaders();
		
    headers = headers.set('Accept', 'application/json');
    return this.http.put<PluginOpResponse>(`${this.backend.url_for('api')}/plugins/${encodeURIComponent(String(name))}`, operation, 
		  { headers: headers, params: params,
		    observe: 'body', responseType: 'json' })	;
	}

}