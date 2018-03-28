
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
import { HttpClient, HttpHeaders } from '@angular/common/http';
import { Observable } from 'rxjs/Observable';
import 'rxjs/add/observable/of';
import 'rxjs/add/operator/map';

import { ConfigurationService } from '../../../services/config.service';

import { InterfaceData } from '../models/InterfaceData';
import { InterfaceInfo } from '../models/InterfaceInfo';


@Injectable()
export class BlackboardApiService
{
  constructor(private config: ConfigurationService,
              private http: HttpClient) {}

  public list_interfaces(pretty?: boolean): Observable<InterfaceInfo[]>
  {
    let headers = new HttpHeaders();
    headers = headers.set('Accept', 'application/json');
    let options = { headers: headers }
    return this.http.get<InterfaceInfo[]>(`${this.config.get('apiurl')}/blackboard/interfaces`, options);
	}

  public get_interface_info(type: string, id: string, pretty?: boolean): Observable<InterfaceInfo>
  {
    if (type === null || type == undefined) {
      throw new Error("Required parameter type is null or undefined (get_interface_info)");
    }
    if (id === null || id == undefined) {
      throw new Error("Required parameter id is null or undefined (get_interface_info)");
    }
    let headers = new HttpHeaders();
    headers = headers.set('Accept', 'application/json');
    let options = { headers: headers }
    return this.http.get<InterfaceInfo>(`${this.config.get('apiurl')}/blackboard/interfaces/${encodeURIComponent(String(type))}/${encodeURIComponent(String(id))}`, options);
	}

  public get_interface_data(type: string, id: string, pretty?: boolean): Observable<InterfaceData>
  {
    if (type === null || type == undefined) {
      throw new Error("Required parameter type is null or undefined (get_interface_data)");
    }
    if (id === null || id == undefined) {
      throw new Error("Required parameter id is null or undefined (get_interface_data)");
    }
    let headers = new HttpHeaders();
    headers = headers.set('Accept', 'application/json');
    let options = { headers: headers }
    return this.http.get<InterfaceData>(`${this.config.get('apiurl')}/blackboard/interfaces/${encodeURIComponent(String(type))}/${encodeURIComponent(String(id))}/data`, options);
	}

}