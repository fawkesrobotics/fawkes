
/****************************************************************************
 *  Image -- API Service
 *  (auto-generated, do not modify directly)
 *
 *  Fawkes Image REST API.
 *  Access images through a REST API.
 *
 *  API Contact: Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
 *  API Version: v1beta1
 *  API License: Apache 2.0
 ****************************************************************************/

import { Injectable } from '@angular/core';
import { HttpClient, HttpHeaders, HttpParams, HttpResponse } from '@angular/common/http';
import { Observable } from 'rxjs';

import { BackendConfigurationService } from '../../../services/backend-config/backend-config.service';

import { ImageInfo } from '../models/ImageInfo';


@Injectable()
export class ImageApiService
{
  constructor(private backend: BackendConfigurationService,
              private http: HttpClient) {}

  public list_images(pretty?: boolean): Observable<ImageInfo[]>
  {
		let params = new HttpParams();
		if (pretty) {
		  params = params.set("pretty", pretty.toString());
		}
    let headers = new HttpHeaders();
		
    headers = headers.set('Accept', 'application/json');
    return this.http.get<ImageInfo[]>(`${this.backend.url_for('api')}/images`, 
		  { headers: headers, params: params,
		    observe: 'body', responseType: 'json' })	;
	}

  public get_image(id: string, pretty?: boolean): Observable<HttpResponse<string>>
  {
    if (id === null || id == undefined) {
      throw new Error("Required parameter id is null or undefined (get_image)");
    }
		let params = new HttpParams();
		if (pretty) {
		  params = params.set("pretty", pretty.toString());
		}
    let headers = new HttpHeaders();
		
    return this.http.get(`${this.backend.url_for('api')}/images/${encodeURIComponent(String(id))}`, 
		  { headers: headers, params: params,
		    observe: 'response', responseType: 'text' })	;
	}

}