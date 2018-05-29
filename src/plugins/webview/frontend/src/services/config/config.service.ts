
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import { Injectable, EventEmitter } from '@angular/core';
import { HttpClient } from '@angular/common/http';

import { Observable, from } from 'rxjs';
import { tap, map } from 'rxjs/operators';

import { BackendConfigurationService } from '../backend-config/backend-config.service';
import { ConfigTree } from './model/ConfigTree';
import { ConfigurationApiService } from './api.service';

const CACHE_INTERVAL_MSEC = 300000;

@Injectable()
export class ConfigurationService {

  public changed: EventEmitter<string>;

  private cache = {};
  
  constructor(private backendcfg: BackendConfigurationService,
              private config_api: ConfigurationApiService)
  {
    this.changed = new EventEmitter();
  }

  get(query: string = "")
  {
    if (query in this.cache &&
        (Date.now() - this.cache[query].retrieval_time) <= CACHE_INTERVAL_MSEC)
    {
      return from([this.cache[query].data]);
    } else {
      return this.config_api.get_config(query)
        .pipe(
          tap(conf => {
            this.cache[query] = {
              retrieval_time: Date.now(),
              data: conf.config
            }}),
          map(conf => conf.config)
        );
    }
  }
}
