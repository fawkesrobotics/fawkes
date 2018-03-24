
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import { Injectable } from '@angular/core';
import { Router } from '@angular/router';


@Injectable()
export class ConfigurationService {

  public config = {}

  constructor()
  {
    let url = new URL(window.location.href); 
    this.config['url'] = {
      host     : url.host,
      hostname : url.hostname,
      port     : url.port,
      protocol : url.protocol,
      base     : url.origin
    };
    this.config['urlbase'] = this.config['url'].base;
    this.config['apiurl'] = this.config['url'].base + "/api";

    this.config['apiurl'] = 'http://localhost:8088/api';
  }

  get(key : string) {
    const keys: string[] = key.split('.');
    let result: any = this.config[keys.shift()!];
  
    for (const key of keys) {
      if (result === null || typeof(result) !== 'object') {
        return undefined;
      }
    
      result = result[key];
    }
    return result;
  }

  set(key: string, value: any)
  {
    const keys: string[] = key.split('.');
    if (keys.length == 0) {
      throw new Error("Key may not be the empty string");
    }

    let parent = this.config;
    while (keys.length > 1) {
      parent = this.config[keys[0]];
      keys.shift();
    }
    parent[keys[0]] = value;
  }
}
