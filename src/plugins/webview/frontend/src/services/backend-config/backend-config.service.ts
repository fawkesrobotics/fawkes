
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import { Injectable, EventEmitter } from '@angular/core';
import { HttpClient } from '@angular/common/http';

import { Observable, interval } from 'rxjs';
import { delay, retryWhen } from 'rxjs/operators';

import { environment } from '../../environments/environment';

import { Backend } from './model/Backend';
import { Service } from './model/Service';

@Injectable()
export class BackendConfigurationService {

  public backend_changed: EventEmitter<string>;
  
  private backends_ = {}
  private backends_list_ = null;
  private auto_refresh_subscription_ = null;

  public current_backend:string = 'origin';

  constructor(private http: HttpClient)
  {
    this.backend_changed = new EventEmitter();

    let local_url = new URL(window.location.href);
    let api_url   = new URL(`${local_url.origin}/api`);
    let prom_url  = null;

    if (! environment.production) {
      api_url   = new URL(`http://${local_url.hostname}:8088/api`);
      prom_url  = new URL(`${api_url.protocol}//${api_url.hostname}:9090`);
    }

    this.backends_['origin'] = {
      show:        true,
      name:        'Local',
      url:         local_url,
      services: {
        api:         api_url,
        prometheus:  prom_url
      }
    };

    this.refresh();
  }

  set_backend(backend: string)
  {
    if (this.backends_[backend]) {
      this.current_backend = backend;
      this.backend_changed.emit(backend);
    }
  }

  get backend()
  {
    return this.current_backend;
  }

  get backend_name()
  {
    return this.backends_[this.current_backend].name;
  }

  get backends()
  {
    return Object.assign({}, this.backends_);
  }

  get backend_list()
  {
    if (! this.backends_list_) {
      this.backends_list_ = []
      for (let b in this.backends_) {
        if (this.backends_[b].show) {
          this.backends_list_.push(Object.assign({id: b}, this.backends_[b]));
        }
      }
      this.backends_list_.sort((a,b) => {
        if (a.id == 'origin' && b.id == 'origin') {
          return 0;
        } else if (a.id == 'origin') {
          return 1;
        } else if (b.id == 'origin') {
          return -1;
        } else {
          let a_s = a.name.toUpperCase();
          let b_s = b.name.toUpperCase();
          if (a_s < b_s) {
            return -1;
          } else if (a_s > b_s) {
            return 1;
          } else {
            return 0;
          }
        }
      });
    }
    return this.backends_list_;
  }

  services(): string[]
  {
    return Object.keys(this.backends_[this.current_backend]);
  }

  has_url_for(service: string): boolean {
    return (this.backends_[this.current_backend].services[service] != null);
  }

  url_for(service : string) {
    if (this.backends_[this.current_backend].services[service]) {
      let url = this.backends_[this.current_backend].services[service].toString();
      return url.replace(/\/$/, "");
    } else {
      console.log(`No service ${service} known for backend ${this.current_backend}`);
      return null;
    }
  }

  url_expand(url: string)
  {
    return new URL(url
                   .replace('$ORIGIN', this.backends_['origin'].url.origin)
                   .replace('$HOSTNAME', this.backends_['origin'].url.hostname)
                   .replace('$HOST', this.backends_['origin'].url.host)
                   .replace('$SCHEME', this.backends_['origin'].url.protocol)
                   .replace('$PORT', this.backends_['origin'].url.port));
  }
  
  refresh()
  {
    this.http.get<Backend[]>(this.backends_['origin'].services['api'] + '/backends')
      .pipe(
        retryWhen(errors => errors.pipe(delay(5000)))
      )
      .subscribe(
        (recv_backends) => {
          let new_backends = {};
          for (let backend of recv_backends) {
            // Ignore if we got some "origin" from the remote
            if (backend.id == 'origin')  continue;

            let services = {};
            for (let service of backend.services) {
              services[service.name] = this.url_expand(service.url);
            }
            new_backends[backend.id] = {
              show: true,
              name: backend.name,
              url: this.url_expand(backend.url),
              services: services
            }
          }
          // Check if origin backend should be show, i.e., it is not
          // included in the list of received backends.
          // check if any of the received environments maches origin
          let have_origin = false;
          let new_origin = "origin";
          let origin_backend = this.backends_['origin'];
          for (let bn in new_backends) {
            let backend = new_backends[bn];
            if (backend.url.toString() == origin_backend.url.toString() &&
                backend.services['api'].toString() === origin_backend.services['api'].toString())
            {
              have_origin = true;
              new_origin = bn;
              break;
            }
          }

          // Origin is immutable, it may only be shown or not
          this.backends_['origin'].show = ! have_origin;
          new_backends['origin'] = this.backends_['origin'];

          this.backends_ = new_backends;
          this.backends_list_ = null;

          if (have_origin && this.current_backend == 'origin') {
            this.set_backend(new_origin);
          }
          let new_idx = Object.keys(this.backends_).findIndex((b) => {
            return (b == new_origin && this.backends_[b].show);
          });
          if (new_idx < 0) {
            this.set_backend(new_origin);
          }

          this.auto_refresh_subscription_ =
            interval(300000).subscribe((num) => {
              this.auto_refresh_subscription_.unsubscribe();
              this.auto_refresh_subscription_ = null;
              this.refresh();
            });
        },
        (err) => {
          if (err.status == 0) {
            console.error("Failed to retrieve list of available backends, API not reachable");
          } else {
            console.error("Failed to retrieve list of available backends: ${err.error}");
          }
        });
  }
}
