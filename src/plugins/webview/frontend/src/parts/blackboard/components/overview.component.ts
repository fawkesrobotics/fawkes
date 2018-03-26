
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import { Component, OnInit } from '@angular/core';
import { Observable } from 'rxjs/Observable';
import 'rxjs/add/observable/interval';

import { BlackboardApiService } from '../services/api.service';
import { InterfaceInfo } from '../models/InterfaceInfo';

@Component({
  selector: 'blackboard-overview',
  templateUrl: './overview.component.html',
  styleUrls: ['./overview.component.scss']
})
export class BlackboardOverviewComponent implements OnInit {

  loading = false;
  auto_refresh_subscription = null;
  interfaces = null;
  zero_message = "No graph has been retrieved";
  
  constructor(private api_service: BlackboardApiService)
  {}

  ngOnInit() {
    this.refresh();
  }

  keys(obj) {
    return Object.keys(obj);
  }
  
  refresh()
  {
    this.loading = true;
    this.zero_message = "Retrieving interface info";

    this.api_service.list_interfaces().subscribe(
      (interfaces) => {
        let ifs = {}
        for (let i of interfaces) {
          if (ifs[i.type]) {
            ifs[i.type].push(i);
          } else {
            ifs[i.type] = [i];
          }
        }
        this.interfaces = ifs;
        this.loading = false;
      },
      (err) => {
        this.interfaces = null;
        if (err.status == 0) {
          this.zero_message="API server unavailable. Robot down?";
        } else {
          this.zero_message=`Failed to retrieve interfaces: ${err.error}`;
        }
        this.loading = false;
      }
    );
  }

  private enable_autorefresh()
  {
    if (this.auto_refresh_subscription)  return;
    this.auto_refresh_subscription =
      Observable.interval(2000).subscribe((num) => {
        this.refresh();
      });
    this.refresh();
  }

  private disable_autorefresh()
  {
    if (this.auto_refresh_subscription) {
      this.auto_refresh_subscription.unsubscribe();
      this.auto_refresh_subscription = null;
    }
  }

  toggle_autorefresh()
  {
    if (this.auto_refresh_subscription) {
      this.disable_autorefresh();
    } else {
      this.enable_autorefresh();
    }
  }

}
