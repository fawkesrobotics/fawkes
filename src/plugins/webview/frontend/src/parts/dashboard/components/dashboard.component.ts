
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import { Component, OnInit, ViewChild } from '@angular/core';
import { Router, ActivatedRoute, ParamMap } from '@angular/router';
import { MatTableDataSource } from '@angular/material';

import { Observable } from 'rxjs/Observable';
import 'rxjs/add/observable/interval';

//import { DashboardApiService } from '../services/api.service';

@Component({
  selector: 'dashboard',
  templateUrl: './dashboard.component.html',
  styleUrls: ['./dashboard.component.scss']
})
export class DashboardComponent implements OnInit {

  auto_refresh_subscription = null;
  loading = false;
  zero_message = "No facts received.";

  query_cpu =
    'avg without (cpu)(irate(node_cpu{job="node",instance="localhost:9100",mode!="idle"}[5m]))';

  query_threads = 'sum by (threadname)(irate(namedprocess_namegroup_thread_cpu_seconds_total{groupname="fawkes",instance="localhost:9256",job="proc"}[5m]))';

  query_mem = 'namedprocess_namegroup_memory_bytes{memtype="resident"}';

  constructor(//private readonly api_service: DashboardApiService,
              private router: Router)
  {
  }

  ngOnInit() {
  }

  refresh()
  {
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
