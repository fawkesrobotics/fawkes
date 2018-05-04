
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import { Component, OnInit, OnDestroy, ViewChild } from '@angular/core';
import { MatTableDataSource } from '@angular/material';
import { Observable, interval } from 'rxjs';

import { BackendConfigurationService } from '../../../services/backend-config/backend-config.service';
import { ConfigurationService } from '../../../services/config/config.service';

@Component({
  selector: 'dashboard',
  templateUrl: './dashboard.component.html',
  styleUrls: ['./dashboard.component.scss']
})
export class DashboardComponent implements OnInit, OnDestroy {

  private backend_subscription = null;

  auto_refresh_subscription = null;
  loading = false;
  zero_message = "No facts received.";

  query_cpu =
    'avg without (cpu)(irate(node_cpu{job="node",instance="localhost:9100",mode!="idle"}[5m]))';

  query_threads = 'sum by (threadname)(irate(namedprocess_namegroup_thread_cpu_seconds_total{groupname="fawkes",instance="localhost:9256",job="proc"}[5m]))';

  query_mem = 'namedprocess_namegroup_memory_bytes{memtype="resident"}';

  query_ntp_offset = 'node_ntp_offset_seconds{instance="localhost:9100"}';
  query_mem_avail = 'node_memory_MemAvailable{instance="localhost:9100"}';
  query_ssd_avail = 'node_filesystem_avail{instance="localhost:9100",mountpoint="/"}'
  query_swap_used = '(node_memory_SwapTotal{instance="localhost:9100"}-node_memory_SwapFree{instance="localhost:9100"})/node_memory_SwapTotal{instance="localhost:9100"}';

  charts = [];

  constructor(private backendcfg: BackendConfigurationService,
              private config: ConfigurationService)
  {}

  ngOnInit() {
    this.backend_subscription = this.backendcfg.backend_changed.subscribe((b) => { this.refresh() });
    this.config.get('/webview/dashboard')
      .subscribe(conf => {
        if ('webview' in conf &&
            'dashboard' in conf['webview'] &&
            'charts' in conf['webview']['dashboard'])
        {
          this.charts = Object.values(conf['webview']['dashboard']['charts']);
          this.charts.forEach(c => {
            if (! ('remove_all_zero' in c)) {
              c.remove_all_zero = true;
            }
          });
        }
      });
  }

  ngOnDestroy()
  {
    this.backend_subscription.unsubscribe();
    this.backend_subscription = null;
  }

  refresh()
  {
  }

  private enable_autorefresh()
  {
    if (this.auto_refresh_subscription)  return;
    this.auto_refresh_subscription =
      interval(2000).subscribe((num) => {
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
