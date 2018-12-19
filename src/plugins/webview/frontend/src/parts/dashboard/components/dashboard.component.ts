
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import { Component, OnInit, OnDestroy } from '@angular/core';
import { interval } from 'rxjs';

import { BackendConfigurationService } from '../../../services/backend-config/backend-config.service';
import { ConfigurationService } from '../../../services/config/config.service';

@Component({
  selector: 'ff-dashboard',
  templateUrl: './dashboard.component.html',
  styleUrls: ['./dashboard.component.scss']
})
export class DashboardComponent implements OnInit, OnDestroy {

  private backend_subscription = null;

  auto_refresh_subscription = null;
  loading = false;
  zero_message = 'No facts received.';

  charts = [];
  stats  = [];

  constructor(private backendcfg: BackendConfigurationService,
              private config: ConfigurationService) {}

  ngOnInit() {
    this.backend_subscription = this.backendcfg.backend_changed.subscribe((b) => { this.refresh(); });
    this.config.get('/webview/dashboard')
      .subscribe(conf => {
        if ('webview' in conf && 'dashboard' in conf['webview']) {

          if ('charts' in conf['webview']['dashboard']) {
            this.charts = Object.values(conf['webview']['dashboard']['charts']);
            this.charts.forEach(c => {
              if (! ('remove_all_zero' in c)) {
                c.remove_all_zero = true;
              }
            });
          }
          if ('stats' in conf['webview']['dashboard']) {
            this.stats = Object.values(conf['webview']['dashboard']['stats']);
            this.stats.forEach(s => {
              if ('divisor' in s) {
                s.factor = 1. / s.divisor;
              }
              console.log("Stat " + s.name);
              console.log("Query " + s.query);
              console.log("Factor " + s.factor);
              console.log("thresholds " + s.thresholds);
              console.log("thresholds type " + typeof s.thresholds);
              console.log("thresholds test " + Array.from(s.thresholds));
              console.log("Unit " + s.unit);
            });
          }
        }
      });
  }

  ngOnDestroy() {
    this.backend_subscription.unsubscribe();
    this.backend_subscription = null;
  }

  refresh() {
  }

  private enable_autorefresh() {
    if (this.auto_refresh_subscription) {  return; }
    this.auto_refresh_subscription =
      interval(2000).subscribe((num) => {
        this.refresh();
      });
    this.refresh();
  }

  private disable_autorefresh() {
    if (this.auto_refresh_subscription) {
      this.auto_refresh_subscription.unsubscribe();
      this.auto_refresh_subscription = null;
    }
  }

  toggle_autorefresh() {
    if (this.auto_refresh_subscription) {
      this.disable_autorefresh();
    } else {
      this.enable_autorefresh();
    }
  }
}
