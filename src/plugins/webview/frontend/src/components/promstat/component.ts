
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import {Component, Input, AfterViewInit, ViewChild,
        OnInit, OnDestroy, HostListener} from '@angular/core';
import { HttpClient } from '@angular/common/http';

import { Observable, interval } from 'rxjs';

import { BackendConfigurationService } from '../../services/backend-config/backend-config.service';

@Component({
  selector: 'prom-stat',
  templateUrl: './template.html',
  styleUrls: ['./style.scss']
})
export class PrometheusStatComponent implements AfterViewInit, OnInit, OnDestroy {
  @Input() query?: string = '';
  @Input() unit?: string = '';
  @Input() factor?: number = 1;
  @Input() decimals?: number = 2;
  @Input() label?: string = "?";
  @Input() refresh_interval_sec?: number = 10;
  @Input() thresholds: number[] = [];

  have_data: boolean = false;
  zero_message: string = "No data received";
  auto_refresh_subscription = null;

  value: string = "";
  css_class: string = "";
  
  private backend_subscription = null;

  constructor(private backendcfg: BackendConfigurationService,
              private http: HttpClient)
  {}

  ngAfterViewInit()
  {
    if (!this.query) {
      this.zero_message='No query';
    } else {
      this.enable_autorefresh();
    }
  }

  ngOnInit() {
    this.backend_subscription = this.backendcfg.backend_changed.subscribe((backend) => {
      this.refresh();
    });
  }

  ngOnDestroy()
  {
    this.disable_autorefresh();
    this.backend_subscription.unsubscribe();
    this.backend_subscription = null;
  }

  @HostListener('window:focus', ['$event'])
  onFocus(ev: FocusEvent)
  {
    this.enable_autorefresh();
  }

  @HostListener('window:blur', ['$event'])
  onBlur(ev: FocusEvent)
  {
    this.disable_autorefresh();
  }

  refresh()
  {
    if (! this.backendcfg.has_url_for('prometheus')) {
      this.zero_message = 'No Prometheus backend service known';
      return;
    }

    let url =
      `${this.backendcfg.url_for('prometheus')}/api/v1/query?query=${encodeURIComponent(String(this.query))}`;
    this.http.get<any>(url)
      .subscribe(
        (obj) => {
          if (obj.status == 'success' && obj.data.result.length > 0) {
            let v = parseFloat(obj.data.result[0].value[1]) * this.factor;
            this.value = `${v.toFixed(this.decimals)} ${this.unit}`;
            if (this.thresholds.length > 0) {
              if (this.thresholds.length == 4) {
                if (v < this.thresholds[0]) {
                  this.css_class="promstat-red";
                } else if (v >= this.thresholds[0] && v < this.thresholds[1]) {
                  this.css_class="promstat-orange";
                } else if (v >= this.thresholds[1] && v <= this.thresholds[2]) {
                  this.css_class="promstat-green";
                } else if (v > this.thresholds[2] && v <= this.thresholds[3]) {
                  this.css_class="promstat-orange";
                } else if (v > this.thresholds[3]) {
                  this.css_class="promstat-red";
                }
              } else if (this.thresholds.length == 2) {
                if (this.thresholds[0] < this.thresholds[1]) {
                  if (v < this.thresholds[0]) {
                    this.css_class="promstat-green";
                  } else if (v >= this.thresholds[0] && v <= this.thresholds[1]) {
                    this.css_class="promstat-orange";
                  } else if (v > this.thresholds[1]) {
                    this.css_class="promstat-red";
                  }
                } else {
                  if (v > this.thresholds[0]) {
                    this.css_class="promstat-green";
                  } else if (v <= this.thresholds[0] && v >= this.thresholds[1]) {
                    this.css_class="promstat-orange";
                  } else if (v < this.thresholds[1]) {
                    this.css_class="promstat-red";
                  }
                }
              }
            }
          } else if (obj.status != 'success') {
            this.value = "N/A";
            this.css_class = "promstat-red";
          } else if (obj.data.result.length == 0) {
            this.value = "N/A";
            this.css_class = "promstat-orange";
          } else {
            this.value = `N/A (${obj.message || '?'})`;
            this.css_class = "promstat-red";
          }
        });
  }

  format_bytes(value: number)
  {
    let units = ['B', 'KB', 'MB', 'GB', 'TB', 'PB', 'EB']
    let idx = 0;
    while (value > 1024 && idx < units.length - 1) {
      value /= 1024;
      idx += 1;
    }
    if (value > 0 && value < 10) {
      return ""; //d3.format('.1f')(value) + units[idx];
    } else {
      return ""; //d3.format('.0f')(value) + units[idx];
    }
  }

  private enable_autorefresh()
  {
    if (this.auto_refresh_subscription)  return;
    this.auto_refresh_subscription =
      interval(this.refresh_interval_sec * 1000).subscribe((num) => {
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
