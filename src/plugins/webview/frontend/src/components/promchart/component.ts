
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import { Component, Input, AfterViewInit, ViewChild,
         OnInit, OnDestroy, HostListener } from '@angular/core';
import { HttpClient } from '@angular/common/http';

import { Observable, forkJoin, interval } from 'rxjs';
import { map } from 'rxjs/operators';

import { BackendConfigurationService } from '../../services/backend-config/backend-config.service';

import * as c3 from 'c3';
import * as d3 from 'd3';

const DEFAULT_Y_AXIS: c3.YAxisConfiguration = {
  padding: { top: 0.01, bottom: 0.0 },
};

const DEFAULT_LEGEND: c3.LegendOptions = {
  // amount of padding to put between each legend element
  padding: 5,
  // define custom height and width for the legend item tile
  item: {
    tile: {
      width: 15,
      height: 2
    }
  }
};

@Component({
  selector: 'prom-chart',
  templateUrl: './template.html',
  styleUrls: ['./style.scss']
})
export class PrometheusChartComponent implements AfterViewInit, OnInit, OnDestroy {
  @Input() time_range?: number = 900;
  @Input() step_sec?: number = 15;
  @Input() query?: string = null;
  @Input() queries?: string[] = [];
  @Input() legend_format?: string = null;;
  @Input() legend_formats?: string[] = [];
  @Input() y_axis?: c3.YAxisConfiguration = Object.assign({}, DEFAULT_Y_AXIS);
  @Input() show_grid?: boolean = true;
  @Input() legend?: c3.LegendOptions = Object.assign({}, DEFAULT_LEGEND);
  @Input() groups?: string[][] = null;
  @Input() group_all?: boolean = true;
  @Input() remove_all_zero?: boolean = true;
  @Input() x_tick_format?: string = '%H:%M:%S';
  @Input() y_tick_format?: string = '.1f';
  @Input() y_min?: number = null;
  @Input() y_max?: number = null;
  @Input() y_tick_count?: number = 4;
  @Input() y_axis_center?: number = null;
  @Input() refresh_interval_sec?: number = 60;

  @ViewChild('chart') chart_elem;

  have_data: boolean = false;
  zero_message: string = "No data received";
  auto_refresh_subscription = null;

  private backend_subscription = null;

  constructor(private backendcfg: BackendConfigurationService,
              private http: HttpClient)
  {}

  ngAfterViewInit()
  {
    if (!this.query && this.queries.length == 0) {
      this.zero_message='No query configured';
    } else {
      if (this.query) {
        this.queries.push(this.query);
        if (this.legend_format) {
          this.legend_formats.push(this.legend_format);
        } else {
          this.legend_formats.push(this.query);
        }
      }

      if (this.queries.length == 0) {
        this.zero_message='No queries configured';
      } else if (this.legend_formats.length != this.queries.length) {
        this.queries = [];
        this.zero_message = '#queries != #legend_formats';
      }

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

    let end   = Math.floor(Date.now() / 1000);
    let start = end - this.time_range;
    let step_sec = this.step_sec;
    let urls = this.queries
      .map((q) =>
           `${this.backendcfg.url_for('prometheus')}/api/v1/query_range?query=${encodeURIComponent(String(q))}&start=${start}&end=${end}&step=${this.step_sec}s`);


    forkJoin(urls.map((u) => this.http.get<any>(u)))
      .pipe(
        map((res: any[]) =>
            res.map((r,i) => this.proc_res(r, start, end, step_sec, this.legend_formats[i])))
      )
      .subscribe(
        (data_arr: any[]) => {
          if (data_arr.length > 0) {

            let timeline: number[] = [];
            for (let i = start; i <= end; i += this.step_sec) {
              timeline.push(i * 1000);
            }

            let columns = [['__x', ...timeline]];
            let groups = this.group_all ? [[]] : this.groups;
            let types = {};

            for (let data of data_arr) {

              for (let d of data) {
                if (d.name in types) {
                  console.warn(`Duplicate data name ${d.name}, overwriting`);
                }
                types[d.name] = 'area';
              }

              let group = [];
              if (this.group_all) {
                for (let d of data) {
                  groups[0].push(d.name);
                }
              }

              for (let d of data) {
                columns.push([d.name, ...d.values]);
              }
            }

            // If we only have the x axis, but no actual data, abort
            if (columns.length == 1) {
              this.zero_message = "No data available";
              return;
            }

            if (! this.y_axis.tick) {
              this.y_axis.tick = {};
            }
            if (this.y_tick_format) {
              if (this.y_tick_format == 'bytes' || this.y_tick_format == 'bytes/s') {
                this.y_axis.tick.format = this.format_bytes_func(this.y_tick_format);
              } else {
                this.y_axis.tick.format = d3.format(this.y_tick_format);
              }
            } else {
              this.y_axis.tick.format = d3.format('.1f');
            }
            if (this.y_min) {
              this.y_axis.min = this.y_min;
            }
            if (this.y_max) {
              this.y_axis.max = this.y_max;
            }
            if (this.y_tick_count) {
              this.y_axis.tick.count = this.y_tick_count + 1;
            }
            if (this.y_axis_center != null) {
              this.y_axis.center = this.y_axis_center;
            }

            if (! this.legend.item) {
              this.legend.item = {};
            }
            this.legend.item.onclick =
              function (d) {
                let shown = this.api.data.shown();
                if (shown.findIndex((de) => de.id == d) < 0) {
                  // we are switching from another activated single data
                  this.api.hide();
                  this.api.show(d);
                } else {
                  if (shown.length <= 1) {
                    // click on zoomed-in data
                    this.api.show();
                  } else {
                    // select data on fully visible chart
                    this.api.hide();
                    this.api.show(d);
                  }
                }
              };

            let chart = c3.generate({
              bindto: this.chart_elem.nativeElement,
              size: {
                height: 200
              },
              data: {
                x: '__x',
                columns: columns,
                types: types,
                groups: groups,
              },
              axis: {
                x: {
                  type: 'timeseries',
                  tick: {
                    format: this.x_tick_format
                  }
                },
                y: this.y_axis
              },
              point: {
                show: false
              },
              grid: {
                x: {
                  show: this.show_grid
                },
                y: {
                  show: this.show_grid
                }
              },
              legend: this.legend
            });
            this.have_data = true;
          } else {
            this.zero_message = "No data available";
          }
        },
        (err) => {
          this.have_data = false;
          if (err.status == 0) {
            this.zero_message="Prometheus server unavailable.";
          } else {
            this.zero_message=`Failed to retrieve data: ${err.error || err.status}`;
          }
        }
      );
  }

  render_label_template(str, labels) {
    var regex = /\(\(\s*(.+?)\s*\)\)/g;
    return str.replace(regex, (m,g) => labels[g] ? labels[g] : g);
  }

  private proc_res(res, start, end, step_sec, legend_format) {
    if (res.status != 'success')  return null;
    let data = [];

    for (let r of  res.data.result) {
      let d = {
        name: this.render_label_template(legend_format, r.metric),
        values: []
      };

      let base_timestamp = start;
      for (let v of r.values) {
        let dp = parseFloat(v[1]);
        if (isNaN(dp)) {
          dp = 0.;
        }

        const timestamp = parseFloat(v[0]);
        for (let t = base_timestamp; t < timestamp; t += step_sec)
        {
          d.values.push(0.);
        }

        base_timestamp = timestamp + step_sec;
        d.values.push(dp);
      }

      for (let t = base_timestamp; t < end; t += step_sec)
      {
        d.values.push(0.);
      }

      data.push(d);
    }

    // filter all-zero metrics
    if (this.remove_all_zero) {
      let to_erase = []
      for (let i = 0; i < res.data.result.length; ++i) {
        if (data[i].values.every((y) => (Math.abs(y) < Number.EPSILON))) {
          to_erase.push(i);
        }
      }
      if (to_erase.length > 0) {
        let erase_names = to_erase.map((e) => data[e].name);
        for (let e of to_erase.sort((a,b) => b - a)) {
          data.splice(e, 1);
        }
      }
    }

    return data;
  }

  format_bytes_func(what: string)
  {
    let units = ['B', 'KB', 'MB', 'GB', 'TB', 'PB', 'EB']
    if (what == "bytes/s") {
      units = ['B/s', 'KB/s', 'MB/s', 'GB/s', 'TB/s', 'PB/s', 'EB/s']
    }
    return function(value: number): string {
      let idx = 0;
      while (Math.abs(value) > 1024 && idx < units.length - 1) {
        value /= 1024;
        idx += 1;
      }
      if (value > 0 && value < 10) {
        return d3.format('.1f')(value) + ' ' + units[idx];
      } else {
        return d3.format('.0f')(value) + ' ' + units[idx];
      }
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
