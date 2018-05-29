
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import { Component, OnInit, OnDestroy, ViewChild } from '@angular/core';
import { MatTableDataSource, MatSnackBar, MAT_CHECKBOX_CLICK_ACTION } from '@angular/material';
import { SelectionModel } from '@angular/cdk/collections';

import { BackendConfigurationService } from '../../../services/backend-config/backend-config.service';
import { LockoutService } from '../../../services/lockout/lockout.service';
import { CardListFilterComponent } from '../../../components/filter/component';
import { PluginApiService } from '../services/api.service';
import { Plugin } from '../models/Plugin';
import { PluginOpRequest } from '../models/PluginOpRequest';

import { Observable, interval } from 'rxjs';

@Component({
  selector: 'plugins-overview',
  templateUrl: './overview.component.html',
  styleUrls: ['./overview.component.scss'],
  providers: [{provide: MAT_CHECKBOX_CLICK_ACTION, useValue: 'noop'}]
})
export class PluginOverviewComponent implements OnInit, OnDestroy {

  private backend_subscription = null;

  displayed_columns = ['select', 'name', 'description'];

  auto_refresh_subscription = null;
  loading = false;
  zero_message = "No plugins received.";

  ops_pending = {};
  data_source = new MatTableDataSource();

  @ViewChild(CardListFilterComponent) private readonly card_filter_: CardListFilterComponent;

  selection = new SelectionModel<Plugin>(true, []);
  
  constructor(private snack_bar: MatSnackBar,
              private readonly api_service: PluginApiService,
              private backendcfg: BackendConfigurationService,
              private lockout: LockoutService)
  {}

  ngOnInit() {
    this.refresh();

    this.card_filter_.filterEvent
      .subscribe((query: string) => {
        this.apply_filter(query)
      });

    this.backend_subscription = this.backendcfg.backend_changed.subscribe((b) => {
      this.refresh();
    });
  }

  ngOnDestroy()
  {
    this.backend_subscription.unsubscribe();
    this.backend_subscription = null;
    this.disable_autorefresh();
  }

  refresh()
  {
    this.loading = true;
    this.zero_message = "Retrieving plugins";
    
    this.api_service.list_plugins()
      .subscribe(
        (plugins) => {
          this.ops_pending = {};
          this.data_source.data = plugins;
          this.selection.clear();
          this.selection.select(...plugins.filter(p => p.is_loaded));
          for (let p of plugins)
          if (plugins.length == 0) {
            this.zero_message = "No plugins available";
          }
          this.loading = false;
        },
        (err) => {
          this.data_source.data = [];
          if (err.status == 0) {
            this.zero_message="API server unavailable. Robot down?";
          } else {
            this.zero_message=`Failed to retrieve plugins: ${err.error}`;
          }
          this.loading = false;
        }
      );
  }

  plugin_toggle(plugin: Plugin)
  {
    if (this.lockout.enabled)  return;
    if (this.ops_pending[plugin.name])  return;

    let request: PluginOpRequest = {
      kind: "PluginOpRequest",
      apiVersion: PluginOpRequest.API_VERSION,
      desired_state: plugin.is_loaded ? 'UNLOADED' : 'LOADED'
    };
    this.ops_pending[plugin.name] = true;
    this.api_service.set_plugin_state(plugin.name, request)
      .subscribe(
        (response) => {
          this.ops_pending[plugin.name] = false;
          if (response.state === 'ERROR') {
            this.snack_bar.open(`Loading '${plugin.name} failed: ${response.message}`, '',
                                { duration: 3000 });
          } else if (response.state === 'LOADED') {
            plugin.is_loaded = true;
          } else {
            plugin.is_loaded = false;
          }

          if (plugin.is_loaded) {
            this.selection.select(plugin);
          } else {
            this.selection.deselect(plugin);
          }
        },
        (err) => {
          this.ops_pending[plugin.name] = false;
          if (err.status == 0) {
            this.snack_bar.open(`Loading '${plugin.name} failed: backend not reachable`, '',
                                { duration: 3000 });
          } else if (err.error && err.error.message) {
            this.snack_bar.open(`Loading '${plugin.name} failed: ${err.error.message}`, '',
                                { duration: 3000 });
          } else {
            this.snack_bar.open(`Loading '${plugin.name} failed: ${err.errmsg || err.status}`, '',
                                { duration: 3000 });
          }
        });
  }
  
  private enable_autorefresh()
  {
    if (this.auto_refresh_subscription)  return;
    this.auto_refresh_subscription =
      interval(10000).subscribe((num) => {
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

  apply_filter(filter_value: string) {
    filter_value = filter_value.trim();
    // MatTableDataSource defaults to lowercase matches
    filter_value = filter_value.toLowerCase();
    this.data_source.filter = filter_value;
  }
}
