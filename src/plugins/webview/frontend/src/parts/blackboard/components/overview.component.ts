
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
  selected_interfaces = [];
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

  indexof_selected_interface(hash: string, id: string)
  {
    return this.selected_interfaces.findIndex(
      (element) => (element[0] === hash) && (element[1] === id)
    );
  }

  select_interface(hash: string, id: string)
  {
    let index = this.indexof_selected_interface(hash, id);
    if (index == -1) {
      this.selected_interfaces.push([hash, id])
    }
    this.refresh_data(hash, id);
  }

  unselect_interface(hash: string, id: string)
  {
    let index = this.indexof_selected_interface(hash, id);
    if (index >= 0) {
      this.selected_interfaces.splice(index, 1);
    }
  }

  refresh_data(hash: string, id: string)
  {
    if (! this.interfaces[hash] || ! this.interfaces[hash].instances[id]) {
      return;
    }
    this.interfaces[hash].instances[id].loading = true;
    this.zero_message = "Retrieving interface data";

    let type = this.interfaces[hash].type

    this.api_service.get_interface_data(type, id).subscribe(
      (interface_data) => {
        this.interfaces[hash].instances[id].data = interface_data
        this.interfaces[hash].instances[id].loading = false;
      },
      (err) => {
        this.interfaces[hash].instances[id].enabled = false;
        this.interfaces[hash].instances[id].loading = false;
      }
    );
  }
  
  refresh()
  {
    this.loading = true;
    this.zero_message = "Retrieving interface info";

    this.api_service.list_interfaces().subscribe(
      (interfaces) => {
        let updated = []
        let ifs = {}
        if (this.interfaces) {
          ifs = this.interfaces;
        }
        for (let i of interfaces) {
          if (ifs[i.hash]) {
            if (ifs[i.hash].instances[i.id]) {
              ifs[i.hash].instances[i.id].info = i;
            } else {
              ifs[i.hash].instances[i.id] = {info: i, data: null, loading: false};
            }
          } else {
            let instances = {}
            instances[i.id] = {info: i, data: null, loading: false};
            ifs[i.hash] = {"type": i.type, instances: instances};
          }
          updated.push([i.hash, i.id]);
        }
        for (let hash of Object.keys(ifs)) {
          for (let id of Object.keys(ifs[hash].instances)) {
            let index = updated.findIndex(
              (element) => (element[0] === hash) && (element[1] === id));
            let enabled = (index >= 0);
            let index_selected = this.indexof_selected_interface(hash, id);
            if (index_selected >= 0 && enabled && ! ifs[hash].instances[id].enabled) {
              this.refresh_data(hash, id);
            }
            ifs[hash].instances[id].enabled = enabled;
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
