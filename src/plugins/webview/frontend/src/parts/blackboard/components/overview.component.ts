
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import { Component, OnInit, OnDestroy } from '@angular/core';
import { Observable, interval } from 'rxjs';
import { SessionStorage } from 'ngx-store';

import { BackendConfigurationService } from '../../../services/backend-config/backend-config.service';
import { BlackboardApiService } from '../services/api.service';
import { InterfaceInfo } from '../models/InterfaceInfo';

@Component({
  selector: 'blackboard-overview',
  templateUrl: './overview.component.html',
  styleUrls: ['./overview.component.scss']
})
export class BlackboardOverviewComponent implements OnInit, OnDestroy {

  private backend_subscription = null;

  loading = false;
  auto_refresh_subscription = null;
  @SessionStorage() selected_interfaces = [];
  interfaces = null;
  zero_message = "No graph has been retrieved";

  dotgraph: string = null;
  dotgraph_zero_message: string = 'No blackboard graph available';
  dotgraph_loading: boolean = false;

  known_types = {};
  
  constructor(private api_service: BlackboardApiService,
              private backendcfg: BackendConfigurationService)
  {}

  ngOnInit() {
    this.refresh(true);
    this.backend_subscription = this.backendcfg.backend_changed.subscribe((b) => {
      this.refresh(true);
    });
  }

  ngOnDestroy()
  {
    this.backend_subscription.unsubscribe();
    this.backend_subscription = null;
    this.disable_autorefresh();
  }

  keys(obj) {
    return Object.keys(obj);
  }

  /** Check if interface is OK.
   * An interface is ok if it is in the list of interfaces received from the remote.
   * @param id_hash array of strings with two entries, id and hash
   * @return true if interface can be used, false otherwise
   */
  iok(hash_id: string[]): boolean {
    return ( this.interfaces &&
             hash_id[0] in this.interfaces &&
             hash_id[1] in this.interfaces[hash_id[0]].instances );
  }

  /** Get interface.
   * Assumes that the interface validity has been checked before with iok().
   * @param hash_id array of strings with two entries, id and hash
   * @return interface instance descriptor
   */
  ifc(hash_id: string[]) {
    return this.interfaces[hash_id[0]].instances[hash_id[1]];
  }

  ifcd(hash_id: string[], field: string) {
    let iface = this.ifc(hash_id);
    let idx = iface.info.fields.findIndex(f => f.name == field);
    if (idx < 0) { // Ooops
      console.warn(`Cannot find info for field '${hash_id[0]}-${hash_id[1]}--${field}'`);
      return iface.data.data[field];
    } else {
      if (iface.info.fields[idx].is_array) {
        let rv = [];
        for (let v of iface.data.data[field]) {
          let t = iface.info.fields[idx].type;
          if (t == 'double' || t == 'float') {
            rv.push(v.toFixed(4));
          } else {
            rv.push(v);
          }
        }
        return rv;
      } else {
        let t = iface.info.fields[idx].type;
        if (t == 'double' || t == 'float') {
          return iface.data.data[field].toFixed(4);
        } else {
          return iface.data.data[field];
        }
      }
    }
  }

  indexof_selected_interface(hash: string, id: string)
  {
    return this.selected_interfaces.findIndex(
      (element) => (element[0] === hash) && (element[1] === id)
    );
  }

  select_interface(hash: string, id: string)
  {
    this.dotgraph = null;
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
    if (! this.iok([hash, id])) return;

    this.interfaces[hash].instances[id].loading = true;
    this.zero_message = "Retrieving interface data";

    let type = this.interfaces[hash].type

    this.api_service.get_interface_data(type, id).subscribe(
      (interface_data) => {
        this.interfaces[hash].instances[id].data = interface_data
        this.interfaces[hash].instances[id].loading = false;
      },
      (err) => {
        if (this.iok([hash, id])) {
          this.interfaces[hash].instances[id].enabled = false;
          this.interfaces[hash].instances[id].loading = false;
        }
      }
    );
  }
  
  refresh(refresh_data = false)
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
          if (! (i.hash in this.known_types)) {
            this.known_types[i.hash] = i.type;
          }
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

        if (refresh_data) {
          for (let hash_id of this.selected_interfaces) {
            this.refresh_data(hash_id[0], hash_id[1]);
          }
        }

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

  refresh_graph()
  {
    if (this.dotgraph_loading) return;

    this.dotgraph_loading = true;
    this.dotgraph_zero_message='Retrieving graph';

    this.api_service.get_graph().subscribe(
      (graphmsg) => {
        if (graphmsg.dotgraph != "") {
          this.dotgraph = graphmsg.dotgraph;
        } else {
          this.dotgraph = null;
          this.dotgraph_zero_message = "Received empty blackboard graph";
        }
        this.dotgraph_loading = false;
      },
      (err) => {
        this.dotgraph = null;
        if (err.status == 0) {
          this.dotgraph_zero_message="API server unavailable. Robot down?";
        } else {
          this.dotgraph_zero_message=`Failed to retrieve graph: ${err.error}`;
        }
        this.dotgraph_loading = false;
      });
  }
  
  toggle_graph()
  {
    if (this.dotgraph) {
      this.dotgraph = null;
    } else {
      this.dotgraph = '';
      this.refresh_graph();
    }
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
