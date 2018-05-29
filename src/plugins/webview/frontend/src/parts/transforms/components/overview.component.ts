
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import { Component, OnInit, OnDestroy } from '@angular/core';
import { Observable, interval } from 'rxjs';

import { BackendConfigurationService } from '../../../services/backend-config/backend-config.service';
import { TransformsApiService } from '../services/api.service';
import { TransformsGraph } from '../models/TransformsGraph';

@Component({
  selector: 'transforms-overview',
  templateUrl: './overview.component.html',
  styleUrls: ['./overview.component.scss']
})
export class TransformsOverviewComponent implements OnInit, OnDestroy {

  loading = false;
  auto_refresh_subscription = null;
  dotgraph = null;
  zero_message = "No graph has been retrieved";

  private backend_subscription = null;

  constructor(private api_service: TransformsApiService,
              private backendcfg: BackendConfigurationService)
  {}

  ngOnInit() {
    this.refresh();
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
    this.zero_message = "Retrieving graph";

    this.api_service.get_graph().subscribe(
      (graphmsg) => {
        if (graphmsg.dotgraph != "") {
          this.dotgraph = graphmsg.dotgraph;
        } else {
          this.dotgraph = null;
          this.zero_message = "Received empty transforms graph";
        }
        this.loading = false;
      },
      (err) => {
        this.dotgraph = null;
        if (err.status == 0) {
          this.zero_message="API server unavailable. Robot down?";
        } else {
          this.zero_message=`Failed to retrieve graph: ${err.error}`;
        }
        this.loading = false;
      }
    );
  }

  private enable_autorefresh()
  {
    if (this.auto_refresh_subscription)  return;
    this.auto_refresh_subscription =
      interval(1000).subscribe((num) => {
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
