
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import { Component, OnInit } from '@angular/core';
import { Observable } from 'rxjs/Observable';
import 'rxjs/add/observable/interval';

import { BehaviorEngineApiService } from '../services/api.service';

@Component({
  selector: 'skiller-overview',
  templateUrl: './overview.component.html',
  styleUrls: ['./overview.component.scss']
})
export class SkillerOverviewComponent implements OnInit {

  loading = false;
  auto_refresh_subscription = null;
  skill = null;
  zero_message = "No graph has been retrieved";
  
  constructor(private api_service: BehaviorEngineApiService)
  {}

  ngOnInit() {
    this.refresh();
  }

  refresh()
  {
    this.loading = true;
    this.zero_message = "Retrieving graph";

    this.api_service.get_skill("active").subscribe(
      (skill) => {
        this.skill = skill;
        this.loading = false;
      },
      (err) => {
        this.skill = null;
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
