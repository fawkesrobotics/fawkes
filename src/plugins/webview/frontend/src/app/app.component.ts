
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import { Component } from '@angular/core';

import { SwUpdateNotifierService } from '../services/update-notifier/update-notifier.service';

@Component({
  selector: 'app-root',
  //templateUrl: './app.component.html',
  template: '<ff-chrome class="ff-light-theme"></ff-chrome>',
  styleUrls: ['./app.component.scss'],
  providers: [SwUpdateNotifierService]
})
export class AppComponent {
  title = 'app';

  constructor(private update_notifier: SwUpdateNotifierService)
  {}
}
