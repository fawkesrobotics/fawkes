
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import { Component } from '@angular/core';

@Component({
  selector: 'app-root',
  //templateUrl: './app.component.html',
  template: '<ff-chrome class="ff-light-theme"></ff-chrome>',
  styleUrls: ['./app.component.scss']
})
export class AppComponent {
  title = 'app';
}
