
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// Copyright  2017  The Kubernetes Authors
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import {Component} from '@angular/core';
import {NavService} from '../../../services/nav/service';

@Component({
  selector: 'ff-nav-hamburger',
  templateUrl: 'template.html',
})
export class HamburgerComponent {
  constructor(private readonly navService_: NavService) {}

  toggle(): void {
    this.navService_.toggle();
  }
}
