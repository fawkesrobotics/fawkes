
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// Copyright  2017  The Kubernetes Authors
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import {MatDrawer} from '@angular/material';

export class NavService {
  private nav_: MatDrawer;

  toggle(): void {
    this.nav_.toggle();
  }

  setVisibility(isVisible: boolean): void {
    this.nav_.toggle(isVisible);
  }

  setNav(nav: MatDrawer): void {
    this.nav_ = nav;
  }

  isVisible(): boolean {
    return this.nav_.opened;
  }
}
