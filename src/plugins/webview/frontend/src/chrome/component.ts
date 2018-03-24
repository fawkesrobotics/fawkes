
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// Copyright  2017  The Kubernetes Authors
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import {Component} from '@angular/core';

//import {NotificationsService} from '../common/services/global/notifications';
import { AssetsService } from '../services/global/assets';

@Component({
  selector: 'ff-chrome',
  templateUrl: './template.html',
  styleUrls: ['./style.scss'],
})
export class ChromeComponent {
  loading = false;

  constructor(public assets: AssetsService) {}

  isSystemBannerVisible(): boolean {
    return false;
  }

  getSystemBannerClass(): string {
    return 'ff-bg-warning';
  }

  getSystemBannerMessage(): string {
    return `<b>System is going to be shut down in 5 min...</b>`;
  }
}
