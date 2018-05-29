
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// Copyright  2017  The Kubernetes Authors
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import {Component} from '@angular/core';

//import {NotificationsService} from '../common/services/global/notifications';
import { AssetsService } from '../services/global/assets';

import { BackendConfigurationService } from '../services/backend-config/backend-config.service';

@Component({
  selector: 'ff-chrome',
  templateUrl: './template.html',
  styleUrls: ['./style.scss'],
})
export class ChromeComponent {
  loading = false;

  constructor(public assets: AssetsService,
              public backend_conf: BackendConfigurationService)
  {}

  isSystemBannerVisible(): boolean {
    return false;
  }

  getSystemBannerClass(): string {
    return 'ff-bg-warning';
  }

  getSystemBannerMessage(): string {
    return `<b>System is going to be shut down in 5 min...</b>`;
  }

  number_symbol(num: number)
  {
    switch (num+1) {
      case 1: return 'looks_one';
      case 2: return 'looks_two';
      case 3: return 'looks_3';
      case 4: return 'looks_4';
      case 5: return 'looks_5';
      case 6: return 'looks_6';
      default: return 'add_box';
    }   
  }
}
