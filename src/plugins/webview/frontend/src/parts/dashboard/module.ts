
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import { NgModule } from '@angular/core';

import { DashboardServicesModule } from './service.module';
import { DashboardComponentModule } from './component.module';

@NgModule({
  imports: [
    DashboardServicesModule,
    DashboardComponentModule,
  ],
  exports: [
    DashboardServicesModule,
    DashboardComponentModule,
  ],
})
export class DashboardModule { }
