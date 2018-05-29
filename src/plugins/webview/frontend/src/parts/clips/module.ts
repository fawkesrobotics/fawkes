
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import { NgModule } from '@angular/core';

import { ClipsRoutingModule } from './routing.module';
import { ClipsServicesModule } from './service.module';
import { ClipsComponentModule } from './component.module';

@NgModule({
  imports: [
    ClipsRoutingModule,
    ClipsServicesModule,
    ClipsComponentModule,
  ],
  exports: [
    ClipsRoutingModule,
    ClipsServicesModule,
    ClipsComponentModule,
  ],
})
export class ClipsModule { }
