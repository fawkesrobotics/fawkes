
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import { NgModule } from '@angular/core';

import { BlackboardRoutingModule } from './routing.module';
import { BlackboardServicesModule } from './service.module';
import { BlackboardComponentModule } from './component.module';

@NgModule({
  imports: [
    BlackboardRoutingModule,
    BlackboardServicesModule,
    BlackboardComponentModule,
  ],
  exports: [
    BlackboardRoutingModule,
    BlackboardServicesModule,
    BlackboardComponentModule,
  ],
})
export class BlackboardModule { }
