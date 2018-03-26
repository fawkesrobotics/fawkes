
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import { NgModule } from '@angular/core';

import { SkillerRoutingModule } from './routing.module';
import { SkillerServicesModule } from './service.module';
import { SkillerComponentModule } from './component.module';

@NgModule({
  imports: [
    SkillerRoutingModule,
    SkillerServicesModule,
    SkillerComponentModule,
  ],
  exports: [
    SkillerRoutingModule,
    SkillerServicesModule,
    SkillerComponentModule,
  ],
})
export class SkillerModule { }
