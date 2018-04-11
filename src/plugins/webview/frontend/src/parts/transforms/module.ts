
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import { NgModule } from '@angular/core';

import { TransformsRoutingModule } from './routing.module';
import { TransformsServicesModule } from './service.module';
import { TransformsComponentModule } from './component.module';

@NgModule({
  imports: [
    TransformsRoutingModule,
    TransformsServicesModule,
    TransformsComponentModule,
  ],
  exports: [
    TransformsRoutingModule,
    TransformsServicesModule,
    TransformsComponentModule,
  ],
})
export class TransformsModule { }
