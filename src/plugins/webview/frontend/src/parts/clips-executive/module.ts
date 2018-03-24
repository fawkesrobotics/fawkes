
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import { NgModule } from '@angular/core';

import { ClipsExecutiveRoutingModule } from './routing.module';
import { ClipsExecutiveServicesModule } from './service.module';
import { ClipsExecutiveComponentModule } from './component.module';
//import { ApiModule as ClipsExecutiveApiModule } from './api/api.module';

@NgModule({
  imports: [
    ClipsExecutiveRoutingModule,
    ClipsExecutiveServicesModule,
    ClipsExecutiveComponentModule,
    //ClipsExecutiveApiModule,
  ],
  exports: [
    ClipsExecutiveRoutingModule,
    ClipsExecutiveServicesModule,
    ClipsExecutiveComponentModule,
    //ClipsExecutiveApiModule,
  ],
})
export class ClipsExecutiveModule { }
