
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import { NgModule } from '@angular/core';

import { ImageRoutingModule } from './routing.module';
import { ImageServicesModule } from './service.module';
import { ImageComponentModule } from './component.module';

@NgModule({
  imports: [
    ImageRoutingModule,
    ImageServicesModule,
    ImageComponentModule,
  ],
  exports: [
    ImageRoutingModule,
    ImageServicesModule,
    ImageComponentModule,
  ],
})
export class ImageModule { }
