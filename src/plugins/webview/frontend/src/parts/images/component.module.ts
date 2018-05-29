
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import { NgModule } from '@angular/core';
import { CommonModule } from '@angular/common';
import { SharedModule } from '../../shared.module';

import { ImageRoutingModule } from './routing.module';
import { ImageServicesModule } from './service.module';

import { ImageOverviewComponent } from './components/overview.component';

import { ComponentsModule } from '../../components/module';
import { PipesModule } from '../../pipes/module';

@NgModule({
  imports: [
    SharedModule,
    ComponentsModule,
    PipesModule,
    ImageRoutingModule,
    ImageServicesModule,
  ],
  exports: [
    ImageOverviewComponent,
  ],
  declarations: [ImageOverviewComponent]
})
export class ImageComponentModule { }
