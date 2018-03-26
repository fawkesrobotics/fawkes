
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import { NgModule } from '@angular/core';
import { CommonModule } from '@angular/common';
import { SharedModule } from '../../shared.module';

import { SkillerRoutingModule } from './routing.module';
import { SkillerServicesModule } from './service.module';

import { SkillerOverviewComponent } from './components/overview.component';

import { ComponentsModule } from '../../components/module';
import { PipesModule } from '../../pipes/module';

@NgModule({
  imports: [
    SharedModule,
    ComponentsModule,
    PipesModule,
    SkillerRoutingModule,
    SkillerServicesModule,
  ],
  exports: [
    SkillerOverviewComponent,
  ],
  declarations: [SkillerOverviewComponent]
})
export class SkillerComponentModule { }
