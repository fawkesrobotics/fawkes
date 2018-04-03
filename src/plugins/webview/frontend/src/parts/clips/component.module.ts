
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import { NgModule } from '@angular/core';
import { CommonModule } from '@angular/common';
import { SharedModule } from '../../shared.module';

import { ClipsRoutingModule } from './routing.module';
import { ClipsServicesModule } from './service.module';

import { ClipsEnvComponent } from './components/clips-env.component';

import { ComponentsModule } from '../../components/module';
import { PipesModule } from '../../pipes/module';

@NgModule({
  imports: [
    SharedModule,
    ComponentsModule,
    PipesModule,
    ClipsRoutingModule,
    ClipsServicesModule,
  ],
  exports: [
    ClipsEnvComponent,
  ],
  declarations: [ClipsEnvComponent]
})
export class ClipsComponentModule { }
