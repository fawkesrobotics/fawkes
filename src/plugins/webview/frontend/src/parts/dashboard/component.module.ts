
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import { NgModule } from '@angular/core';
import { CommonModule } from '@angular/common';
import { SharedModule } from '../../shared.module';

import { DashboardServicesModule } from './service.module';

import { DashboardComponent } from './components/dashboard.component';

import { ComponentsModule } from '../../components/module';

@NgModule({
  imports: [
    SharedModule,
    ComponentsModule,
    DashboardServicesModule,
  ],
  exports: [
    DashboardComponent,
  ],
  declarations: [DashboardComponent]
})
export class DashboardComponentModule { }
