
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import { NgModule } from '@angular/core';
import { CommonModule } from '@angular/common';
import { SharedModule } from '../../shared.module';

import { PluginRoutingModule } from './routing.module';
import { PluginServicesModule } from './service.module';

import { PluginOverviewComponent } from './components/overview.component';

import { ComponentsModule } from '../../components/module';
import { PipesModule } from '../../pipes/module';

@NgModule({
  imports: [
    SharedModule,
    ComponentsModule,
    PipesModule,
    PluginRoutingModule,
    PluginServicesModule,
  ],
  exports: [
    PluginOverviewComponent,
  ],
  declarations: [PluginOverviewComponent]
})
export class PluginComponentModule { }
