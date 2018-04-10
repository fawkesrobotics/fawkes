
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import { NgModule } from '@angular/core';

import { PluginRoutingModule } from './routing.module';
import { PluginServicesModule } from './service.module';
import { PluginComponentModule } from './component.module';

@NgModule({
  imports: [
    PluginRoutingModule,
    PluginServicesModule,
    PluginComponentModule,
  ],
  exports: [
    PluginRoutingModule,
    PluginServicesModule,
    PluginComponentModule,
  ],
})
export class PluginModule { }
