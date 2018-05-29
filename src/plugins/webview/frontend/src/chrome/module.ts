
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// Copyright  2017  The Kubernetes Authors
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import {NgModule} from '@angular/core';

//import {ComponentsModule} from '../common/components/module';
import { AppRoutingModule } from '../app/app-routing.module';
import {SharedModule} from '../shared.module';

import {ChromeComponent} from './component';
import {NavModule} from './nav/module';
//import {NotificationsComponent} from './notifications/component';
//import {UserPanelComponent} from './userpanel/component';

@NgModule({
  declarations: [
    ChromeComponent,
    //NotificationsComponent,
    //UserPanelComponent,
  ],
  imports: [
    SharedModule,
    AppRoutingModule,
    NavModule,
  ],
  exports: [
    ChromeComponent
  ],
})
export class ChromeModule {}
