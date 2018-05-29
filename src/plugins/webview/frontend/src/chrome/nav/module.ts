
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// Copyright  2017  The Kubernetes Authors
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import {NgModule} from '@angular/core';
//import {ComponentsModule} from '../../common/components/module';

import {NavServiceModule} from '../../services/nav/module';
import {SharedModule} from '../../shared.module';
import { AppRoutingModule } from '../../app/app-routing.module';

import {NavComponent} from './component';
import {HamburgerComponent} from './hamburger/component';
import {NavItemComponent} from './item/component';

@NgModule({
  declarations: [NavComponent, NavItemComponent, HamburgerComponent],
  exports: [NavComponent, NavItemComponent, HamburgerComponent],
  imports: [SharedModule, AppRoutingModule, /*ComponentsModule,*/ NavServiceModule]
})
export class NavModule {}
