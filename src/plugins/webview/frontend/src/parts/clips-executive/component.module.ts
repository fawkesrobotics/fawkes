
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import { NgModule } from '@angular/core';
import { CommonModule } from '@angular/common';
import { SharedModule } from '../../shared.module';

import { ClipsExecutiveRoutingModule } from './routing.module';
import { ClipsExecutiveServicesModule } from './service.module';

import { GoalListComponent } from './components/goal-list.component';
import { GoalDetailComponent } from './components/goal-detail.component';
import { DomainComponent } from './components/domain.component';

import { ComponentsModule } from '../../components/module';
import { PipesModule } from '../../pipes/module';

@NgModule({
  imports: [
    SharedModule,
    ComponentsModule,
    PipesModule,
    ClipsExecutiveRoutingModule,
    ClipsExecutiveServicesModule,
  ],
  exports: [
    GoalListComponent,
    GoalDetailComponent,
    DomainComponent,
  ],
  declarations: [GoalListComponent, GoalDetailComponent, DomainComponent]
})
export class ClipsExecutiveComponentModule { }
