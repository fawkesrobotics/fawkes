
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import { NgModule } from '@angular/core';
import { Routes, RouterModule } from '@angular/router';

import { GoalListComponent } from './components/goal-list.component';
import { GoalDetailComponent } from './components/goal-detail.component';
import { DomainComponent } from './components/domain.component';

const routes: Routes = [
  { path: 'clips-executive', redirectTo: 'clips-executive/goal', pathMatch: 'full' },
  { path: 'clips-executive/goal', component: GoalListComponent },
  { path: 'clips-executive/goal/:id', component: GoalDetailComponent },
  { path: 'clips-executive/domain', component: DomainComponent },
];

@NgModule({
  imports: [RouterModule.forChild(routes)],
  exports: [RouterModule]
})
export class ClipsExecutiveRoutingModule { }
