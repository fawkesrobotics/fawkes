
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import { NgModule } from '@angular/core';
import { Routes, RouterModule } from '@angular/router';

import { SkillerOverviewComponent } from './components/overview.component';

const routes: Routes = [
  { path: 'skiller', component: SkillerOverviewComponent },
];

@NgModule({
  imports: [RouterModule.forChild(routes)],
  exports: [RouterModule]
})
export class SkillerRoutingModule { }
