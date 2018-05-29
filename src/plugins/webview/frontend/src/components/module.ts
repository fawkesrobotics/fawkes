
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import { NgModule } from '@angular/core';

import { SharedModule } from '../shared.module';
import { PipesModule } from '../pipes/module';

import { PropertyComponent } from './property/component';
import { HSpaceComponent } from './hspace/component';
import { ListZeroStateComponent } from './zerostate/component';
import { DotGraphComponent } from './dotgraph/component';
import { CardListFilterComponent } from './filter/component';
import { PrometheusChartComponent } from './promchart/component';
import { PrometheusStatComponent } from './promstat/component';

const COMPONENTS = [
  PropertyComponent,
  HSpaceComponent,
  ListZeroStateComponent,
  DotGraphComponent,
  CardListFilterComponent,
  PrometheusChartComponent,
  PrometheusStatComponent,
];

@NgModule({
  imports: [
    SharedModule,
    PipesModule,
  ],
  declarations: COMPONENTS,
  exports: COMPONENTS
})
export class ComponentsModule {}
