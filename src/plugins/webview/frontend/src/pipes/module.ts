
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import {NgModule} from '@angular/core';

import {SharedModule} from '../shared.module';

import { DefaultPipe } from './default/default.pipe';
import { SafeHTMLPipe } from './safe/safe_html.pipe';

const PIPES = [
  DefaultPipe,
  SafeHTMLPipe,
];

@NgModule({
  imports: [
    SharedModule,
  ],
  declarations: PIPES,
  exports: PIPES,
})
export class PipesModule {}
