
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import {NgModule} from '@angular/core';

import { ClipsApiService } from './services/api.service';

@NgModule({
  providers: [ClipsApiService],
})
export class ClipsServicesModule {}
