
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import {NgModule} from '@angular/core';

//import { HttpClientInMemoryWebApiModule } from 'angular-in-memory-web-api';

import { ClipsExecutiveApiService } from './services/api.service';
//import { ClipsExecutiveInMemoryDataService } from './services/mock-data.service';

@NgModule({
  imports: [
    // The HttpClientInMemoryWebApiModule module intercepts HTTP requests
    // and returns simulated server responses.
    // Remove it when a real server is ready to receive requests.
    //HttpClientInMemoryWebApiModule
    //  .forFeature(ClipsExecutiveInMemoryDataService, {apiBase: "api/clips-executive/"}),
  ],

  providers: [ClipsExecutiveApiService],
})
export class ClipsExecutiveServicesModule {}
