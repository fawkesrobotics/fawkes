
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import { TestBed, inject } from '@angular/core/testing';

import { PluginApiService } from './api.service';

describe('PluginApiService', () => {
  beforeEach(() => {
    TestBed.configureTestingModule({
      providers: [PluginApiService]
    });
  });

  it('should be created', inject([PluginApiService], (service: PluginApiService) => {
    expect(service).toBeTruthy();
  }));
});
