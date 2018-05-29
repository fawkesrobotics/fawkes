
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import { TestBed, inject } from '@angular/core/testing';

import { ClipsExecutiveApiService } from './api.service';

describe('ClipsExecutiveApiService', () => {
  beforeEach(() => {
    TestBed.configureTestingModule({
      providers: [ClipsExecutiveApiService]
    });
  });

  it('should be created', inject([ClipsExecutiveApiService], (service: ClipsExecutiveApiService) => {
    expect(service).toBeTruthy();
  }));
});
