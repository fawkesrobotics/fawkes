
// Copyright  2018  Tim Niemueller <niemueller@kbsg.rwth-aachen.de>
// License: Apache 2.0 (http://www.apache.org/licenses/LICENSE-2.0)

import { TestBed, inject } from '@angular/core/testing';

import { GoalsService } from './goals.service';

describe('GoalsService', () => {
  beforeEach(() => {
    TestBed.configureTestingModule({
      providers: [GoalsService]
    });
  });

  it('should be created', inject([GoalsService], (service: GoalsService) => {
    expect(service).toBeTruthy();
  }));
});
